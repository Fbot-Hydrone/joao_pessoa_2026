#!/usr/bin/env python3
"""Capture ChArUco views for an OFFLINE calibration. No GUI, no ROS.

    python3 scripts/capture_charuco.py --out /repo/calib_images

Why this exists: `cameracalibrator` does capture and solve in one GUI process,
on the machine holding the camera. On a Tegra X1 the solve is the problem --
measured at 18+ minutes and still running, with the GUI frozen throughout
because camera_calibrator.py:298 calls do_calibration() straight from the mouse
callback, on the same thread as cv2.imshow.

Splitting the two puts the slow half on a fast machine and, more importantly,
keeps the IMAGES. A GUI calibration that goes wrong has to be re-waved from
scratch; a directory of frames can be re-solved as often as you like.

Coverage is what makes a calibration good, so this tracks it explicitly instead
of asking you to watch four bars: it bins each detected view by where the board
sits in frame, how much of the frame it fills, and how far it is tilted, and it
only keeps a frame that lands in a bin still short of views. That is the same
idea as the GUI's X/Y/Size/Skew bars, printed as a table.
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np


def get_dictionary(name):
    ids = {"4x4_250": cv2.aruco.DICT_4X4_250, "5x5_250": cv2.aruco.DICT_5X5_250,
           "6x6_250": cv2.aruco.DICT_6X6_250, "7x7_250": cv2.aruco.DICT_7X7_250,
           "aruco_orig": cv2.aruco.DICT_ARUCO_ORIGINAL}
    d = ids[name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(d)
    return cv2.aruco.Dictionary_get(d)


def detect_markers(gray, dictionary):
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=cv2.aruco.DetectorParameters_create())
    return corners, ids


def _device_source(args):
    """Frames straight off the V4L2 node. Exclusive: one opener at a time."""
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.device}\n"
                 "  If something else already holds it -- a running "
                 "calibrator, or\n"
                 "  jetson_up.sh --sources -- use --from-topic "
                 "/down_cam/image_raw instead.")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    try:
        while True:
            ok, frame = cap.read()
            yield frame if ok else None
    finally:
        cap.release()


def _topic_source(args):
    """Frames off a ROS topic, so this can run alongside whatever owns the
    camera. Deliberately BEST_EFFORT with depth 1: dropping frames is correct
    here -- we want a spread of distinct views, not every one of them."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("charuco_capture")
    box = {}

    def cb(msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("bgr8", "rgb8"):
            img = buf.reshape(msg.height, msg.width, 3)
            box["f"] = img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img
        elif msg.encoding == "mono8":
            box["f"] = cv2.cvtColor(buf.reshape(msg.height, msg.width),
                                    cv2.COLOR_GRAY2BGR)

    node.create_subscription(
        Image, args.from_topic, cb,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST, depth=1))
    print(f"subscribed to {args.from_topic}")
    waited = 0
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except ExternalShutdownException:
                # How rclpy reports SIGINT/SIGTERM from its own signal
                # handler. It is the normal way this program ends, not a
                # fault, so it must not print a traceback over the summary.
                break
            frame = box.pop("f", None)
            if frame is None:
                # A silent subscriber looks identical to a camera seeing
                # nothing, so say which it is. Counted in TICKS, not by
                # accumulating 0.2 into a float and testing `== 5.0` -- 0.2 is
                # not representable in binary, so that comparison never fires.
                waited += 1
                if waited in (25, 75):
                    print(f"\n  no frames yet after {waited // 5}s on "
                          f"{args.from_topic} -- is the publisher running, "
                          f"and is ROS_DOMAIN_ID the same?")
            else:
                waited = 0
            yield frame
    finally:
        node.destroy_node()
        # rclpy installs its own signal handler; on SIGINT/SIGTERM the context
        # is already down by the time this runs, and shutting it down twice
        # raises "rcl_shutdown already called".
        if rclpy.ok():
            rclpy.shutdown()


def _chown_to_dir_owner(out):
    """Hand the output back to whoever owns the tree it was written into.

    The container runs as root and the output directory is a bind mount of the
    user's checkout, so every frame lands root-owned and the user cannot then
    delete or move them without sudo. Match the owner of the PARENT directory,
    which is the checkout itself and therefore the right answer whoever ran it.
    """
    if os.geteuid() != 0:
        return
    try:
        parent = os.path.dirname(os.path.abspath(out)) or "."
        st = os.stat(parent)
        if st.st_uid == 0:
            return                      # genuinely root's tree; leave it
        os.chown(out, st.st_uid, st.st_gid)
        for name in os.listdir(out):
            os.chown(os.path.join(out, name), st.st_uid, st.st_gid)
    except OSError:
        pass                            # cosmetic; never fail the capture


def main():
    # Without a TTY -- `docker run` with no -t, a pipe, nohup -- Python
    # BLOCK-buffers stdout, so progress output sits in the buffer and the
    # program appears to be doing nothing at all. reconfigure() is the
    # in-process equivalent of python3 -u.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass

    ap = argparse.ArgumentParser()
    ap.add_argument("--device",
                    default="/dev/v4l/by-path/"
                            "platform-70090000.xusb-usb-0:2.2:1.0-video-index0")
    ap.add_argument("--out", default="calib_images")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--dict", default="4x4_250")
    ap.add_argument("--min-markers", type=int, default=12,
                    help="reject a view with fewer than this many markers")
    ap.add_argument("--per-bin", type=int, default=2,
                    help="frames to keep per coverage bin")
    ap.add_argument("--target", type=int, default=45,
                    help="stop once this many frames are kept")
    ap.add_argument("--from-topic", default="",
                    help="subscribe to a ROS image topic instead of opening "
                         "the V4L2 device, e.g. /down_cam/image_raw. V4L2 "
                         "capture is EXCLUSIVE -- if anything else already "
                         "holds the camera (a running calibrator, or "
                         "jetson_up.sh --sources) the device cannot be opened "
                         "a second time, but its published images are free to "
                         "anyone on the same DDS domain.")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    dictionary = get_dictionary(args.dict)

    frames = _topic_source(args) if args.from_topic else _device_source(args)

    bins = {}
    kept = 0
    received = 0
    last_report = 0.0
    print(f"Saving to {args.out}/  --  Ctrl-C when the table stops filling.",
          flush=True)
    print("Move the board: all four corners of the frame, close AND far,")
    print("and tilted 30-45 degrees. Frames that add nothing are discarded.\n")

    try:
        for frame in frames:
            if kept >= args.target:
                break
            if frame is None:
                continue
            args.width, args.height = frame.shape[1], frame.shape[0]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = detect_markers(gray, dictionary)
            n = 0 if ids is None else len(ids)
            received += 1

            # Report unconditionally. The first version only printed when a
            # frame was KEPT, so a run that detected nothing produced no
            # output whatsoever and was indistinguishable from a hang. The
            # number that matters when nothing is being kept is how many
            # markers are actually visible.
            now = time.time()
            if now - last_report > 0.5:
                last_report = now
                pos = len({(k[0], k[1]) for k in bins})
                size = len({k[2] for k in bins})
                tilt = len({k[3] for k in bins})
                note = "" if n >= args.min_markers else \
                       f"  <- need {args.min_markers}+, board not in view?"
                print(f"\rseen {received:5d}  kept {kept:3d}/{args.target}  "
                      f"markers now {n:2d}{note}  "
                      f"[pos {pos}/9 size {size}/3 tilt {tilt}/2]   ",
                      end="", flush=True)
            # `not corners` as well as the threshold: with --min-markers 0 a
            # frame containing no markers passes the test and then reaches
            # np.concatenate([]), which raises "need at least one array to
            # concatenate" and takes the capture loop down with it.
            if n < args.min_markers or not len(corners):
                continue

            pts = np.concatenate([c.reshape(-1, 2) for c in corners])
            cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
            # Fraction of the frame the board spans -- the "Size" axis.
            span = ((pts[:, 0].max() - pts[:, 0].min()) *
                    (pts[:, 1].max() - pts[:, 1].min()))
            fill = span / float(args.width * args.height)
            # Tilt, as the departure of the board's bounding quad from a
            # rectangle: the ratio of its two diagonals. Square-on is 1.0.
            w = pts[:, 0].max() - pts[:, 0].min()
            h = pts[:, 1].max() - pts[:, 1].min()
            aspect = max(w, h) / max(1.0, min(w, h))

            key = (int(cx / (args.width / 3.0)),
                   int(cy / (args.height / 3.0)),
                   0 if fill < 0.25 else (1 if fill < 0.55 else 2),
                   0 if aspect < 1.25 else 1)
            if bins.get(key, 0) >= args.per_bin:
                continue
            bins[key] = bins.get(key, 0) + 1
            kept += 1
            cv2.imwrite(os.path.join(args.out, f"view_{kept:03d}.png"), frame)


    except KeyboardInterrupt:
        print()

    _chown_to_dir_owner(args.out)

    pos = len({(k[0], k[1]) for k in bins})
    size = len({k[2] for k in bins})
    tilt = len({k[3] for k in bins})
    print(f"\n\n{kept} frames kept, {received} frames seen, "
          f"in {args.out}/")
    if received == 0:
        print("  NO FRAMES ARRIVED AT ALL -- this is a plumbing problem, not")
        print("  a coverage one. Check the publisher and ROS_DOMAIN_ID.")
    print(f"  position bins {pos}/9   size bins {size}/3   tilt bins {tilt}/2")
    if pos < 5 or size < 2 or tilt < 2:
        print("  THIN COVERAGE. size<2 leaves fx/fy poorly separated from")
        print("  distance; tilt<2 leaves distortion poorly constrained.")
    print("\nNow solve it somewhere fast:")
    print(f"  python3 scripts/calibrate_offline.py --images {args.out}")


if __name__ == "__main__":
    main()
