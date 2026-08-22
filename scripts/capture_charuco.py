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


def _tilt_degrees(gray, corners, ids, args, _cache={}):
    """Angle between the board's normal and the camera's optical axis.

    Recovered with solvePnP through a NOMINAL camera. That is circular-looking
    -- using an assumed camera to judge views for calibrating that camera --
    but it is not: tilt is an angle, and it is insensitive to getting the focal
    length wrong. A 30 deg board reads as roughly 30 deg whether fx is 550 or
    700, which is all the binning needs.
    """
    board = _cache.get("board")
    if board is None:
        if hasattr(cv2.aruco, "CharucoBoard_create"):
            board = cv2.aruco.CharucoBoard_create(
                args.cols, args.rows, 0.022, 0.016, get_dictionary(args.dict))
        else:
            board = cv2.aruco.CharucoBoard(
                (args.cols, args.rows), 0.022, 0.016, get_dictionary(args.dict))
        _cache["board"] = board
        f = args.width / (2.0 * np.tan(np.radians(args.nominal_hfov) / 2.0))
        _cache["K"] = np.array([[f, 0, args.width / 2.0],
                                [0, f, args.height / 2.0], [0, 0, 1]])
    try:
        if hasattr(cv2.aruco, "interpolateCornersCharuco"):
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board, minMarkers=1)
        else:
            det = cv2.aruco.CharucoDetector(board)
            cc, ci, _, _ = det.detectBoard(gray)
        if cc is None or len(cc) < 6:
            return None
        obj, img = board.matchImagePoints(cc, ci)
        ok, rvec, _ = cv2.solvePnP(obj, img, _cache["K"], None)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        return float(np.degrees(np.arccos(min(1.0, abs(float(R[2, 2]))))))
    except (cv2.error, AttributeError, TypeError, ValueError):
        return None


def _hud(frame, corners, ids, tilt, bins, kept, args, novel):
    """Live view: what the camera sees, plus what is still MISSING.

    The ROS GUI's four bars say how full each axis is. They do not say what to
    do next, and they do not show tilt as an angle -- which was the axis that
    silently stayed thin here. This names the gap.
    """
    view = frame.copy()
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(view, corners, ids)

    pos = len({(k[0], k[1]) for k in bins})
    size = len({k[2] for k in bins})
    steep = sum(v for k, v in bins.items() if k[3] == 2)

    # Green while this pose is being banked, grey once its bin is full.
    accent = (80, 230, 80) if novel else (170, 170, 170)
    cv2.rectangle(view, (0, 0), (view.shape[1], 74), (0, 0, 0), -1)
    cv2.putText(view, f"kept {kept}/{args.target}   tilt {tilt:4.0f} deg",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, accent, 2)

    need = []
    if pos < 6:
        need.append("EDGES")
    if size < 3:
        need.append("CLOSE+FAR")
    if steep < 8:
        need.append(f"TILT>30 ({steep}/8)")
    msg = "need: " + ", ".join(need) if need else "coverage OK - solve it"
    cv2.putText(view, msg, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (60, 200, 255) if need else (80, 230, 80), 2)
    cv2.putText(view, f"pos {pos}/9  size {size}/3  steep {steep}",
                (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    # A horizon-style tilt gauge: the bar fills as the board leans over.
    x0, y0, w = 8, view.shape[0] - 20, 200
    cv2.rectangle(view, (x0, y0), (x0 + w, y0 + 12), (60, 60, 60), -1)
    frac = max(0.0, min(1.0, tilt / 45.0))
    cv2.rectangle(view, (x0, y0), (x0 + int(w * frac), y0 + 12),
                  (80, 230, 80) if tilt >= 30 else (60, 200, 255), -1)
    cv2.line(view, (x0 + int(w * 30 / 45.0), y0 - 3),
             (x0 + int(w * 30 / 45.0), y0 + 15), (255, 255, 255), 1)
    cv2.putText(view, "30", (x0 + int(w * 30 / 45.0) - 8, y0 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

    cv2.imshow("charuco capture  --  q or Esc to finish", view)
    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        raise KeyboardInterrupt


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
    ap.add_argument("--cols", type=int, default=9,
                    help="board squares across (see docs/CALIBRATION.md)")
    ap.add_argument("--rows", type=int, default=11)
    ap.add_argument("--nominal-hfov", type=float, default=60.0,
                    help="rough horizontal FOV, used ONLY to judge board tilt")
    ap.add_argument("--min-markers", type=int, default=12,
                    help="reject a view with fewer than this many markers")
    ap.add_argument("--per-bin", type=int, default=2,
                    help="frames to keep per coverage bin")
    ap.add_argument("--target", type=int, default=45,
                    help="stop once this many frames are kept")
    ap.add_argument("--show", action="store_true",
                    help="live window with the detections and a coverage HUD. "
                         "Being able to see what the camera sees is most of "
                         "what the ROS calibration GUI was giving you; this "
                         "adds the tilt angle, which that GUI does not show.")
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

            # Defaults so that EVERY path below can draw the HUD and print the
            # status line. The live view has to keep updating while you are
            # still hunting for the board -- a window that only repaints once
            # the board is already detected is useless for aiming.
            tilt = 0.0
            novel = False

            # `not len(corners)` as well as the threshold: with --min-markers 0
            # a frame containing no markers passes the test and then reaches
            # np.concatenate([]), which raises "need at least one array to
            # concatenate" and takes the capture loop down with it.
            if n >= args.min_markers and len(corners):
                pts = np.concatenate([c.reshape(-1, 2) for c in corners])
                cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
                # Fraction of the frame the board spans -- the "Size" axis.
                span = (pts[:, 0].max() - pts[:, 0].min()) * \
                       (pts[:, 1].max() - pts[:, 1].min())
                fill = span / float(args.width * args.height)

                # Tilt, from the board's actual POSE. The first version
                # inferred it from the bounding box aspect ratio, which is not
                # tilt at all: a board held square-on but off to one side has a
                # skewed bounding box, and one rotated about the axis pointing
                # at the camera has none. On a real set this metric rated
                # "tilt 2/2", the true spread was median 7 deg and max 22.7 --
                # NOT ONE view past 30 -- and the calibration was degenerate,
                # cx wandering to -49 between halves of the same data.
                measured = _tilt_degrees(gray, corners, ids, args)
                if measured is not None:
                    tilt = measured
                    key = (int(cx / (args.width / 3.0)),
                           int(cy / (args.height / 3.0)),
                           0 if fill < 0.25 else (1 if fill < 0.55 else 2),
                           0 if tilt < 15.0 else (1 if tilt < 30.0 else 2))
                    novel = bins.get(key, 0) < args.per_bin
                    if novel:
                        bins[key] = bins.get(key, 0) + 1
                        kept += 1
                        cv2.imwrite(
                            os.path.join(args.out, f"view_{kept:03d}.png"),
                            frame)

            if args.show:
                _hud(frame, corners, ids, tilt, bins, kept, args, novel)

            # Report unconditionally. The first version only printed when a
            # frame was KEPT, so a run that detected nothing produced no output
            # whatsoever and was indistinguishable from a hang.
            now = time.time()
            if now - last_report > 0.5:
                last_report = now
                pos = len({(k[0], k[1]) for k in bins})
                size = len({k[2] for k in bins})
                steep = sum(v for k, v in bins.items() if k[3] == 2)
                note = "" if n >= args.min_markers else \
                       f"  <- need {args.min_markers}+, board not in view?"
                print(f"\rseen {received:5d}  kept {kept:3d}/{args.target}  "
                      f"markers {n:2d} tilt {tilt:4.0f}deg{note}  "
                      f"[pos {pos}/9 size {size}/3 steep {steep}]   ",
                      end="", flush=True)

    except KeyboardInterrupt:
        print()
    finally:
        if args.show:
            cv2.destroyAllWindows()

    _chown_to_dir_owner(args.out)

    pos = len({(k[0], k[1]) for k in bins})
    size = len({k[2] for k in bins})
    tiltb = len({k[3] for k in bins})
    steep = sum(1 for k, v in bins.items() if k[3] == 2 for _ in range(v))
    print(f"\n\n{kept} frames kept, {received} frames seen, "
          f"in {args.out}/")
    if received == 0:
        print("  NO FRAMES ARRIVED AT ALL -- this is a plumbing problem, not")
        print("  a coverage one. Check the publisher and ROS_DOMAIN_ID.")
    print(f"  position {pos}/9   size {size}/3   tilt {tiltb}/3  "
          f"({steep} views past 30 deg)")
    if pos < 6 or size < 3 or steep < 8:
        print("  THIN COVERAGE -- expect an unstable solve:")
        if pos < 6:
            print("    position: get the board to the frame EDGES (fixes cx, cy)")
        if size < 3:
            print("    size: needs close AND far (separates fx from depth)")
        if steep < 8:
            print(f"    tilt: only {steep} views past 30 deg, want 8+. This is")
            print("      the one people skip, and the one that decouples focal")
            print("      length from the principal point.")
    print("\nNow solve it somewhere fast:")
    print(f"  python3 scripts/calibrate_offline.py --images {args.out}")


if __name__ == "__main__":
    main()
