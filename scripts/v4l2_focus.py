#!/usr/bin/env python3
"""Find the belly camera's focus and lock it. No ROS.

    python3 scripts/v4l2_focus.py --device /dev/video2            # sweep
    python3 scripts/v4l2_focus.py --device /dev/video2 --set 15   # lock only

The C922 has autofocus and forgets every setting at power-up, so the value
found here goes to the launch (down_cam_focus:=N) and down_cam_usb_node
re-applies it on every start. Over a plain floor or a pad filling the frame the
autofocus has no contrast to work with and hunts.

The sweep needs TEXTURE at the distance you care about, and the arena floor
has none. Put the on-screen ChArUco board (charuco_screen.py) in the middle of
the frame at that distance, run the sweep, and the best value is printed and
left on the camera. Run it at hover height (~1.2 m) and at sweep height; pick a
value that is good at both. See docs/CALIBRATION.md §7.

Controls go through raw V4L2 ioctls, not v4l2-ctl or cv2 properties, so this
works in the container as well and means the same on every OpenCV version.
"""
import argparse
import fcntl
import os
import struct
import sys
import time

# struct v4l2_control {__u32 id; __s32 value;}, _IOWR('V', 27/28, ...)
VIDIOC_G_CTRL = 0xC008561B
VIDIOC_S_CTRL = 0xC008561C
CID_FOCUS_ABSOLUTE = 0x009A090A
CID_FOCUS_AUTO = 0x009A090C


def get_ctrl(fd, cid):
    buf = bytearray(struct.pack("Ii", cid, 0))
    fcntl.ioctl(fd, VIDIOC_G_CTRL, buf)
    return struct.unpack("Ii", buf)[1]


def set_ctrl(fd, cid, value):
    fcntl.ioctl(fd, VIDIOC_S_CTRL, bytearray(struct.pack("Ii", cid, value)))


def lock_focus(device, value):
    """Autofocus off, focus_absolute = value. Returns (autofocus, focus) read
    back from the camera. Opens its own fd: controls work alongside a stream.
    Autofocus must go off FIRST or the camera ignores the absolute value."""
    fd = os.open(device, os.O_RDWR)
    try:
        set_ctrl(fd, CID_FOCUS_AUTO, 0)
        set_ctrl(fd, CID_FOCUS_ABSOLUTE, int(value))
        return get_ctrl(fd, CID_FOCUS_AUTO), get_ctrl(fd, CID_FOCUS_ABSOLUTE)
    finally:
        os.close(fd)


def sharpness(gray):
    """Laplacian variance over the middle half of the frame, divided by the
    image variance so auto-exposure changing the contrast does not move it."""
    import cv2
    h, w = gray.shape
    roi = gray[h // 4:3 * h // 4, w // 4:3 * w // 4]
    return cv2.Laplacian(roi, cv2.CV_64F).var() / max(roi.var(), 1e-6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", required=True)
    ap.add_argument("--set", type=int, default=None,
                    help="just lock focus at this value and exit")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--stop", type=int, default=100,
                    help="C922: 0 = far, 250 = closest; the useful end for "
                         "0.5-3 m is the low one")
    ap.add_argument("--step", type=int, default=5)
    ap.add_argument("--settle", type=float, default=0.6,
                    help="seconds for the lens to get there")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    fd = os.open(args.device, os.O_RDWR)
    try:
        get_ctrl(fd, CID_FOCUS_AUTO)
    except OSError as e:
        sys.exit(f"{args.device} has no focus control ({e}). Wrong device? "
                 "v4l2-ctl --list-devices")
    finally:
        os.close(fd)

    if args.set is not None:
        auto, got = lock_focus(args.device, args.set)
        print(f"autofocus={auto} focus_absolute={got}")
        sys.exit(0 if (auto == 0 and got == args.set) else 1)

    import cv2
    import numpy as np
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.device}")
    # same mode the drone runs, so the score is for the image it flies with
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.read()[0]:
        sys.exit("no frames")

    scores = []
    try:
        for value in range(args.start, args.stop + 1, args.step):
            auto, got = lock_focus(args.device, value)
            if auto != 0 or got != value:
                print(f"{value:4d}  camera holds autofocus={auto} "
                      f"focus={got}, skipped")
                continue
            t_end = time.time() + args.settle
            while time.time() < t_end:      # drain frames taken mid-move
                cap.read()
            s = []
            for _ in range(args.frames):
                ok, frame = cap.read()
                if ok:
                    s.append(sharpness(cv2.cvtColor(frame,
                                                    cv2.COLOR_BGR2GRAY)))
            if s:
                scores.append((value, float(np.median(s))))
                print(f"{value:4d}  {scores[-1][1]:8.4f}", flush=True)
    finally:
        cap.release()

    if not scores:
        sys.exit("the camera refused every focus value in the range")
    top = max(v for _, v in scores)
    print()
    for value, v in scores:
        print(f"{value:4d}  {'#' * int(40 * v / top)}")
    best = max(scores, key=lambda t: t[1])[0]
    auto, got = lock_focus(args.device, best)
    print(f"\nsharpest: {best}  (left on the camera: autofocus={auto} "
          f"focus_absolute={got})")
    print(f"launch:   down_cam_focus:={best}")


if __name__ == "__main__":
    main()
