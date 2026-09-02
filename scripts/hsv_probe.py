#!/usr/bin/env python3
"""
hsv_probe — measure the ACTUAL HSV of the pad in a camera topic, so the
detector's thresholds can be compared against the image instead of guessed at.

Run from the ros2 distrobox, same as depth_probe.py:

    distrobox enter ros2
    cd ~/work/competition/joao_pessoa_2026
    source scripts/env.sh        # strips conda, sources ROS, ROS_DOMAIN_ID=42
    python3 scripts/hsv_probe.py

    python3 scripts/hsv_probe.py                       # down cam, blue band
    python3 scripts/hsv_probe.py --topic /zed/zed_node/rgb/image_rect_color
    python3 scripts/hsv_probe.py --roi 260,180,120,120  # x,y,w,h in pixels
    python3 scripts/hsv_probe.py --dump frame.png       # save what it measured

WHY THIS EXISTS. The blue band in phase1.launch.py is

    blue_hsv_low = [95, 30, 50]        # H>=95, S>=30, V>=50

and the comment beside it records the pad measuring S 37-75 when those numbers
were set (2026-08-18, lossless frame, 3 m hover). That is SEVEN POINTS of
margin at the low end. Overexposure walks a pixel toward white, and white is
S=0 — so a pad that loses eight points of saturation stops existing for the
detector, with no error anywhere: the node only publishes when it detects, so
a washed-out pad and an empty floor produce the same silence.

This prints the distribution, not a single number, because that is what decides
the question: if the 10th percentile of S over the pad is below 30, the
threshold is cutting into real pad pixels and the image is the problem.

WHAT TO COMPARE AGAINST. The `blue pixels` block is the honest one — it is
measured over pixels the detector's own band accepts. The `ROI` block is
measured over everything in the window and is what tells you the pad has gone
pale, because it includes the pixels the band is rejecting.
"""

import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs opencv: pip install opencv-python-headless, or use the "
             "ros2 distrobox where cv2 is already present")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

# The detector's own band, so the probe reports against the number that is
# actually in force rather than a number typed here. Keep in sync with
# phase1.launch.py's blue_hsv_low / pad_detector_node's blue_hsv_high.
BLUE_LOW = (95, 30, 50)
BLUE_HIGH = (135, 255, 255)


def _to_bgr(msg):
    """The image as HxWx3 uint8 BGR, whatever the encoder shipped."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
        return img[:, :, ::-1] if enc == "rgba8" else img
    if enc == "mono8":
        img = buf.reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unhandled encoding {msg.encoding!r}")


def _stats(name, chan):
    """Percentiles, because the tail is what the threshold cuts into."""
    if chan.size == 0:
        return f"  {name:2s}  (no pixels)"
    p = np.percentile(chan, [1, 10, 50, 90, 99])
    return (f"  {name:2s}  min {chan.min():3.0f}  p1 {p[0]:3.0f}  "
            f"p10 {p[1]:3.0f}  median {p[2]:3.0f}  p90 {p[3]:3.0f}  "
            f"p99 {p[4]:3.0f}  max {chan.max():3.0f}")


class HsvProbe(Node):
    def __init__(self, args):
        super().__init__("hsv_probe")
        self.args = args
        self.seen = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, args.topic, self._cb, qos)
        self.get_logger().info(f"listening on {args.topic}")

    def _cb(self, msg):
        try:
            bgr = _to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            rclpy.shutdown()
            return

        if self.args.roi:
            x, y, w, h = self.args.roi
            roi = bgr[y:y + h, x:x + w]
            where = f"ROI x={x} y={y} {w}x{h}"
        else:
            roi, where = bgr, f"whole frame {msg.width}x{msg.height}"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(BLUE_LOW, np.uint8),
                           np.array(BLUE_HIGH, np.uint8))
        n_blue = int((mask > 0).sum())
        frac = n_blue / float(mask.size) if mask.size else 0.0

        print(f"\n=== frame {self.seen + 1}  {where}  encoding {msg.encoding}")
        print(f"  pixels accepted by the detector's blue band "
              f"{BLUE_LOW}..{BLUE_HIGH}: {n_blue} ({100 * frac:.2f}%)")

        print("  ROI, every pixel:")
        for i, name in enumerate(("H", "S", "V")):
            print(_stats(name, hsv[:, :, i].ravel()))

        if n_blue:
            print("  blue pixels only:")
            sel = mask > 0
            for i, name in enumerate(("H", "S", "V")):
                print(_stats(name, hsv[:, :, i][sel]))
        else:
            # The interesting case. Say what it would have taken.
            s_hi = np.percentile(hsv[:, :, 1].ravel(), 99)
            v_hi = np.percentile(hsv[:, :, 2].ravel(), 99)
            print("  NOTHING passed the band. Best in frame: "
                  f"S p99 {s_hi:.0f} (band needs >= {BLUE_LOW[1]}), "
                  f"V p99 {v_hi:.0f} (needs >= {BLUE_LOW[2]}).")
            if s_hi < BLUE_LOW[1]:
                print("  -> SATURATION is what fails. That is the overexposure "
                      "signature: the colour has been washed toward white.")

        # How much of the frame is simply blown out. A pad that is clipped to
        # 255 has no colour left to threshold on, whatever the band says.
        v = hsv[:, :, 2].ravel()
        blown = float((v >= 250).mean())
        print(f"  clipped highlights (V >= 250): {100 * blown:.1f}% of the ROI")

        if self.args.dump:
            cv2.imwrite(self.args.dump, roi)
            print(f"  wrote {self.args.dump}")

        self.seen += 1
        if self.seen >= self.args.count:
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/down_cam/image_raw")
    ap.add_argument("--roi", type=lambda s: tuple(int(v) for v in s.split(",")),
                    help="x,y,w,h — measure only this window, e.g. the pad")
    ap.add_argument("--count", type=int, default=1,
                    help="how many frames to report (default 1)")
    ap.add_argument("--dump", help="write the measured region to this PNG")
    args = ap.parse_args()
    if args.roi and len(args.roi) != 4:
        ap.error("--roi wants x,y,w,h")

    rclpy.init()
    node = HsvProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
