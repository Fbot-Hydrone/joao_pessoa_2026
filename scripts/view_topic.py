#!/usr/bin/env python3
"""Show a ROS image topic in a window. A stand-in for rqt_image_view.

    python3 scripts/view_topic.py /hydrone/pads/down/debug_image

rqt_image_view is NOT in the drone's image -- it pulls in most of rqt and Qt
for one window. This needs only cv2, which is already there for the detectors,
and the same GTK3 backend the calibration GUI used.

Keys:  q or Esc quit,  s save the current frame to /tmp
"""
import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray, for the encodings this stack emits."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img
    if msg.encoding == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width),
                            cv2.COLOR_GRAY2BGR)
    if msg.encoding in ("32FC1", "16UC1"):
        # Depth. Scale to something visible rather than showing a black frame.
        dtype = np.float32 if msg.encoding == "32FC1" else np.uint16
        d = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        d = np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
        hi = float(np.percentile(d[d > 0], 95)) if np.any(d > 0) else 1.0
        return cv2.applyColorMap(
            np.clip(d / max(hi, 1e-6) * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_TURBO)
    raise ValueError(f"unsupported encoding {msg.encoding!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--scale", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("view_topic")
    box = {"n": 0}

    def cb(msg):
        try:
            box["img"] = to_bgr(msg)
            box["n"] += 1
        except ValueError as exc:
            print(exc, file=sys.stderr)

    # BEST_EFFORT depth 1: image publishers here use sensor QoS, and a RELIABLE
    # subscriber would simply never match them -- a black window and no error.
    node.create_subscription(
        Image, args.topic, cb,
        QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST, depth=1))
    print(f"subscribed to {args.topic}  (q/Esc quit, s save)")

    saved = 0
    waited = 0
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except ExternalShutdownException:
                break
            img = box.pop("img", None)
            if img is None:
                waited += 1
                if waited in (100, 300):
                    print(f"  no frames after {waited // 20}s on {args.topic} "
                          f"-- is it publishing, and does ROS_DOMAIN_ID match?")
                continue
            waited = 0
            if args.scale != 1.0:
                img = cv2.resize(img, None, fx=args.scale, fy=args.scale)
            cv2.imshow(args.topic, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                saved += 1
                path = f"/tmp/view_{saved:03d}.png"
                cv2.imwrite(path, img)
                print(f"  wrote {path}")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"\n{box['n']} frames received")


if __name__ == "__main__":
    main()
