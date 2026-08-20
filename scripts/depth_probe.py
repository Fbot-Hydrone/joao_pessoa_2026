#!/usr/bin/env python3
"""
depth_probe — print per-pixel depth in metres from a 32FC1 depth Image topic.

Run from the ros2 distrobox (it mounts $HOME, so this file is already visible
there — nothing needs to go into the docker image):

    distrobox enter ros2
    cd ~/work/competition/joao_pessoa_2026
    source scripts/env.sh        # strips conda, sources ROS, ROS_DOMAIN_ID=42
    python3 scripts/depth_probe.py

env.sh matters: with conda active, `python3` is the 3.11 from biguasim_env,
while /opt/ros/humble is built for 3.10 — rclpy then fails looking for
_rclpy_pybind11.cpython-311-*.so, which does not exist. env.sh drops conda from
PATH so `python3` resolves to /usr/bin/python3.10 and rclpy imports.

    python3 scripts/depth_probe.py                     # centre pixel, raw topic
    python3 scripts/depth_probe.py --px 320,240 --px 100,50
    python3 scripts/depth_probe.py --grid 5x5
    python3 scripts/depth_probe.py --topic /zed/zed_node/depth/depth_registered
    python3 scripts/depth_probe.py --once --dump frame.npy

Values are ALREADY in metres — biguasim's sensors.py:406 divides the raw
centimetre buffer by 100 before it ever reaches the ROS encoder, and
DepthMapEncoder (sensor_data_encode.py:513-536) ships those floats unchanged.
Do not scale again.

Two topics carry the same frame with different invalid-pixel conventions:
  /biguasim/uav0_id0/DepthCamera            sky = 655.04 m (float16 buffer max)
  /zed/zed_node/depth/depth_registered      sky = NaN (zed_mimic_node.py:200)
Both are reported here as "no return".
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

DEFAULT_TOPIC = "/biguasim/uav0_id0/DepthCamera"
SKY_SENTINEL_M = 655.0


def decode_32fc1(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (32FC1) -> (H, W) float32 array of metres."""
    if msg.encoding != "32FC1":
        raise ValueError(f"expected 32FC1, got {msg.encoding!r}")
    if msg.is_bigendian and sys.byteorder == "little":
        raise ValueError("big-endian payload on a little-endian host")

    arr = np.frombuffer(msg.data, dtype=np.float32)
    # step is the row stride in BYTES; slice the padding off if there is any
    stride = msg.step // 4
    if stride != msg.width:
        return arr.reshape(msg.height, stride)[:, : msg.width]
    return arr.reshape(msg.height, msg.width)


def describe(value: float) -> str:
    if not np.isfinite(value) or value >= SKY_SENTINEL_M:
        return "  no return"
    return f"{value:8.3f} m"


class DepthProbe(Node):
    def __init__(self, args):
        super().__init__("depth_probe")
        self.args = args
        self.frames = 0

        # The bridge publishes RELIABLE (ardubridge_node.py:147 passes a bare
        # depth of 10). BEST_EFFORT here matches that and would also match a
        # best-effort publisher, so this subscriber connects either way.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, args.topic, self.on_depth, qos)
        self.get_logger().info(f"listening on {args.topic}")

    def on_depth(self, msg: Image):
        depth = decode_32fc1(msg)
        self.frames += 1

        valid = depth[np.isfinite(depth) & (depth < SKY_SENTINEL_M)]
        pct = 100.0 * valid.size / depth.size
        print(f"\n--- frame {self.frames}  {msg.width}x{msg.height}  "
              f"valid {valid.size}/{depth.size} ({pct:.1f}%)")
        if valid.size:
            print(f"    nearest {valid.min():.3f} m   farthest {valid.max():.3f} m   "
                  f"median {np.median(valid):.3f} m")

        for u, v in self.args.px or [(msg.width // 2, msg.height // 2)]:
            if 0 <= u < msg.width and 0 <= v < msg.height:
                # row index is v (y), column index is u (x)
                print(f"    ({u:4d},{v:4d}) = {describe(float(depth[v, u]))}")
            else:
                print(f"    ({u:4d},{v:4d}) = out of bounds")

        if self.args.grid:
            cols, rows = self.args.grid
            us = np.linspace(0, msg.width - 1, cols).astype(int)
            vs = np.linspace(0, msg.height - 1, rows).astype(int)
            print("    grid (m), columns left->right:")
            for v in vs:
                cells = []
                for u in us:
                    d = float(depth[v, u])
                    ok = np.isfinite(d) and d < SKY_SENTINEL_M
                    cells.append(f"{d:7.2f}" if ok else "      -")
                print(f"      v={v:4d} " + " ".join(cells))

        if self.args.dump:
            np.save(self.args.dump, depth)
            print(f"    saved {self.args.dump}  (np.load -> (H, W) float32 metres)")

        if self.args.once:
            raise SystemExit(0)


def parse_px(text):
    u, v = text.split(",")
    return int(u), int(v)


def parse_grid(text):
    cols, rows = text.lower().split("x")
    return int(cols), int(rows)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    p.add_argument("--px", type=parse_px, action="append", metavar="U,V",
                   help="pixel to probe, repeatable (default: image centre)")
    p.add_argument("--grid", type=parse_grid, metavar="COLSxROWS",
                   help="also print a coarse sample grid, e.g. 5x5")
    p.add_argument("--dump", metavar="FILE.npy",
                   help="save each frame as a numpy array of metres")
    p.add_argument("--once", action="store_true", help="exit after one frame")
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = DepthProbe(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
