#!/usr/bin/env python3
"""
odom_error_node — measures visual-odometry drift against BiguaSim ground truth.

Subscribes to the two odometry streams that sources_sim.launch.py produces and
writes their difference to a CSV at the repo root:

  in:  /zed/zed_node/odom      nav_msgs/Odometry  visual_odometry_node's ESTIMATE
       /zed/zed_node/odom_GT   nav_msgs/Odometry  zed_mimic's GROUND TRUTH
  out: <repo>/odom_error_<YYYYmmdd_HHMMSS>.csv   (+ optional throttled stdout)

SIM-ONLY: odom_GT exists only under BiguaSim. On the real drone there is no
oracle, so this node has nothing to compare against.

Why the poses are anchored before subtracting
---------------------------------------------
The two streams do NOT share an origin. visual_odometry_node fixes its odom
origin at identity on its first RGB frame, while odom_GT is BiguaSim's raw world
pose — whatever spawn point and heading config.yaml gives. Subtracting them
directly yields a large constant offset (and a rotated frame) that buries the
drift we actually want to see.

So both streams are re-expressed as MOTION SINCE A COMMON ANCHOR, and the
comparison is a full rigid one:

    T_err(t) = [T_gt0⁻¹ · T_gt(t)]⁻¹ · [T_vo0⁻¹ · T_vo(t)]

The anchor is the first SYNCHRONIZED PAIR, not each stream's first message:
zed_mimic and visual_odometry_node start at different times, and anchoring each
stream independently would bake that startup offset into every later sample.

Why pairs are matched on stamp, not on arrival
----------------------------------------------
The streams run at very different rates — config.yaml throttles the cameras to
20 Hz while DynamicsSensor is unthrottled (~82 Hz sim loop) — so a callback that
just used "the latest GT I happen to hold" would compare poses up to a full
camera period apart and report motion as if it were drift.

Both stamps come from zed_mimic's own clock (its _cb_odom and _cb_rgb both call
get_clock().now()), so they are directly comparable. This node therefore buffers
GT, and on each VO message picks the GT sample with the NEAREST stamp, rejecting
the pair outright when the residual exceeds max_dt. That residual is written to
every row as dt_sync, so sync quality is visible in the data rather than assumed.
"""

import csv
import math
import os
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry

# Console print cadence when print_diff is on. The CSV always gets every sample;
# this only throttles stdout so it stays readable next to zed_mimic and VO logs.
PRINT_PERIOD = 1.0

# Below this travelled distance, drift-as-a-percentage is noise (dividing a
# near-zero error by a near-zero path length), so it is left blank in the CSV.
MIN_PATH_FOR_DRIFT_PCT = 0.10

CSV_COLUMNS = [
    "t_rel",          # s since the anchor pair
    "dt_sync",        # s, vo_stamp - gt_stamp for this pair (sync residual)
    "gt_x", "gt_y", "gt_z",        # ground-truth motion since anchor (odom frame)
    "vo_x", "vo_y", "vo_z",        # VO motion since anchor (odom frame)
    "err_x", "err_y", "err_z",     # vo - gt, per axis, in the odom frame
    "err_norm",       # m, magnitude of the position error
    "err_yaw_deg",    # deg, VO yaw minus GT yaw, wrapped to [-180, 180]
    "err_angle_deg",  # deg, total angular error (geodesic, all three axes)
    "path_len",       # m, distance actually travelled per ground truth
    "drift_pct",      # 100 * err_norm / path_len
]


def _mat_from_odom(msg: Odometry) -> np.ndarray:
    """nav_msgs/Odometry pose -> 4x4 homogeneous transform."""
    q = msg.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    # Normalize defensively: a denormalized quaternion would silently scale the
    # rotation block and corrupt every downstream error.
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / n, y / n, z / n, w / n

    T = np.eye(4)
    T[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    p = msg.pose.pose.position
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def _inv(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform — transpose, not a general matrix inverse."""
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def _yaw(R: np.ndarray) -> float:
    """Yaw (rotation about Z) of a rotation matrix, in radians."""
    return math.atan2(R[1, 0], R[0, 0])


def _geodesic_angle(R: np.ndarray) -> float:
    """Total rotation angle of R about its own axis, in radians (always >= 0)."""
    # clip guards acos against trace values pushed just outside [-1, 1] by
    # floating-point error.
    return math.acos(float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _find_repo_root(start: str) -> str | None:
    """Walk up from `start` looking for the repo root.

    colcon builds this package with --symlink-install, so __file__ resolves back
    into the source tree (/ws/src/... in the container) rather than the install
    space — which means walking up from here reaches the real repo root.
    """
    d = os.path.abspath(start)
    while True:
        if (os.path.isdir(os.path.join(d, "src"))
                and os.path.isfile(os.path.join(d, "docker-compose.yml"))):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


class OdomErrorNode(Node):

    def __init__(self):
        super().__init__("odom_error_node")

        # ── Parameters ──────────────────────────────────────────────────────
        # The two streams to compare. Defaults match sources_sim.launch.py:
        # visual_odometry_node owns /zed/zed_node/odom, zed_mimic is pointed at
        # /zed/zed_node/odom_GT via its out_odom parameter.
        self.declare_parameter("in_odom", "/zed/zed_node/odom")
        self.declare_parameter("in_odom_gt", "/zed/zed_node/odom_GT")
        # Max |vo_stamp - gt_stamp| accepted for a pair, in seconds. GT runs at
        # the ~82 Hz sim loop (12 ms period), so nearest-neighbour matching is
        # worst-case ~6 ms off; 20 ms accepts that while still rejecting real
        # dropouts. Raise it only if rejections are high AND you accept that the
        # error then includes genuine motion between the two samples.
        self.declare_parameter("max_dt", 0.02)
        # How much GT history to keep for stamp matching. VO publishes tens of ms
        # after the frame stamp it carries, so its match is always already in the
        # buffer; 2 s is slack for a stalled VO callback.
        self.declare_parameter("buffer_seconds", 2.0)
        # Where the CSV goes. Empty = auto-detect the repo root from __file__.
        self.declare_parameter("log_dir", "")
        # Echo a throttled summary line to stdout as well as the CSV.
        self.declare_parameter("print_diff", False)

        self.max_dt = float(self.get_parameter("max_dt").value)
        self.buffer_seconds = float(self.get_parameter("buffer_seconds").value)
        self.print_diff = bool(self.get_parameter("print_diff").value)

        # ── State ───────────────────────────────────────────────────────────
        self.gt_buf: deque = deque()   # (stamp_sec, 4x4), stamp-ordered
        self.anchor_gt: np.ndarray | None = None
        self.anchor_vo: np.ndarray | None = None
        self.t0 = 0.0
        self.path_len = 0.0
        self.prev_gt_rel: np.ndarray | None = None
        self.n_pairs = 0
        self.n_rejected = 0
        self.max_err = 0.0
        self.last_row: dict | None = None
        self.next_print = 0.0

        # ── CSV ─────────────────────────────────────────────────────────────
        self.csv_path = self._open_log()

        # ── I/O ─────────────────────────────────────────────────────────────
        p = lambda n: self.get_parameter(n).value
        # QoS depth 10 matches both publishers (default reliable/volatile).
        self.create_subscription(Odometry, p("in_odom_gt"), self._cb_gt, 10)
        self.create_subscription(Odometry, p("in_odom"), self._cb_vo, 10)

        self.get_logger().info(
            f"Odom error ready — {p('in_odom')} vs {p('in_odom_gt')}, "
            f"max_dt={self.max_dt * 1e3:.0f} ms, print={self.print_diff}")
        self.get_logger().info(f"Logging to {self.csv_path}")

        # One-shot sanity check: an empty CSV is almost always a topic that never
        # arrived, which is worth saying out loud rather than leaving to guesswork.
        self.startup_timer = self.create_timer(5.0, self._check_startup)

    # ─────────────────────────────────────────────────────────────────────────

    def _open_log(self) -> str:
        log_dir = self.get_parameter("log_dir").value
        if not log_dir:
            log_dir = _find_repo_root(__file__)
            if log_dir is None:
                log_dir = os.getcwd()
                self.get_logger().warn(
                    f"Repo root not found from {__file__}; logging to cwd {log_dir}")
        os.makedirs(log_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(log_dir, f"odom_error_{stamp}.csv")
        self.csv_file = open(path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_COLUMNS)
        return path

    def _check_startup(self):
        self.startup_timer.cancel()
        if self.n_pairs:
            return
        got_gt = "yes" if self.gt_buf else "NO"
        self.get_logger().warn(
            f"No synchronized pairs after 5 s (ground truth received: {got_gt}, "
            f"pairs rejected on dt: {self.n_rejected}). Check both nodes are up: "
            f"ros2 topic hz /zed/zed_node/odom /zed/zed_node/odom_GT")

    # ─────────────────────────────────────────────────────────────────────────
    # Ground truth: buffer by stamp so VO can match against it
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_gt(self, msg: Odometry):
        t = _stamp_to_sec(msg.header.stamp)
        self.gt_buf.append((t, _mat_from_odom(msg)))
        cutoff = t - self.buffer_seconds
        while self.gt_buf and self.gt_buf[0][0] < cutoff:
            self.gt_buf.popleft()

    # ─────────────────────────────────────────────────────────────────────────
    # VO: the sparse stream, so it drives the comparison
    # ─────────────────────────────────────────────────────────────────────────

    def _cb_vo(self, msg: Odometry):
        if not self.gt_buf:
            return

        t_vo = _stamp_to_sec(msg.header.stamp)
        # ~164 buffered samples at 82 Hz over 2 s — a linear scan at the 20 Hz VO
        # rate is negligible, and keeps the matching obvious.
        t_gt, T_gt = min(self.gt_buf, key=lambda e: abs(e[0] - t_vo))
        dt = t_vo - t_gt
        if abs(dt) > self.max_dt:
            self.n_rejected += 1
            # Throttled: a persistent mismatch would otherwise spam every frame.
            self.get_logger().warn(
                f"Dropping pair: nearest GT is {dt * 1e3:+.1f} ms away "
                f"(max_dt {self.max_dt * 1e3:.0f} ms)",
                throttle_duration_sec=5.0)
            return

        T_vo = _mat_from_odom(msg)

        if self.anchor_gt is None:
            self.anchor_gt = T_gt
            self.anchor_vo = T_vo
            self.t0 = t_vo
            self.get_logger().info(
                f"Anchored on first synchronized pair (dt {dt * 1e3:+.1f} ms)")

        # Motion since the anchor, for each stream independently.
        rel_gt = _inv(self.anchor_gt) @ T_gt
        rel_vo = _inv(self.anchor_vo) @ T_vo
        # Full rigid difference: what VO got wrong, as a transform.
        T_err = _inv(rel_gt) @ rel_vo

        # Per-axis error in the odom frame (interpretable as "VO thinks it is
        # this far north/east/up of where it really is"). Its magnitude is the
        # same as T_err's translation; only the frame it is expressed in differs.
        err = rel_vo[:3, 3] - rel_gt[:3, 3]
        err_norm = float(np.linalg.norm(err))

        err_yaw = math.degrees(
            (_yaw(rel_vo[:3, :3]) - _yaw(rel_gt[:3, :3]) + math.pi) % (2 * math.pi) - math.pi)
        err_angle = math.degrees(_geodesic_angle(T_err[:3, :3]))

        # Path length from ground truth — the honest denominator for drift rate.
        if self.prev_gt_rel is not None:
            self.path_len += float(np.linalg.norm(rel_gt[:3, 3] - self.prev_gt_rel))
        self.prev_gt_rel = rel_gt[:3, 3].copy()

        drift_pct = (100.0 * err_norm / self.path_len
                     if self.path_len >= MIN_PATH_FOR_DRIFT_PCT else None)

        t_rel = t_vo - self.t0
        self.csv.writerow([
            f"{t_rel:.4f}", f"{dt:.6f}",
            f"{rel_gt[0, 3]:.4f}", f"{rel_gt[1, 3]:.4f}", f"{rel_gt[2, 3]:.4f}",
            f"{rel_vo[0, 3]:.4f}", f"{rel_vo[1, 3]:.4f}", f"{rel_vo[2, 3]:.4f}",
            f"{err[0]:.4f}", f"{err[1]:.4f}", f"{err[2]:.4f}", f"{err_norm:.4f}",
            f"{err_yaw:.3f}", f"{err_angle:.3f}",
            f"{self.path_len:.4f}", "" if drift_pct is None else f"{drift_pct:.2f}",
        ])
        # Flush per row: a sim killed with Ctrl-C should still leave a complete
        # file. At 20 Hz this costs nothing.
        self.csv_file.flush()

        self.n_pairs += 1
        self.max_err = max(self.max_err, err_norm)
        self.last_row = {"t": t_rel, "err_norm": err_norm, "drift_pct": drift_pct}

        if self.print_diff and t_rel >= self.next_print:
            self.next_print = t_rel + PRINT_PERIOD
            print(f"[{t_rel:7.1f}s] err {err_norm:6.3f} m "
                  f"(x {err[0]:+.3f}  y {err[1]:+.3f}  z {err[2]:+.3f})   "
                  f"yaw {err_yaw:+7.2f} deg   ang {err_angle:6.2f} deg",
                  flush=True)

    # ─────────────────────────────────────────────────────────────────────────

    def close(self):
        if self.n_pairs:
            last = self.last_row
            pct = "n/a" if last["drift_pct"] is None else f"{last['drift_pct']:.2f}%"
            self.get_logger().info(
                f"{self.n_pairs} pairs, {self.n_rejected} rejected on dt | "
                f"final error {last['err_norm']:.3f} m over {self.path_len:.2f} m "
                f"travelled ({pct}) | peak {self.max_err:.3f} m")
        else:
            self.get_logger().warn("No synchronized pairs were logged")
        self.get_logger().info(f"Wrote {self.csv_path}")
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = OdomErrorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
