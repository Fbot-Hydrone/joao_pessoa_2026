#!/usr/bin/env python3
"""
feature_map_node — a sparse world map built from the ZED's own visual features.

The pad map (pad_map_node) records the few things the drone wants to land on.
This node records everything else it has looked at: ORB features from the ZED's
RGB stream, back-projected through the registered depth into the world frame and
accumulated in a voxel grid, plus a 2-D grid of how thoroughly each patch of
floor has been observed.

    /zed/zed_node/rgb/image_rect_color  ─┐
    /zed/zed_node/depth/depth_registered ├─> ORB -> back-project -> voxel hash
    /zed/zed_node/rgb/camera_info       ─┤
    /mavros/local_position/pose         ─┘
                                              |
                              /hydrone/map/features   sensor_msgs/PointCloud2
                              /hydrone/map/coverage   nav_msgs/OccupancyGrid

Why this and not the VO node
----------------------------
hydrone_bringup/visual_odometry_node already detects ORB features and
back-projects them — but it is in the flight-critical loop: the FCU navigates on
its output with GPS disabled, so anything added there can cost the vehicle its
position estimate. This node is a pure consumer. It subscribes, it never
publishes a pose, and it holds no TF, so the worst it can do when it misbehaves
is use CPU.

What the coverage grid is for
-----------------------------
It answers "where have I actually looked?", which is the question a search has to
ask before it can claim there is nothing left to find. It is published for the
operator and for RViz; the mission's search pattern does NOT consume it yet —
pad_mission_node flies a fixed bounded spiral. Feeding coverage back into the
planner (skip a leg that is already well seen, re-fly one that is not) is the
natural next step and is deliberately left out until the fixed pattern has been
flown end to end.

Frames follow pad_detector_node exactly: poses come from
/mavros/local_position/pose so the map lands in the same frame the setpoints
use, and TF is consulted only for the constant camera mount.
"""

import array
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

import tf2_ros

# A leaf numpy-only helper from the vision package — no ROS node, no
# cv_bridge. See image_convert's module docstring for why cv_bridge is not
# used anywhere in this stack.
from hydrone_vision.image_convert import bgr_image_to_numpy, depth_image_to_numpy


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix.

    Deliberately duplicated from hydrone_vision/pad_detector_node rather than
    imported: a mapping node has no business importing the vision package's ROS
    entry point — and everything that module pulls in at import time — for
    twelve lines of algebra.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FeatureMapNode(Node):

    def __init__(self, **kwargs):
        super().__init__("feature_map", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("in_rgb", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("in_depth",
                               "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("in_info", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("optical_frame",
                               "zed_left_camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cloud_topic", "/hydrone/map/features")
        self.declare_parameter("coverage_topic", "/hydrone/map/coverage")

        self.declare_parameter("max_features", 400)
        self.declare_parameter("voxel_size", 0.15)
        self.declare_parameter("coverage_res", 0.5)
        # Depth gating: the sim's far plane and the ZED's own noise floor both
        # produce garbage outside a sane band.
        self.declare_parameter("min_depth", 0.4)
        self.declare_parameter("max_depth", 20.0)
        # Hard caps. An unbounded map on an unbounded plane is how a long flight
        # ends in swap.
        self.declare_parameter("max_voxels", 200000)
        self.declare_parameter("max_coverage_cells", 100000)
        self.declare_parameter("publish_hz", 1.0)
        # Process at most this many frames per second; mapping does not need the
        # full camera rate and the ZED stream is shared with the VO.
        self.declare_parameter("process_hz", 4.0)

        p = lambda n: self.get_parameter(n).value
        self.optical_frame = p("optical_frame")
        self.base_frame = p("base_frame")
        self.voxel = float(p("voxel_size"))
        self.cov_res = float(p("coverage_res"))
        self.min_depth = float(p("min_depth"))
        self.max_depth = float(p("max_depth"))
        self.max_voxels = int(p("max_voxels"))
        self.max_cov = int(p("max_coverage_cells"))
        self.world_frame = "map"

        rate = float(p("process_hz"))
        self.min_period_ns = int(1e9 / rate) if rate > 0.0 else 0

        # ── State ───────────────────────────────────────────────────────────
        self.orb = cv2.ORB_create(nfeatures=int(p("max_features")))
        self.K: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.pose: PoseStamped | None = None
        self.R_base_opt: np.ndarray | None = None
        self.t_base_opt: np.ndarray | None = None
        self._last_ns = 0
        self._full_warned = False

        # Voxel hash: (i, j, k) -> hit count. A dict, not a dense array: the map
        # is overwhelmingly empty and its extent is not known in advance.
        self.voxels: dict[tuple[int, int, int], int] = {}
        self.coverage: dict[tuple[int, int], int] = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_cloud = self.create_publisher(PointCloud2, p("cloud_topic"), 1)
        self.pub_cov = self.create_publisher(OccupancyGrid,
                                             p("coverage_topic"), 1)

        self.create_subscription(CameraInfo, p("in_info"), self._cb_info,
                                 sensor_qos)
        self.create_subscription(Image, p("in_depth"), self._cb_depth,
                                 sensor_qos)
        self.create_subscription(PoseStamped, p("pose_topic"), self._cb_pose,
                                 sensor_qos)
        self.create_subscription(Image, p("in_rgb"), self._cb_rgb, sensor_qos)

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._publish)

        self.get_logger().info(
            f"feature_map ready — {self.voxel * 100:.0f} cm voxels, "
            f"{self.cov_res * 100:.0f} cm coverage cells.")

    # ────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _cb_depth(self, msg: Image):
        self.depth = depth_image_to_numpy(msg)

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_rgb(self, msg: Image):
        if self.min_period_ns:
            now = self.get_clock().now().nanoseconds
            if now - self._last_ns < self.min_period_ns:
                return
            self._last_ns = now

        if self.K is None or self.depth is None or self.pose is None:
            return
        if not self._ensure_mount_tf():
            return

        gray = cv2.cvtColor(bgr_image_to_numpy(msg), cv2.COLOR_BGR2GRAY)
        keypoints = self.orb.detect(gray, None)
        if not keypoints:
            return

        pts = np.array([k.pt for k in keypoints], dtype=np.float64)
        world = self._to_world(pts)
        if world is None:
            return
        self._accumulate(world)

    # ────────────────────────────────────────────────────────────────────────
    # Geometry
    # ────────────────────────────────────────────────────────────────────────

    def _to_world(self, pts: np.ndarray) -> np.ndarray | None:
        """Nx2 pixels -> Mx3 world points, dropping any without usable depth."""
        depth = self.depth
        h, w = depth.shape[:2]
        u = np.rint(pts[:, 0]).astype(np.int32)
        v = np.rint(pts[:, 1]).astype(np.int32)
        keep = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        if not keep.any():
            return None
        u, v = u[keep], v[keep]

        d = depth[v, u]
        good = (np.isfinite(d) & (d >= self.min_depth) & (d <= self.max_depth))
        if not good.any():
            return None
        u, v, d = u[good], v[good], d[good].astype(np.float64)

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            return None
        # Optical frame: Z forward along the axis, X right, Y down.
        local = np.stack([(u - cx) / fx * d, (v - cy) / fy * d, d], axis=1)

        q = self.pose.pose.orientation
        R_world_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_base = np.array([self.pose.pose.position.x,
                           self.pose.pose.position.y,
                           self.pose.pose.position.z])
        R_world_opt = R_world_base @ self.R_base_opt
        p_cam = p_base + R_world_base @ self.t_base_opt
        return local @ R_world_opt.T + p_cam

    def _accumulate(self, points: np.ndarray):
        """Fold world points into the voxel hash and the coverage grid."""
        full = len(self.voxels) >= self.max_voxels
        if full and not self._full_warned:
            self.get_logger().warn(
                f"feature map hit max_voxels ({self.max_voxels}); new cells are "
                "being dropped. Raise the cap or the voxel size.")
            self._full_warned = True

        keys = np.floor(points / self.voxel).astype(np.int64)
        cov_keys = np.floor(points[:, :2] / self.cov_res).astype(np.int64)

        for (i, j, k), (ci, cj) in zip(map(tuple, keys), map(tuple, cov_keys)):
            if not full or (i, j, k) in self.voxels:
                self.voxels[(i, j, k)] = self.voxels.get((i, j, k), 0) + 1
            if len(self.coverage) < self.max_cov or (ci, cj) in self.coverage:
                self.coverage[(ci, cj)] = self.coverage.get((ci, cj), 0) + 1

    def _ensure_mount_tf(self) -> bool:
        if self.R_base_opt is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().warn(
                f"waiting for TF {self.base_frame} -> {self.optical_frame}",
                throttle_duration_sec=5.0)
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        self.t_base_opt = np.array([t.x, t.y, t.z])
        self.R_base_opt = quat_to_matrix(q.x, q.y, q.z, q.w)
        return True

    # ────────────────────────────────────────────────────────────────────────
    # Output
    # ────────────────────────────────────────────────────────────────────────

    def _publish(self):
        if not self.voxels:
            return
        stamp = self.get_clock().now().to_msg()
        self.pub_cloud.publish(self._cloud(stamp))
        if self.coverage:
            self.pub_cov.publish(self._grid(stamp))

    def _cloud(self, stamp) -> PointCloud2:
        """One point at the centre of every occupied voxel."""
        keys = np.array(list(self.voxels.keys()), dtype=np.float64)
        xyz = (keys + 0.5) * self.voxel

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.height = 1
        msg.width = xyz.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = xyz.astype(np.float32).tobytes()
        return msg

    def _grid(self, stamp) -> OccupancyGrid:
        """Coverage as an OccupancyGrid: 0 = barely seen, 100 = looked at hard.

        Not an obstacle map — the value is observation count, log-compressed so
        one heavily-textured wall does not flatten everything else to zero.
        """
        keys = np.array(list(self.coverage.keys()), dtype=np.int64)
        counts = np.array(list(self.coverage.values()), dtype=np.float64)
        i_min, j_min = keys[:, 0].min(), keys[:, 1].min()
        i_max, j_max = keys[:, 0].max(), keys[:, 1].max()
        width = int(i_max - i_min + 1)
        height = int(j_max - j_min + 1)

        grid = np.full(width * height, -1, dtype=np.int8)
        value = np.clip(np.log1p(counts) / math.log1p(50.0), 0.0, 1.0) * 100.0
        idx = (keys[:, 1] - j_min) * width + (keys[:, 0] - i_min)
        grid[idx] = value.astype(np.int8)

        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.info.resolution = self.cov_res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(i_min) * self.cov_res
        msg.info.origin.position.y = float(j_min) * self.cov_res
        msg.info.origin.orientation.w = 1.0
        # array.array('b') hits rclpy's int8[] fast path; a Python list would be
        # copied element by element (same trap as the image encoders).
        msg.data = array.array('b', grid.tobytes())
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FeatureMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
