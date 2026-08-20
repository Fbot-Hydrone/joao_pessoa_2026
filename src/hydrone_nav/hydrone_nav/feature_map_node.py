#!/usr/bin/env python3
"""
feature_map_node — a world map built by back-projecting the ZED's depth image.

The pad map (pad_map_node) records the few things the drone wants to land on.
This node records everything else it has looked at: the ZED's registered depth,
sampled on a stride grid, back-projected into the world frame and accumulated in
a voxel grid, plus a 2-D grid of how thoroughly each patch of floor has been
observed.

    /zed/zed_node/depth/depth_registered ─┐
    /zed/zed_node/rgb/camera_info        ├─> sample -> back-project -> voxel hash
    /zed/zed_node/odom                   ─┘
                                              |
                              /hydrone/map/features   sensor_msgs/PointCloud2
                              /hydrone/map/coverage   nav_msgs/OccupancyGrid

Why depth and not ORB features
------------------------------
This node used to key the map on ORB keypoints, and on the competition arena
that produced nothing usable. MEASURED on a live sim frame (2026-08-20), drone
on the ground facing the maze wall at 4.86 m:

    ORB keypoints found:  46  (of a 400 cap)
    their v range:        216..246 of 480 rows

Every keypoint landed in a 30-pixel band. The arena is matte white wall on
blown-out white floor under a smooth sky, so the only intensity gradient in the
frame is the horizon line — ORB has nothing else to grip. The map was a
horizontal line of points, never a surface.

Worse, a corner detector puts its keypoints exactly ON depth discontinuities,
which is the single worst place to sample a depth image. The wall sits at
4.86 m, but the depth under those 46 keypoints ranged 5.36..18.07 m: each ORB
patch straddles wall-and-background, so the rounded pixel returns the wall or
whatever is behind it, essentially at random. Those are "flying pixels", and
they are why the old cloud looked like noise rather than a wall.

Depth pixels have the opposite bias — they are dense exactly where surfaces are
— so the map now samples depth directly on a stride grid and explicitly REJECTS
the discontinuities that ORB used to seek out. See `_edge_mask`.

Why this and not the VO node
----------------------------
hydrone_bringup/visual_odometry_node also consumes this camera — but it is in
the flight-critical loop: the FCU navigates on its output with GPS disabled, so
anything added there can cost the vehicle its position estimate. This node is a
pure consumer. It subscribes, it never publishes a pose, and it holds no TF, so
the worst it can do when it misbehaves is use CPU.

What the coverage grid is for
-----------------------------
It answers "where have I actually looked?", which is the question a search has to
ask before it can claim there is nothing left to find. It is published for the
operator and for RViz; the mission's search pattern does NOT consume it yet —
pad_mission_node flies a fixed bounded spiral. Feeding coverage back into the
planner (skip a leg that is already well seen, re-fly one that is not) is the
natural next step and is deliberately left out until the fixed pattern has been
flown end to end.

The map is only as good as the pose it is folded into. This node reads the ZED's
odometry, so any drift in it smears the map by exactly that much — it does not
and cannot correct for it. TF is consulted only for the constant camera mount,
matching pad_detector_node.

The pose comes from the ZED's odometry rather than the FCU because the ZED
produces exactly one pose per camera frame, carrying that frame's own
timestamp — so each depth image is back-projected with the attitude it was
actually taken at. See the `odom_topic` parameter for the measurements behind
that choice. The cloud is published in the odometry's frame (`odom`), which is
continuous; `map` steps whenever the EKF corrects and would tear the cloud at
every correction. map_odom_node publishes map->odom for display in `map`.
"""

import array
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

import tf2_ros

# A leaf numpy-only helper from the vision package — no ROS node, no
# cv_bridge. See image_convert's module docstring for why cv_bridge is not
# used anywhere in this stack.
from hydrone_vision.image_convert import depth_image_to_numpy


def _stamp_ns(stamp) -> int:
    """builtin_interfaces/Time -> nanoseconds."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


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
        self.declare_parameter("in_depth",
                               "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("in_info", "/zed/zed_node/rgb/camera_info")
        # Pose source. The ZED's own odometry, NOT /mavros/local_position/pose.
        #
        # visual_odometry_node stamps its odometry with the IMAGE's timestamp
        # (`self._publish(msg.header.stamp)`), and zed_mimic gives RGB and depth
        # an identical stamp, so there is exactly one pose per camera frame and
        # its stamp matches the depth frame's EXACTLY. The pairing stops being
        # an approximation and becomes an identity.
        #
        # MEASURED 2026-08-20, why the FCU pose cannot do this job here:
        #   /zed/zed_node/odom            2.47 Hz  (== the camera rate)
        #   /mavros/local_position/pose   1.02 Hz
        # and the FCU rate is not raisable — every MAVLink stream sat at exactly
        # 1.00 Hz and asking for 30 Hz produced 1.69 Hz, i.e. ArduPilot SITL is
        # running at ~5% of real time. Back-projecting a pitched frame against a
        # second-old attitude tilts the reconstructed ground: at a 10 m ground
        # range one degree of attitude is 0.175 m of height, and the airframe
        # pitches to fly forward, so the floor arrives as a set of planes each
        # tilted by its own frame's attitude error.
        #
        # This also matches the real drone, where the ZED's odometry is what
        # feeds MAVROS in the first place — reading it here takes the pose from
        # the source rather than from a slower copy downstream of it.
        self.declare_parameter("odom_topic", "/zed/zed_node/odom")
        self.declare_parameter("optical_frame",
                               "zed_left_camera_optical_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cloud_topic", "/hydrone/map/features")
        self.declare_parameter("coverage_topic", "/hydrone/map/coverage")

        # Sample every Nth pixel in each axis. 4 turns a 640x480 depth image
        # into 160x120 = 19200 candidates, which is far more than the voxel
        # grid can distinguish at 15 cm and cheap enough to do at process_hz.
        # Lower it only if the map looks holey at long range.
        self.declare_parameter("stride", 4)
        self.declare_parameter("voxel_size", 0.15)
        self.declare_parameter("coverage_res", 0.5)
        # Depth gating: the sim's far plane and the ZED's own noise floor both
        # produce garbage outside a sane band.
        self.declare_parameter("min_depth", 0.4)
        self.declare_parameter("max_depth", 20.0)
        # Flying-pixel rejection. A pixel is dropped when depth varies by more
        # than this across its 3x3 neighbourhood — i.e. it sits on a silhouette
        # and its value is a blend of foreground and background. This is the
        # whole reason the map shows surfaces instead of noise; see the module
        # docstring for the measurement that motivated it.
        self.declare_parameter("max_edge_step", 0.30)
        # Hard caps. An unbounded map on an unbounded plane is how a long flight
        # ends in swap. Higher than the old ORB-era cap because dense sampling
        # legitimately fills far more cells.
        self.declare_parameter("max_voxels", 400000)
        self.declare_parameter("max_coverage_cells", 100000)
        self.declare_parameter("publish_hz", 1.0)
        # Process at most this many frames per second; mapping does not need the
        # full camera rate and the ZED stream is shared with the VO.
        self.declare_parameter("process_hz", 4.0)
        # How far the nearest pose may be from the depth frame's stamp before
        # the frame is dropped as unusable. See _pose_at for why pairing a stale
        # image with a fresh pose is measured in METRES, not milliseconds.
        #
        # Tight on purpose. The odometry carries the image's own stamp, so a
        # match should be EXACT; anything outside this window means the pose for
        # that frame never arrived — the VO declining to publish on a frame it
        # could not track, which on this arena it will (see the ORB measurement
        # above). Dropping such a frame is right. Widening this does not recover
        # the pose, it just back-projects the frame with somebody else's.
        self.declare_parameter("max_pose_dt", 0.15)

        p = lambda n: self.get_parameter(n).value
        self.optical_frame = p("optical_frame")
        self.base_frame = p("base_frame")
        self.stride = max(1, int(p("stride")))
        self.voxel = float(p("voxel_size"))
        self.cov_res = float(p("coverage_res"))
        self.min_depth = float(p("min_depth"))
        self.max_depth = float(p("max_depth"))
        self.max_edge_step = float(p("max_edge_step"))
        self.max_voxels = int(p("max_voxels"))
        self.max_cov = int(p("max_coverage_cells"))
        # Set from the odometry's own header, not hardcoded: the map belongs in
        # whatever frame the pose that built it lives in. That is `odom` — which
        # is also the RIGHT frame for a local map, because odom is continuous,
        # whereas `map` steps whenever the EKF corrects and would tear the cloud
        # at every correction. map_odom_node publishes map->odom, so RViz can
        # still display this in `map`.
        self.world_frame = "odom"

        rate = float(p("process_hz"))
        self.min_period_ns = int(1e9 / rate) if rate > 0.0 else 0

        # ── State ───────────────────────────────────────────────────────────
        self.K: np.ndarray | None = None
        # A short history of poses, not just the latest one, so each depth frame
        # can be matched to where the drone actually was when it was captured.
        self.pose_buf: deque[tuple[int, Pose]] = deque(maxlen=200)
        self.max_pose_dt_ns = int(float(p("max_pose_dt")) * 1e9)
        self._dropped_no_pose = 0
        # The pose chosen for the frame currently being processed, set by
        # _cb_depth before _to_world reads it.
        self.pose: Pose | None = None
        self.R_base_opt: np.ndarray | None = None
        self.t_base_opt: np.ndarray | None = None
        self._last_ns = 0
        self._full_warned = False
        # Pixel coordinate grids, rebuilt only when the image size changes.
        self._grid_shape: tuple[int, int] | None = None
        self._u: np.ndarray | None = None
        self._v: np.ndarray | None = None

        # Voxel hash: (i, j, k) -> [count, sum_x, sum_y, sum_z]. The sums make
        # the published point the centroid of the voxel's contents rather than
        # the voxel's centre — see _cloud for why that distinction shows up as
        # duplicate ground planes. A dict, not a dense array: the map is
        # overwhelmingly empty and its extent is not known in advance.
        self.voxels: dict[tuple[int, int, int], list] = {}
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

        # Poses get a deep queue, unlike the images: the point is to keep a
        # HISTORY to match depth frames against, so dropping intermediate poses
        # under load is exactly the wrong trade. Images want the newest frame,
        # poses want all of them.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.create_subscription(CameraInfo, p("in_info"), self._cb_info,
                                 sensor_qos)
        self._odom_topic = p("odom_topic")
        self.create_subscription(Odometry, self._odom_topic, self._cb_odom,
                                 pose_qos)
        # The depth frame is the trigger: it is the only image this node needs,
        # so there is no RGB/depth pairing to get wrong.
        self.create_subscription(Image, p("in_depth"), self._cb_depth,
                                 sensor_qos)

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._publish)

        self.get_logger().info(
            f"feature_map ready — dense depth, stride {self.stride}, "
            f"{self.voxel * 100:.0f} cm voxels, "
            f"{self.cov_res * 100:.0f} cm coverage cells.")

    # ────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _cb_odom(self, msg: Odometry):
        # Adopt the odometry's own frame for the map: the cloud belongs in the
        # frame of the pose that placed it.
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().info(
                f"map frame is '{msg.header.frame_id}' (from {self._odom_topic})")
            self.world_frame = msg.header.frame_id
        self.pose_buf.append((_stamp_ns(msg.header.stamp), msg.pose.pose))

    def _pose_at(self, stamp_ns: int) -> Pose | None:
        """The pose nearest in time to `stamp_ns`, or None if none is close.

        Using the LATEST pose instead of this is not a rounding error, it is
        metres of map. MEASURED 2026-08-20 with the drone hovering motionless
        (vertical speed 0.001 m/s, altitude steady to 3 mm), the airframe still
        oscillated 7.6 deg peak-to-peak in pitch, and at a 10 m ground range one
        degree of attitude moves a back-projected point 0.175 m vertically. The
        reconstructed floor swung between -1.86 m and +2.26 m — from the SAME
        depth frame, purely because a different pose got paired with it.
        """
        if not self.pose_buf:
            return None
        best_ns, best = min(self.pose_buf, key=lambda kv: abs(kv[0] - stamp_ns))
        if abs(best_ns - stamp_ns) > self.max_pose_dt_ns:
            return None
        return best

    def _cb_depth(self, msg: Image):
        if self.min_period_ns:
            now = self.get_clock().now().nanoseconds
            if now - self._last_ns < self.min_period_ns:
                return
            self._last_ns = now

        if self.K is None:
            return
        self.pose = self._pose_at(_stamp_ns(msg.header.stamp))
        if self.pose is None:
            self._dropped_no_pose += 1
            self.get_logger().warn(
                f"no pose within {self.max_pose_dt_ns / 1e9:.2f} s of the depth "
                f"frame; dropped {self._dropped_no_pose} frame(s). Raise "
                "max_pose_dt, or find out why the pose rate is low.",
                throttle_duration_sec=10.0)
            return
        if not self._ensure_mount_tf():
            return

        world = self._to_world(depth_image_to_numpy(msg))
        if world is None:
            return
        self._accumulate(world)

    # ────────────────────────────────────────────────────────────────────────
    # Geometry
    # ────────────────────────────────────────────────────────────────────────

    def _edge_mask(self, depth: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """True where depth is locally smooth enough to trust.

        A 3x3 min and max filter bracket each pixel's neighbourhood; a spread
        wider than `max_edge_step` means the pixel straddles a silhouette and
        its depth is a foreground/background blend rather than a surface.

        Invalid neighbours are pushed to opposite sentinels, so a pixel next to
        a hole (the sky, mostly) also fails. That deliberately shaves the wall's
        outline off the map — the outline is the part whose depth cannot be
        trusted, and the face behind it survives intact.
        """
        kernel = np.ones((3, 3), np.uint8)
        lo = np.where(valid, depth, np.float32(1e6))
        hi = np.where(valid, depth, np.float32(-1e6))
        local_min = cv2.erode(lo, kernel)
        local_max = cv2.dilate(hi, kernel)
        return (local_max - local_min) <= self.max_edge_step

    def _to_world(self, depth: np.ndarray) -> np.ndarray | None:
        """Depth image -> Mx3 world points, dropping holes and silhouettes."""
        if depth.ndim != 2:
            return None
        h, w = depth.shape
        depth = np.ascontiguousarray(depth, dtype=np.float32)

        valid = np.isfinite(depth) & (depth >= self.min_depth) & \
            (depth <= self.max_depth)
        if not valid.any():
            return None
        keep = valid & self._edge_mask(depth, valid)

        # Subsample AFTER filtering: the edge test needs full-resolution
        # neighbours to see a discontinuity at all.
        s = self.stride
        keep = keep[::s, ::s]
        if not keep.any():
            return None
        d = depth[::s, ::s][keep].astype(np.float64)

        if self._grid_shape != (h, w):
            vv, uu = np.mgrid[0:h:s, 0:w:s]
            self._u = uu.astype(np.float64)
            self._v = vv.astype(np.float64)
            self._grid_shape = (h, w)
        u = self._u[keep]
        v = self._v[keep]

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            return None
        # Optical frame: Z forward along the axis, X right, Y down.
        local = np.stack([(u - cx) / fx * d, (v - cy) / fy * d, d], axis=1)

        q = self.pose.orientation
        R_world_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_base = np.array([self.pose.position.x,
                           self.pose.position.y,
                           self.pose.position.z])
        R_world_opt = R_world_base @ self.R_base_opt
        p_cam = p_base + R_world_base @ self.t_base_opt
        return local @ R_world_opt.T + p_cam

    def _accumulate(self, points: np.ndarray):
        """Fold world points into the voxel hash and the coverage grid.

        Collapses duplicates with np.unique before touching the dicts: a dense
        frame drops ~19k points into a handful of thousands of cells, and paying
        Python's dict cost per point rather than per cell is the difference
        between keeping up at process_hz and not.
        """
        full = len(self.voxels) >= self.max_voxels
        if full and not self._full_warned:
            self.get_logger().warn(
                f"feature map hit max_voxels ({self.max_voxels}); new cells are "
                "being dropped. Raise the cap or the voxel size.")
            self._full_warned = True

        keys, inv = np.unique(
            np.floor(points / self.voxel).astype(np.int64),
            axis=0, return_inverse=True)
        counts = np.bincount(inv)
        sx = np.bincount(inv, weights=points[:, 0])
        sy = np.bincount(inv, weights=points[:, 1])
        sz = np.bincount(inv, weights=points[:, 2])
        for i, key in enumerate(map(tuple, keys)):
            cell = self.voxels.get(key)
            if cell is not None:
                cell[0] += int(counts[i])
                cell[1] += sx[i]
                cell[2] += sy[i]
                cell[3] += sz[i]
            elif not full:
                self.voxels[key] = [int(counts[i]), sx[i], sy[i], sz[i]]

        # Coverage counts FRAMES that saw a cell, not points that landed in it.
        # A dense frame drops ~139 points into a single 50 cm cell, so counting
        # points would peg every observed cell at "looked at hard" within a few
        # seconds and the grid would stop discriminating. One glance is one
        # observation, which is the question the grid actually answers.
        cov_keys = np.unique(
            np.floor(points[:, :2] / self.cov_res).astype(np.int64), axis=0)
        cov_full = len(self.coverage) >= self.max_cov
        for key in map(tuple, cov_keys):
            if key in self.coverage:
                self.coverage[key] += 1
            elif not cov_full:
                self.coverage[key] = 1

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
        """One point per occupied voxel, at the CENTROID of what landed in it.

        Not the voxel centre. Snapping to centres quantises every surface onto
        the grid, and a surface lying near a voxel boundary is then split into
        two sheets a whole voxel apart. MEASURED 2026-08-20: the arena floor
        back-projects to z = +0.0008 with a std of 0.0007 m — flat to five
        millimetres — but sits almost exactly on the boundary at z = 0, so its
        points fell into the voxels below and above it and were published at
        z = -0.075 and z = +0.075. One 5 mm floor, drawn as two floors 150 mm
        apart, purely as an artefact of the output stage.

        The centroid is what a voxel-grid filter is supposed to emit (it is what
        PCL's VoxelGrid does): the grid decides which points get merged, the
        merged points keep their real position.
        """
        cells = self.voxels.values()
        xyz = np.array([(c[1] / c[0], c[2] / c[0], c[3] / c[0]) for c in cells],
                       dtype=np.float64)

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
        # 50 observations of a cell reads as fully covered. Counts are frames
        # (see _accumulate), so at process_hz 4 that is ~12 s of staring.
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
