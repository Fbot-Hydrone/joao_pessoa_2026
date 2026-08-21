#!/usr/bin/env python3
"""
feature_map_node — a world map accumulated from the ZED's point cloud.

The pad map (pad_map_node) records the few things the drone wants to land on.
This node records everything else it has looked at: the ZED's own registered
point cloud, sampled on a stride grid, folded into the world frame and
accumulated in a voxel grid, plus a 2-D grid of how thoroughly each patch of
floor has been observed.

    /zed/zed_node/point_cloud/cloud_registered ─┐
    /zed/zed_node/odom                          ┴─> sample -> fold -> voxel hash
                                              |
                              /hydrone/map/cloud      sensor_msgs/PointCloud2
                              /hydrone/map/coverage   nav_msgs/OccupancyGrid

Why it does not back-project depth itself
-----------------------------------------
It used to: it read `depth/depth_registered` plus `rgb/camera_info` and turned
pixels into 3-D points. That is the CAMERA's job, and the ZED already does it —
`point_cloud/cloud_registered` comes straight out of the SDK on the drone. Doing
it again up here meant this node held a second, slightly different copy of the
camera's geometry, and it meant the simulator was NOT reproducing a product the
real hardware publishes. The back-projection now lives where the camera lives:
zed_mimic_node in sim, zed_wrapper on the drone. This node consumes the cloud in
both, and cannot tell them apart.

What this node still owns is the part no ZED produces: ACCUMULATION into a
persistent world map, and the coverage grid. Those are ours, so they stay in the
autonomy layer and run identically in sim and on the drone.

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
— so the map now samples the ZED's cloud on a stride grid and explicitly REJECTS
the discontinuities that ORB used to seek out. See `_edge_mask`. That rejection
is why the cloud has to arrive ORGANIZED (height x width, one point per pixel):
a flattened cloud has no pixel neighbours left to compare against.

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
and cannot correct for it. TF is consulted only for the constant camera mount —
looked up against whatever frame_id the cloud arrives stamped with, so the
wrapper's choice of frame (the real one publishes the cloud in the left camera's
robot-convention frame, the images in its optical child) is not something this
node has to be told.

The pose comes from the ZED's odometry rather than the FCU because the ZED
produces exactly one pose per camera frame, carrying that frame's own
timestamp — so each cloud is folded in with the attitude it was actually
captured at. See the `odom_topic` parameter for the measurements behind
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
from sensor_msgs.msg import PointCloud2, PointField

import tf2_ros


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
        # The ZED's own cloud — the SDK's product on the drone, zed_mimic_node's
        # in sim. Same string in both, which is the whole point.
        self.declare_parameter("in_cloud",
                               "/zed/zed_node/point_cloud/cloud_registered")
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
        self.declare_parameter("base_frame", "base_link")
        # NOT /zed/... : this is the accumulated map, which no ZED publishes.
        # The per-frame cloud keeps the camera's name; this one is ours.
        self.declare_parameter("cloud_topic", "/hydrone/map/cloud")
        self.declare_parameter("coverage_topic", "/hydrone/map/coverage")

        # Sample every Nth pixel in each axis. 4 turns a 640x480 cloud
        # into 160x120 = 19200 candidates, which is far more than the voxel
        # grid can distinguish at 15 cm and cheap enough to do at process_hz.
        # Lower it only if the map looks holey at long range.
        self.declare_parameter("stride", 4)
        self.declare_parameter("voxel_size", 0.15)
        self.declare_parameter("coverage_res", 0.5)
        # Depth gating: the sim's far plane and the ZED's own noise floor both
        # produce garbage outside a sane band. Applied to the point's RANGE
        # (its distance from the camera), not to one axis of it, so it means the
        # same thing whichever convention the cloud's frame uses.
        self.declare_parameter("min_depth", 0.4)
        self.declare_parameter("max_depth", 20.0)
        # Flying-pixel rejection. A pixel is dropped when range varies by more
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
        # How far the nearest pose may be from the cloud's stamp before
        # the frame is dropped as unusable. See _pose_at for why pairing a stale
        # image with a fresh pose is measured in METRES, not milliseconds.
        #
        # Tight on purpose. The odometry carries the image's own stamp, so a
        # match should be EXACT; anything outside this window means the pose for
        # that frame never arrived — the VO declining to publish on a frame it
        # could not track, which on this arena it will (see the ORB measurement
        # above). Dropping such a frame is right. Widening this does not recover
        # the pose, it just folds the frame in with somebody else's.
        self.declare_parameter("max_pose_dt", 0.15)

        p = lambda n: self.get_parameter(n).value
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
        # A short history of poses, not just the latest one, so each depth frame
        # can be matched to where the drone actually was when it was captured.
        self.pose_buf: deque[tuple[int, Pose]] = deque(maxlen=200)
        self.max_pose_dt_ns = int(float(p("max_pose_dt")) * 1e9)
        self._dropped_no_pose = 0
        # The pose chosen for the frame currently being processed, set by
        # _cb_cloud before _to_world reads it.
        self.pose: Pose | None = None
        # base_link <- camera mount, cached per cloud frame_id. Keyed rather
        # than singular because the frame the cloud arrives in is the wrapper's
        # choice, not ours, and it is not worth asserting which one it will be.
        self.mounts: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._last_ns = 0
        self._full_warned = False
        self._unorganized_warned = False

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

        # Poses get a deep queue, unlike the cloud: the point is to keep a
        # HISTORY to match camera frames against, so dropping intermediate poses
        # under load is exactly the wrong trade. A cloud wants the newest frame,
        # poses want all of them.
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self._odom_topic = p("odom_topic")
        self.create_subscription(Odometry, self._odom_topic, self._cb_odom,
                                 pose_qos)
        # The cloud is the trigger, and the only sensor input: it carries the
        # geometry AND the camera's intrinsics already applied, so there is no
        # image/camera_info pairing left to get wrong.
        self._cloud_topic = p("in_cloud")
        self.create_subscription(PointCloud2, self._cloud_topic, self._cb_cloud,
                                 sensor_qos)

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._publish)

        self.get_logger().info(
            f"feature_map ready — accumulating {self._cloud_topic}, stride "
            f"{self.stride}, {self.voxel * 100:.0f} cm voxels, "
            f"{self.cov_res * 100:.0f} cm coverage cells.")

    # ────────────────────────────────────────────────────────────────────────

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

    def _cb_cloud(self, msg: PointCloud2):
        if self.min_period_ns:
            now = self.get_clock().now().nanoseconds
            if now - self._last_ns < self.min_period_ns:
                return
            self._last_ns = now

        self.pose = self._pose_at(_stamp_ns(msg.header.stamp))
        if self.pose is None:
            self._dropped_no_pose += 1
            self.get_logger().warn(
                f"no pose within {self.max_pose_dt_ns / 1e9:.2f} s of the cloud; "
                f"dropped {self._dropped_no_pose} frame(s). Raise max_pose_dt, "
                "or find out why the pose rate is low.",
                throttle_duration_sec=10.0)
            return
        mount = self._mount_tf(msg.header.frame_id)
        if mount is None:
            return

        points = self._sample(msg)
        if points is None:
            return
        world = self._to_world(points, mount)
        if world is None:
            return
        self._accumulate(world)

    # ────────────────────────────────────────────────────────────────────────
    # Geometry
    # ────────────────────────────────────────────────────────────────────────

    def _xyz(self, msg: PointCloud2) -> np.ndarray | None:
        """The cloud's x/y/z as an (H, W, 3) float32 view, or None.

        Deliberately narrow: FLOAT32 x/y/z at 4-byte-aligned offsets in a
        4-byte-aligned point. That is what the ZED publishes (x, y, z, rgb —
        four floats, point_step 16) and what zed_mimic_node publishes, and a
        cloud shaped otherwise is a cloud this node has not been told about.
        Guessing at it would put silently wrong geometry into the map, so it
        says so and drops the frame instead.
        """
        fields = {f.name: f for f in msg.fields}
        try:
            xyz = [fields[n] for n in ("x", "y", "z")]
        except KeyError:
            self.get_logger().error(
                f"{self._cloud_topic} has no x/y/z fields; not mapping it",
                throttle_duration_sec=30.0)
            return None
        if any(f.datatype != PointField.FLOAT32 or f.count != 1 or f.offset % 4
               for f in xyz) or msg.point_step % 4:
            self.get_logger().error(
                f"{self._cloud_topic} is not float32 x/y/z on a 4-byte grid "
                f"(point_step {msg.point_step}); not mapping it",
                throttle_duration_sec=30.0)
            return None

        floats = np.frombuffer(msg.data, dtype=np.float32)
        stride = msg.point_step // 4
        n = msg.width * msg.height
        if floats.size < n * stride:
            return None
        cols = [f.offset // 4 for f in xyz]
        pts = floats[:n * stride].reshape(n, stride)[:, cols]
        return pts.reshape(msg.height, msg.width, 3)

    def _edge_mask(self, rng: np.ndarray, valid: np.ndarray) -> np.ndarray:
        """True where range is locally smooth enough to trust.

        A 3x3 min and max filter bracket each pixel's neighbourhood; a spread
        wider than `max_edge_step` means the pixel straddles a silhouette and
        its value is a foreground/background blend rather than a surface.

        Invalid neighbours are pushed to opposite sentinels, so a pixel next to
        a hole (the sky, mostly) also fails. That deliberately shaves the wall's
        outline off the map — the outline is the part whose depth cannot be
        trusted, and the face behind it survives intact.
        """
        kernel = np.ones((3, 3), np.uint8)
        lo = np.where(valid, rng, np.float32(1e6))
        hi = np.where(valid, rng, np.float32(-1e6))
        local_min = cv2.erode(lo, kernel)
        local_max = cv2.dilate(hi, kernel)
        return (local_max - local_min) <= self.max_edge_step

    def _sample(self, msg: PointCloud2) -> np.ndarray | None:
        """Cloud -> Mx3 camera-frame points, dropping holes and silhouettes."""
        pts = self._xyz(msg)
        if pts is None or pts.size == 0:
            return None
        h, w, _ = pts.shape

        # Range, not one axis: the cloud's frame convention is the wrapper's
        # business, and the distance from the camera is the same number in any
        # of them. NaN (no return) fails every comparison, which is the answer.
        rng = np.sqrt((pts.astype(np.float32) ** 2).sum(axis=2))
        with np.errstate(invalid="ignore"):
            valid = np.isfinite(rng) & (rng >= self.min_depth) & \
                (rng <= self.max_depth)
        if not valid.any():
            return None

        if h > 1:
            keep = valid & self._edge_mask(rng, valid)
        else:
            # An unorganized cloud has no neighbours to compare against, so the
            # flying pixels stay in. Usable, but visibly noisier — the real
            # wrapper publishes an organized cloud, so this is a fallback for a
            # cloud that has been through a filter that flattened it.
            keep = valid
            if not self._unorganized_warned:
                self.get_logger().warn(
                    f"{self._cloud_topic} is unorganized (height 1); "
                    "flying-pixel rejection needs pixel neighbours and is off.")
                self._unorganized_warned = True

        # Subsample AFTER filtering: the edge test needs full-resolution
        # neighbours to see a discontinuity at all. On an organized cloud this
        # takes one pixel in stride^2; on a flat one there is a single axis to
        # stride along, so it thins by stride. Both are just density.
        s = self.stride
        keep = keep[::s, ::s]
        if not keep.any():
            return None
        return pts[::s, ::s][keep].astype(np.float64)

    def _to_world(self, points: np.ndarray,
                  mount: tuple[np.ndarray, np.ndarray]) -> np.ndarray | None:
        """Camera-frame points -> world frame, through the mount and the pose."""
        if points.size == 0:
            return None
        R_base_cam, t_base_cam = mount

        q = self.pose.orientation
        R_world_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_base = np.array([self.pose.position.x,
                           self.pose.position.y,
                           self.pose.position.z])
        R_world_cam = R_world_base @ R_base_cam
        p_cam = p_base + R_world_base @ t_base_cam
        return points @ R_world_cam.T + p_cam

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

    def _mount_tf(self, frame: str):
        """base_link <- the cloud's own frame, looked up once and cached.

        The frame comes from the message, not from a parameter: the ZED wrapper
        stamps its cloud with the left camera's robot-convention frame and its
        images with that frame's optical child, and which of the two arrives is
        the wrapper's business. Reading the header means this node is right
        either way, in sim and on the drone, with nothing to configure.

        Constant in flight (it is a mount), so one successful lookup is kept.
        """
        if not frame:
            self.get_logger().error(
                f"{self._cloud_topic} arrived with an empty frame_id; there is "
                "no way to place it in the world", throttle_duration_sec=30.0)
            return None
        mount = self.mounts.get(frame)
        if mount is not None:
            return mount
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().warn(
                f"waiting for TF {self.base_frame} -> {frame}",
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        mount = (quat_to_matrix(q.x, q.y, q.z, q.w),
                 np.array([t.x, t.y, t.z]))
        self.mounts[frame] = mount
        return mount

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
