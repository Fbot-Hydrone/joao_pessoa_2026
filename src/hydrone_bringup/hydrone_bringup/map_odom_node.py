#!/usr/bin/env python3
"""
map_odom_node — publish the real map -> odom transform instead of guessing it.

TF comes up as two disconnected trees, because two different things localise the
vehicle and neither knows about the other:

    odom -> base_link -> {zed_camera_link, down_cam_link, ...}   (VO / zed_mimic)
    map  -> map_ned                                              (MAVROS static)

Everything this stack MAPS (`/hydrone/map/features`, `/hydrone/pads/map`) is
built from /mavros/local_position/pose and therefore lives in `map`. Everything
it SENSES hangs off `base_link`. Without an edge between the trees, RViz can
render the map or the vehicle but never both in the same place.

What this replaces
------------------
A `static_transform_publisher` broadcasting map -> odom as IDENTITY. That is
wrong whenever `odom` and `map` are not the same world frame, and in this stack
they never are:

  * BiguaSim publishes odometry in NWU. zed_mimic republishes it unchanged and
    broadcasts odom -> base_link in NWU. vision_odom_bridge then rotates it
    +90 deg about Z to ENU before MAVROS ever sees it, so `map` is ENU.
  * visual_odometry_node fixes its odom origin to IDENTITY at the first frame,
    so on the VO path `odom` is aligned to whatever attitude the drone booted
    at — not to any world convention at all. The real ZED wrapper behaves the
    same way, so this is not a sim artefact.

MEASURED with the identity transform in place (2026-08-20), at two different
headings, `/mavros/local_position/pose` was Rz(+90 deg) x ground truth to within
0.012 m and 0.55 deg. RViz drew the vehicle 90 deg away from its own point
cloud, with the origin rotated to match.

How it works
------------
The transform is not a constant to be looked up, it is a quantity to be
computed. Both halves are already published:

    map_T_odom = map_T_base . (odom_T_base)^-1

    map_T_base   <- /mavros/local_position/pose  (the FCU's estimate, in `map`)
    odom_T_base  <- TF, from whoever owns odom->base_link (VO or zed_mimic)

This is exactly how AMCL and robot_localization publish map -> odom, and it is
correct for any boot heading, in sim and on the real drone, with no per-setup
constant to maintain.

The residual is not noise to be filtered out — it IS the localisation error.
`odom` is the VO's origin and `map` is the FCU's EKF origin; they coincide at
boot and diverge afterwards by exactly the accumulated drift. Publishing the
measured offset is what makes that drift visible in RViz rather than silently
smearing the map.

This node is a pure observer of two existing signals. It publishes one TF and
nothing else, and no flight-critical code consults map <-> odom (the only two
lookup_transform calls in the stack both read the constant camera mount).
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped

import tf2_ros


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> (x, y, z, w).

    Shepperd's method: pick the largest of the four denominators so the square
    root never divides by something near zero. The naive trace-only formula
    loses all precision at 180 deg, which is a rotation this transform can
    legitimately reach.
    """
    m00, m11, m22 = R[0, 0], R[1, 1], R[2, 2]
    t = m00 + m11 + m22
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


class MapOdomNode(Node):

    def __init__(self, **kwargs):
        super().__init__("map_odom", **kwargs)

        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # SIM-ONLY: also place BiguaSim's ground-truth frame in the tree, so a
        # truth pose can be rendered next to the estimate without borrowing the
        # estimator's frame. Harmless on the real drone — there is no such topic
        # there, so nothing is ever published.
        self.declare_parameter("gt_pose_topic", "/zed/zed_node/pose_GT")
        self.declare_parameter("gt_frame", "odom_gt")
        self.declare_parameter("publish_gt_tf", True)
        self.declare_parameter("publish_hz", 10.0)
        # Report the map->odom offset periodically. This is the localisation
        # error in a single number, so it is worth seeing without RViz.
        self.declare_parameter("log_period_s", 10.0)

        p = lambda n: self.get_parameter(n).value
        self.map_frame = p("map_frame")
        self.odom_frame = p("odom_frame")
        self.base_frame = p("base_frame")

        self.pose: PoseStamped | None = None
        self._warned = False
        self._stale_lookups = 0
        self._last_log_ns = 0
        self._log_period_ns = int(float(p("log_period_s")) * 1e9)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # MAVROS publishes local_position BEST_EFFORT (SensorDataQoS); a
        # RELIABLE subscription silently matches nothing.
        self.create_subscription(
            PoseStamped, p("pose_topic"), self._cb_pose,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.gt_frame = p("gt_frame")
        self.gt_pose: PoseStamped | None = None
        # The latched map -> gt_frame transform, as (translation, rotation).
        self._gt_tf: tuple[np.ndarray, np.ndarray] | None = None
        if p("publish_gt_tf"):
            self.create_subscription(
                PoseStamped, p("gt_pose_topic"), self._cb_gt,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=1))

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._publish)

        self.get_logger().info(
            f"map_odom ready — computing {self.map_frame} -> {self.odom_frame} "
            f"as map_T_base . (odom_T_base)^-1")

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_gt(self, msg: PoseStamped):
        self.gt_pose = msg

    def _publish_gt_tf(self, stamp):
        """Place the ground-truth frame, latching the alignment at boot.

        `map` (the FCU's ENU origin) and the simulator's truth frame are BOTH
        world-fixed, so the transform between them is a CONSTANT — the boot
        alignment — and is computed once rather than hardcoded:

            map_T_gt = map_T_base . (gt_T_base)^-1     evaluated once

        Latching is the whole point. Recomputing it every cycle would drag the
        truth pose onto the estimate no matter how far the estimate had wandered,
        which would hide exactly the error this frame exists to expose. Held
        fixed, the gap you see between truth and estimate in RViz is real.
        """
        if self._gt_tf is None:
            if self.gt_pose is None or self.pose is None:
                return
            # Only latch while the vehicle is LEVEL. The alignment is a constant
            # to be sampled once, so sampling it during a tumble bakes a garbage
            # rotation in for the rest of the run — and this node can be
            # (re)started at any moment, including mid-flight. Observed
            # 2026-08-20: a restart during a crash latched a +2.459 m z offset
            # off a vehicle that was inverted at roll +161 deg and falling.
            # On the ground at boot this passes on the first sample.
            g0 = self.gt_pose.pose.orientation
            R0 = quat_to_matrix(g0.x, g0.y, g0.z, g0.w)
            if R0[2, 2] < math.cos(math.radians(15.0)):
                self.get_logger().warn(
                    "not latching map -> "
                    f"{self.gt_frame}: vehicle is tilted "
                    f"{math.degrees(math.acos(max(-1.0, min(1.0, R0[2, 2])))):.0f}"
                    " deg from level. Waiting for it to be upright.",
                    throttle_duration_sec=10.0)
                return
            q = self.pose.pose.orientation
            R_mb = quat_to_matrix(q.x, q.y, q.z, q.w)
            t_mb = np.array([self.pose.pose.position.x,
                             self.pose.pose.position.y,
                             self.pose.pose.position.z])
            g = self.gt_pose.pose.orientation
            R_gb = quat_to_matrix(g.x, g.y, g.z, g.w)
            t_gb = np.array([self.gt_pose.pose.position.x,
                             self.gt_pose.pose.position.y,
                             self.gt_pose.pose.position.z])
            R = R_mb @ R_gb.T
            self._gt_tf = (t_mb - R @ t_gb, R)
            self.get_logger().info(
                f"latched {self.map_frame} -> {self.gt_frame}: yaw "
                f"{math.degrees(math.atan2(R[1, 0], R[0, 0])):+.1f} deg, "
                f"z {self._gt_tf[0][2]:+.3f} m — the boot alignment between the "
                "FCU origin and the simulator's world frame")

        t_mg, R_mg = self._gt_tf
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.gt_frame
        msg.transform.translation.x = float(t_mg[0])
        msg.transform.translation.y = float(t_mg[1])
        msg.transform.translation.z = float(t_mg[2])
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = \
            matrix_to_quat(R_mg)
        self.tf_broadcaster.sendTransform(msg)

    def _publish(self):
        if self.pose is None:
            self.get_logger().warn("waiting for the FCU pose",
                                   throttle_duration_sec=5.0)
            return

        self._publish_gt_tf(self.get_clock().now().to_msg())

        # Look the odometry up AT THE POSE'S OWN STAMP, not at "latest".
        #
        # This matters more than it looks. The correction is only valid for the
        # instant both halves describe, and MEASURED 2026-08-20 the FCU pose
        # arrives at ~1 Hz while the odometry runs far faster. Pairing a pose
        # from up to a second ago with the newest odometry makes
        #
        #     map_T_odom . odom_T_base(now)  ==  map_T_base(t_pose)
        #
        # i.e. the vehicle FREEZES between pose updates and then snaps forward —
        # and any jump in the VO shows up in RViz for a full second before the
        # correction catches up. With a time-consistent pair the composition
        # becomes map_T_base(t_pose) propagated by the odometry's own motion
        # since t_pose, which is smooth and is what the odom frame is for.
        stamp = rclpy.time.Time.from_msg(self.pose.header.stamp)
        try:
            tf_ob = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, stamp)
        except tf2_ros.TransformException:
            # The pose may be older than the buffer, or newer than the last
            # odometry. Falling back to latest is still better than publishing
            # nothing, but it reintroduces the freeze above, so say so.
            try:
                tf_ob = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, rclpy.time.Time())
            except tf2_ros.TransformException:
                self.get_logger().warn(
                    f"waiting for TF {self.odom_frame} -> {self.base_frame} "
                    "(is the odometry source running?)",
                    throttle_duration_sec=5.0)
                return
            self._stale_lookups += 1
            self.get_logger().warn(
                "no odometry at the pose's stamp; fell back to latest "
                f"({self._stale_lookups}x). The vehicle will look like it "
                "steps at the pose rate until the two line up.",
                throttle_duration_sec=10.0)

        q = self.pose.pose.orientation
        R_mb = quat_to_matrix(q.x, q.y, q.z, q.w)
        t_mb = np.array([self.pose.pose.position.x,
                         self.pose.pose.position.y,
                         self.pose.pose.position.z])

        r = tf_ob.transform.rotation
        R_ob = quat_to_matrix(r.x, r.y, r.z, r.w)
        t_ob = np.array([tf_ob.transform.translation.x,
                         tf_ob.transform.translation.y,
                         tf_ob.transform.translation.z])

        # map_T_odom = map_T_base . (odom_T_base)^-1
        R_mo = R_mb @ R_ob.T
        t_mo = t_mb - R_mo @ t_ob

        msg = TransformStamped()
        # Stamped now, not at the pose's stamp: this transform is consumed by
        # RViz at wall-clock "latest", and back-dating it to a 1 Hz pose leaves
        # TF extrapolating into a gap it will refuse to fill.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = float(t_mo[0])
        msg.transform.translation.y = float(t_mo[1])
        msg.transform.translation.z = float(t_mo[2])
        (msg.transform.rotation.x, msg.transform.rotation.y,
         msg.transform.rotation.z, msg.transform.rotation.w) = \
            matrix_to_quat(R_mo)
        self.tf_broadcaster.sendTransform(msg)

        now = self.get_clock().now().nanoseconds
        if self._log_period_ns and now - self._last_log_ns >= self._log_period_ns:
            self._last_log_ns = now
            yaw = math.degrees(math.atan2(R_mo[1, 0], R_mo[0, 0]))
            self.get_logger().info(
                f"{self.map_frame}->{self.odom_frame}: "
                f"xyz=({t_mo[0]:+.2f}, {t_mo[1]:+.2f}, {t_mo[2]:+.2f}) "
                f"yaw={yaw:+.1f} deg  |  this offset IS the drift between the "
                f"FCU estimate and the odometry origin")


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomNode()
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
