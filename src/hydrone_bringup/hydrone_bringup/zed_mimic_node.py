#!/usr/bin/env python3
"""
zed_mimic_node — republishes BiguaSim camera/odom/IMU topics under the exact
topic names and frame_ids of the real ZED ROS 2 wrapper.

The goal: downstream nodes (hydrone_vision, nav, ...) subscribe only to
/zed/zed_node/* topics. In simulation this node provides them; on the real
drone the zed_wrapper provides them. Switching = swap which node you launch.
Nothing downstream changes.

Inputs (from biguasim_main ardubridge_node, namespace /biguasim, agent name
from config.yaml with biguasim's '_id0' batch suffix — currently agent uav0):
  /biguasim/uav0_id0/RGBCamera               sensor_msgs/Image   (bgr8, no stamp!)
  /biguasim/uav0_id0/RGBCamera/camera_info   sensor_msgs/CameraInfo
  /biguasim/uav0_id0/DepthCamera             sensor_msgs/Image   (32FC1, no stamp!)
  /biguasim/uav0_id0/DynamicsSensor/Odom     nav_msgs/Odometry   (ground truth)
  /biguasim/uav0_id0/DynamicsSensor/IMU      sensor_msgs/Imu
sources_sim.launch.py is the source of truth: it reads biguasim's config.yaml
and overrides every input topic (and the camera offset) from it. The defaults
below only matter when the node is run standalone with `ros2 run`.

Outputs (real ZED wrapper names):
  /zed/zed_node/rgb/image_rect_color     sensor_msgs/Image
  /zed/zed_node/rgb/camera_info          sensor_msgs/CameraInfo
  /zed/zed_node/depth/depth_registered   sensor_msgs/Image (32FC1, meters, NaN=invalid)
  /zed/zed_node/depth/camera_info        sensor_msgs/CameraInfo
  /zed/zed_node/odom                     nav_msgs/Odometry (odom -> base_link)
  /zed/zed_node/imu/data                 sensor_msgs/Imu
  /zed/zed_node/pose_GT                  geometry_msgs/PoseStamped  (SIM-ONLY:
                                         ground-truth position AND quaternion,
                                         the same data as the odometry but flat
                                         enough to echo/plot/RViz directly)

TF:
  odom -> base_link                      dynamic, from the odometry
  base_link -> zed_camera_link           static (camera mounting position)
  zed_camera_link -> zed_left_camera_optical_frame   static (optical convention)
  zed_camera_link -> zed_imu_link        static

Timestamp rule: BiguaSim messages arrive with stamp = 0, and RGB/depth/info
come in as separate messages of the same sim frame. We cache the latest depth
and camera_info, and when an RGB frame arrives we stamp the whole bundle with
one single clock read, so RGB, depth and both CameraInfos always carry an
IDENTICAL header.stamp — required by time-synchronized subscribers downstream.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


# Frame names — must match the real zed_wrapper so downstream code and TF
# lookups work identically in sim and on the drone.
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_CAMERA = "zed_camera_link"
FRAME_OPTICAL = "zed_left_camera_optical_frame"
# Ground truth is NOT the same frame as `odom`, even though both describe the
# vehicle. `odom` belongs to whoever owns the odom->base_link TF (the VO, whose
# origin is wherever the drone booted and which drifts from there); ground truth
# is BiguaSim's fixed world frame. Publishing truth as frame_id "odom" makes
# RViz render it THROUGH the VO's drift: MEASURED 2026-08-20, a perfectly
# stationary ground-truth pose (0.000 m of real movement) was drawn 7.5 m from
# the vehicle and creeping, which reads as "ground truth is jumping" when the
# only thing moving is the frame it was borrowed from.
FRAME_ODOM_GT = "odom_gt"
FRAME_IMU = "zed_imu_link"

# Rotation from a robot-convention frame (X fwd, Y left, Z up) to the optical
# convention (Z fwd out of the lens, X right, Y down). This is the standard
# ROS camera_link -> camera_optical_frame quaternion: RPY(-90deg, 0, -90deg).
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)  # (x, y, z, w)


class ZedMimicNode(Node):

    def __init__(self):
        super().__init__("zed_mimic")

        # ── Parameters ──────────────────────────────────────────────────────
        # Input topics: where BiguaSim's bridge publishes. Parameters because
        # sources_sim.launch.py derives them from biguasim's config.yaml —
        # renaming the agent there is the ONLY edit needed. These defaults are
        # standalone-run fallbacks and mirror config.yaml's current agent (uav0).
        self.declare_parameter("in_rgb", "/biguasim/uav0_id0/RGBCamera")
        self.declare_parameter("in_rgb_info", "/biguasim/uav0_id0/RGBCamera/camera_info")
        self.declare_parameter("in_depth", "/biguasim/uav0_id0/DepthCamera")
        self.declare_parameter("in_odom", "/biguasim/uav0_id0/DynamicsSensor/Odom")
        self.declare_parameter("in_imu", "/biguasim/uav0_id0/DynamicsSensor/IMU")
        # Where to publish odom. When the real VO node owns /zed/zed_node/odom,
        # ground truth is pointed at /zed/zed_node/odom_GT for comparison/debug.
        self.declare_parameter("out_odom", "/zed/zed_node/odom")
        # Ground-truth pose as a plain PoseStamped: BiguaSim knows the drone's
        # exact position AND orientation, but on the Odometry topic the
        # quaternion is buried inside pose.pose.orientation, which is awkward to
        # echo, to plot, and to point an RViz Axes display at. This publishes the
        # same truth in the form you actually want when you are asking "is the
        # estimate's attitude right, or has the airframe genuinely flipped?" —
        # the question that separates a broken estimator from a real crash.
        self.declare_parameter("out_pose", "/zed/zed_node/pose_GT")
        # Whether to broadcast the dynamic odom->base_link TF. Default False: the
        # real VO (visual_odometry_node) is the single owner of that transform.
        # Set True only if zed_mimic is the odom source and nothing else owns TF
        # (e.g. running the mimic standalone). Static TFs below are always sent.
        self.declare_parameter("publish_tf", False)
        # Multiply sim depth values to get meters (set to 0.01 if the sim
        # turns out to report centimeters — verify in RViz once).
        self.declare_parameter("depth_scale", 1.0)
        # Camera mounting position on the drone body, in meters, in base_link
        # (X forward, Y left, Z up). In sim this is overridden from the camera's
        # `location` in biguasim's config.yaml — BiguaSim's body frame is GLU,
        # the same axes as base_link, so the value carries over unchanged. On
        # the real drone, set it to the measured ZED mount. Default mirrors the
        # current config.yaml so a standalone run is not silently off.
        self.declare_parameter("camera_offset_xyz", [0.14, 0.0, -0.08])

        self.depth_scale = self.get_parameter("depth_scale").value

        # ── State: latest depth/info cached until the next RGB frame ───────
        self.last_depth: Image | None = None
        self.last_info: CameraInfo | None = None
        # Whether last_depth has already been through _prepare_depth.
        self._depth_converted = False

        # ── Publishers (real ZED topic names) ───────────────────────────────
        self.pub_rgb = self.create_publisher(
            Image, "/zed/zed_node/rgb/image_rect_color", 10)
        self.pub_rgb_info = self.create_publisher(
            CameraInfo, "/zed/zed_node/rgb/camera_info", 10)
        self.pub_depth = self.create_publisher(
            Image, "/zed/zed_node/depth/depth_registered", 10)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, "/zed/zed_node/depth/camera_info", 10)
        self.pub_odom = self.create_publisher(
            Odometry, self.get_parameter("out_odom").value, 10)
        self.pub_imu = self.create_publisher(
            Imu, "/zed/zed_node/imu/data", 10)
        self.pub_pose_gt = self.create_publisher(
            PoseStamped, self.get_parameter("out_pose").value, 10)

        # ── TF broadcasters ─────────────────────────────────────────────────
        self.publish_tf = self.get_parameter("publish_tf").value
        # Own the odom frame only when we also own its TF. Otherwise this node
        # is the ground-truth REFERENCE alongside a real estimator, and must say
        # so in the frame_id — see FRAME_ODOM_GT.
        self.odom_frame = FRAME_ODOM if self.publish_tf else FRAME_ODOM_GT
        self.tf_dynamic = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        self._send_static_tfs()

        # ── Subscribers (BiguaSim bridge topics) ────────────────────────────
        p = lambda name: self.get_parameter(name).value
        # Image queues are depth 1 ON PURPOSE. BiguaSim pushes frames faster
        # than this node republishes them, and a deeper queue does not buy extra
        # frames — it buys a BACKLOG, handing downstream a picture of where the
        # drone used to be. Mapping back-projects that stale image against a
        # fresh pose, so every queued frame is directly metres of error. Depth 1
        # keeps only the newest frame and drops the rest, which is what a camera
        # driver should do.
        self.create_subscription(Image, p("in_rgb"), self._cb_rgb, 1)
        self.create_subscription(CameraInfo, p("in_rgb_info"), self._cb_info, 10)
        self.create_subscription(Image, p("in_depth"), self._cb_depth, 1)
        self.create_subscription(Odometry, p("in_odom"), self._cb_odom, 10)
        self.create_subscription(Imu, p("in_imu"), self._cb_imu, 10)

        self.get_logger().info("ZED mimic ready — publishing /zed/zed_node/*")

    # ────────────────────────────────────────────────────────────────────────
    # Static TF tree (sent once; latched for late subscribers)
    # ────────────────────────────────────────────────────────────────────────

    def _send_static_tfs(self):
        ox, oy, oz = self.get_parameter("camera_offset_xyz").value

        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = FRAME_BASE
        mount.child_frame_id = FRAME_CAMERA
        mount.transform.translation.x = float(ox)
        mount.transform.translation.y = float(oy)
        mount.transform.translation.z = float(oz)
        mount.transform.rotation.w = 1.0  # camera looks straight forward

        optical = TransformStamped()
        optical.header.stamp = mount.header.stamp
        optical.header.frame_id = FRAME_CAMERA
        optical.child_frame_id = FRAME_OPTICAL
        (optical.transform.rotation.x,
         optical.transform.rotation.y,
         optical.transform.rotation.z,
         optical.transform.rotation.w) = OPTICAL_QUAT

        imu = TransformStamped()
        imu.header.stamp = mount.header.stamp
        imu.header.frame_id = FRAME_CAMERA
        imu.child_frame_id = FRAME_IMU
        imu.transform.rotation.w = 1.0  # IMU sits inside the camera housing

        self.tf_static.sendTransform([mount, optical, imu])

    # ────────────────────────────────────────────────────────────────────────
    # Camera bundle: cache depth/info, publish everything on each RGB frame
    # ────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.last_info = msg

    def _cb_depth(self, msg: Image):
        # Cache the frame RAW and do no work here. BiguaSim publishes depth at
        # ~34 Hz but this node only republishes on an RGB frame, so the vast
        # majority of what arrives is overwritten before anyone sees it.
        #
        # This callback used to do the float32 NaN pass inline: a 1.2 MB
        # np.frombuffer().copy(), a full-array compare, and a 1.2 MB tobytes()
        # back into msg.data — ~2.4 MB of copying plus an rclpy conversion, 34
        # times a second, to throw away 15 frames out of every 16. MEASURED
        # 2026-08-20 that pinned this node at 101% CPU and throttled
        # /zed/zed_node/depth/depth_registered from 34.4 Hz down to 1.0 Hz,
        # which is where the mapper's ~1 s of camera latency came from.
        self.last_depth = msg
        self._depth_converted = False

    def _prepare_depth(self, msg: Image):
        """Scale to metres and turn the sim's far-plane sentinel into NaN.

        Sky / no-geometry pixels saturate the GPU's float16 depth buffer at
        655.04 m (65504 cm). The real ZED reports NaN there (REP 118); leaving
        the sentinel poisons any mean-depth / obstacle / ground-height math and
        crushes the RViz colour scale. Mimic the real sensor.
        """
        depth = np.frombuffer(msg.data, dtype=np.float32).copy()
        if self.depth_scale != 1.0:
            depth *= self.depth_scale
        depth[depth >= 655.0] = np.nan
        msg.data = depth.tobytes()

    def _cb_rgb(self, msg: Image):
        # One clock read for the whole frame — this is what keeps RGB, depth
        # and CameraInfo stamps identical.
        stamp = self.get_clock().now().to_msg()

        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_OPTICAL
        self.pub_rgb.publish(msg)

        if self.last_depth is not None:
            # Convert once per frame that is actually published, and only once
            # even if RGB outruns depth and we republish the same cached frame.
            if not self._depth_converted:
                self._prepare_depth(self.last_depth)
                self._depth_converted = True
            self.last_depth.header.stamp = stamp
            self.last_depth.header.frame_id = FRAME_OPTICAL
            self.pub_depth.publish(self.last_depth)

        if self.last_info is not None:
            self.last_info.header.stamp = stamp
            self.last_info.header.frame_id = FRAME_OPTICAL
            self.pub_rgb_info.publish(self.last_info)
            # Depth is registered onto the RGB image -> same intrinsics.
            self.pub_depth_info.publish(self.last_info)

    # ────────────────────────────────────────────────────────────────────────
    # Odometry: fix frames, republish, and broadcast odom -> base_link TF
    # ────────────────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry):
        # The bridge encoder leaves frame_id/child_frame_id swapped and
        # meaningless ('base_link'/'odom'); rewrite them properly.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = FRAME_BASE
        self.pub_odom.publish(msg)

        # Same truth, flat and inspectable:
        #   ros2 topic echo /zed/zed_node/pose_GT --field pose.orientation
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pub_pose_gt.publish(pose)

        # Broadcast odom->base_link only when this node is the designated TF owner
        # (publish_tf=True). By default the real VO (visual_odometry_node) owns it,
        # so we stay silent here to avoid a second broadcaster of the same TF.
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = FRAME_BASE
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_dynamic.sendTransform(t)

    # ────────────────────────────────────────────────────────────────────────
    # IMU: restamp and rename frame
    # ────────────────────────────────────────────────────────────────────────

    def _cb_imu(self, msg: Imu):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = FRAME_IMU
        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedMimicNode()
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
