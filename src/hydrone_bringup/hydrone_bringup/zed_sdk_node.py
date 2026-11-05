#!/usr/bin/env python3
"""
zed_sdk_node — the real ZED, publishing the same bus zed_mimic_node fakes.

REAL HARDWARE ONLY. This is the drone-side counterpart of zed_mimic_node: it
opens the camera through the Stereolabs SDK and publishes, byte for byte, the
contract the autonomy layer already consumes.

    ZED (USB)  --SDK-->  /zed/zed_node/rgb/image_rect_color
                         /zed/zed_node/rgb/camera_info
                         /zed/zed_node/depth/depth_registered
                         /zed/zed_node/depth/camera_info
                         /zed/zed_node/point_cloud/cloud_registered
                         /zed/zed_node/odom          (the SDK's own tracking)
                         TF: odom -> base_link, and the static camera chain

Nothing above the sources can tell this apart from the simulator. Same topics,
same frames, same encodings, same optical convention.

Why this and not zed_wrapper
----------------------------
`zed-ros2-wrapper` is the obvious answer and it is the right one on hardware
that can run it. This drone cannot: the board is a Tegra X1 on L4T R32.6.1, so
the ZED SDK is 4.0.1 built against CUDA 10.2 / Ubuntu 18.04-20.04, while ROS 2
Humble needs Ubuntu 22.04. Building the C++ wrapper against that mix is a much
larger surface than a few hundred lines of Python doing exactly the six
publications this stack needs. The Python binding is built from source against
the installed SDK headers — see scripts/build_pyzed.sh on the Jetson for why
the stock wheel does not work.

If the drone ever gets a board that runs a supported SDK + Humble combination,
delete this node and launch zed_wrapper instead. The contract above is what
makes that a drop-in swap, and it is why the topic names are the wrapper's and
not something of our own choosing.

IMU
---
The drone's camera is a ZED 2i: tracking fuses its IMU and the world is levelled
by gravity. A first-gen ZED (no IMU) also works, tracking is then visual-only
and the world is the lens' start pose; the odometry handles both. No
/zed/zed_node/imu/data is published, nothing in the stack reads it.

Frames
------
Same tree as zed_mimic_node:

    odom -> base_link                     (broadcast, from SDK tracking)
    base_link -> zed_camera_link          (static, the mount: xyz + rpy)
              -> zed_left_camera_frame    (static, +half baseline in y; cloud)
              -> zed_left_camera_optical_frame  (static, RPY(-90,0,-90), or
                                                 turned 180 about Z when the
                                                 images are rotated upright)

The drone's ZED sits upside down and 15 deg nose down (camera_rpy_deg
[180, 15, 0]). The SDK's image flip would hide that, but it turns IMU fusion
off, so the flip is done here instead and the mount says the truth.

`pad_detector_node` looks up base_link -> zed_left_camera_optical_frame and
nothing else, so the intermediate links exist to match the wrapper's tree rather
than because anything reads them.
"""

import array

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from hydrone_bringup.camera_mount import (
    OPTICAL_QUAT, OPTICAL_QUAT_ROT180, base_from_camera_pose, is_upside_down,
    mount_matrix, quat_from_matrix, quat_from_rpy_deg,
    rotate_intrinsics_180, transform)


FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_CAMERA = "zed_camera_link"
FRAME_LEFT_CAM = "zed_left_camera_frame"
FRAME_OPTICAL = "zed_left_camera_optical_frame"

RESOLUTIONS = {"VGA", "HD720", "HD1080", "HD2K"}
DEPTH_MODES = {"NONE", "PERFORMANCE", "QUALITY", "ULTRA", "NEURAL"}


class ZedSdkNode(Node):

    def __init__(self, **kwargs):
        super().__init__("zed_sdk", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        # VGA is 672x376 per eye at up to 100 fps and is the right default on a
        # Tegra X1: the depth pass is the expensive part and it scales with
        # pixels. HD720 works but leaves little CPU for the detectors.
        self.declare_parameter("resolution", "VGA")
        self.declare_parameter("fps", 15)
        # PERFORMANCE over ULTRA for the same reason. The pads are metres away
        # and metres wide; this is not a scene-reconstruction job.
        self.declare_parameter("depth_mode", "PERFORMANCE")
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 20.0)
        # The point cloud is only read by feature_map_node, which is an
        # observer. It is the single most expensive publication here, so it is
        # off unless asked for.
        self.declare_parameter("publish_point_cloud", False)
        self.declare_parameter("point_cloud_stride", 4)
        # The SDK's visual tracking, published as /zed/zed_node/odom and
        # relayed to the FCU by vision_odom_bridge. Turn it off if something
        # else owns the position estimate.
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        # The mount, base_link -> zed_camera_link (camera body centre).
        # Defaults are the t15 support (drones/supports/parts/zed_mount.py):
        # upside down, 15 deg nose down, body centre 8 cm ahead of and 3.7 cm
        # under the frame centre. rpy is [roll, pitch, yaw] deg, +pitch = down.
        self.declare_parameter("camera_offset_xyz", [0.080, 0.0, -0.037])
        self.declare_parameter("camera_rpy_deg", [180.0, 15.0, 0.0])
        # An upside-down camera gives upside-down images. Rotate them (and
        # depth) 180 deg so consumers see them upright; the optical frame and
        # cx/cy follow. The SDK's own flip is not used because it turns off
        # IMU fusion in tracking. No-op for an upright mount.
        self.declare_parameter("upright_images", True)
        self.declare_parameter("serial_number", 0)

        p = lambda n: self.get_parameter(n).value
        self.min_depth = float(p("min_depth_m"))
        self.max_depth = float(p("max_depth_m"))
        self.want_cloud = bool(p("publish_point_cloud"))
        self.cloud_stride = max(int(p("point_cloud_stride")), 1)
        self.want_odom = bool(p("publish_odom"))
        self.publish_tf = bool(p("publish_tf"))
        self.rpy_deg = [float(v) for v in p("camera_rpy_deg")]
        self.rotate_180 = bool(p("upright_images")) and is_upside_down(
            self.rpy_deg)

        resolution = str(p("resolution")).upper()
        depth_mode = str(p("depth_mode")).upper()
        if resolution not in RESOLUTIONS:
            raise ValueError(f"resolution must be one of {sorted(RESOLUTIONS)}")
        if depth_mode not in DEPTH_MODES:
            raise ValueError(f"depth_mode must be one of {sorted(DEPTH_MODES)}")

        # Imported here, not at module scope, so the package remains importable
        # (and testable) on a machine with no ZED SDK — which is every machine
        # except the drone.
        import pyzed.sl as sl
        self.sl = sl

        # ── Open the camera ─────────────────────────────────────────────────
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, resolution)
        init.camera_fps = int(p("fps"))
        init.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init.coordinate_units = sl.UNIT.METER
        # RIGHT_HANDED_Z_UP_X_FWD is ROS's convention (REP 103). Asking the SDK
        # for it means the odometry needs no axis juggling on the way out —
        # which is exactly the class of error that put the vehicle 90 degrees
        # from its own point cloud in sim (see map_odom_node).
        init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init.depth_minimum_distance = self.min_depth
        init.depth_maximum_distance = self.max_depth
        serial = int(p("serial_number"))
        if serial:
            init.set_from_serial_number(serial)

        self.cam = sl.Camera()
        status = self.cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"could not open the ZED: {status}")

        info = self.cam.get_camera_information()
        self.width = info.camera_configuration.resolution.width
        self.height = info.camera_configuration.resolution.height
        calib = info.camera_configuration.calibration_parameters.left_cam
        self.fx, self.fy = float(calib.fx), float(calib.fy)
        self.cx, self.cy = float(calib.cx), float(calib.cy)
        if self.rotate_180:
            self.cx, self.cy = rotate_intrinsics_180(
                self.cx, self.cy, self.width, self.height)
        # The SDK's images, cloud and pose all belong to the LEFT lens, half a
        # baseline to the camera's left (on the drone's right when inverted).
        self.half_baseline = float(
            info.camera_configuration.calibration_parameters
            .get_camera_baseline()) / 2.0
        # With an IMU the tracking world is levelled by gravity, without one it
        # is the lens' start pose; base_from_camera_pose needs to know which.
        self.gravity_world = bool(
            info.sensors_configuration.accelerometer_parameters.is_available)
        self.T_base_left = mount_matrix(
            p("camera_offset_xyz"), self.rpy_deg) @ transform(
            (0.0, self.half_baseline, 0.0), (0.0, 0.0, 0.0, 1.0))
        self.get_logger().info(
            f"ZED {info.camera_model} sn {info.serial_number} open at "
            f"{self.width}x{self.height} — fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}, depth {depth_mode}, "
            f"cloud {'on' if self.want_cloud else 'off'}, "
            f"odom {'on' if self.want_odom else 'off'}, mount rpy "
            f"{self.rpy_deg} deg, baseline {2 * self.half_baseline:.3f} m, "
            f"images {'rotated 180' if self.rotate_180 else 'as is'}, "
            f"tracking world {'gravity' if self.gravity_world else 'start pose'}")

        if self.want_odom:
            track = sl.PositionalTrackingParameters()
            self.cam.enable_positional_tracking(track)

        self._left = sl.Mat()
        self._depth = sl.Mat()
        self._cloud = sl.Mat()
        self._pose = sl.Pose()
        self._runtime = sl.RuntimeParameters()

        # ── I/O ─────────────────────────────────────────────────────────────
        # BEST_EFFORT with depth 1, matching what MAVROS and the sim sources use
        # for sensor streams and what pad_detector_node subscribes with. A
        # RELIABLE publisher against a BEST_EFFORT subscriber is compatible, but
        # keeping them identical means the sim and the drone exercise the same
        # QoS path — the mismatch that silently delivers nothing is the whole
        # reason test_pad_pipeline exists.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        self.pub_rgb = self.create_publisher(
            Image, "/zed/zed_node/rgb/image_rect_color", sensor_qos)
        self.pub_rgb_info = self.create_publisher(
            CameraInfo, "/zed/zed_node/rgb/camera_info", 10)
        self.pub_depth = self.create_publisher(
            Image, "/zed/zed_node/depth/depth_registered", sensor_qos)
        self.pub_depth_info = self.create_publisher(
            CameraInfo, "/zed/zed_node/depth/camera_info", 10)
        self.pub_cloud = self.create_publisher(
            PointCloud2, "/zed/zed_node/point_cloud/cloud_registered",
            sensor_qos)
        self.pub_odom = self.create_publisher(
            Odometry, "/zed/zed_node/odom", 10)

        self.tf_static = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._send_static_tfs()

        period = 1.0 / max(float(p("fps")), 1.0)
        self.create_timer(period, self._tick)

    # ────────────────────────────────────────────────────────────────────────
    # Static TF tree — the same chain zed_mimic_node publishes
    # ────────────────────────────────────────────────────────────────────────

    def _send_static_tfs(self):
        ox, oy, oz = self.get_parameter("camera_offset_xyz").value
        stamp = self.get_clock().now().to_msg()

        mount = TransformStamped()
        mount.header.stamp = stamp
        mount.header.frame_id = FRAME_BASE
        mount.child_frame_id = FRAME_CAMERA
        mount.transform.translation.x = float(ox)
        mount.transform.translation.y = float(oy)
        mount.transform.translation.z = float(oz)
        (mount.transform.rotation.x,
         mount.transform.rotation.y,
         mount.transform.rotation.z,
         mount.transform.rotation.w) = quat_from_rpy_deg(self.rpy_deg)

        # The SDK measures from the left lens, half a baseline to the left.
        left_cam = TransformStamped()
        left_cam.header.stamp = stamp
        left_cam.header.frame_id = FRAME_CAMERA
        left_cam.child_frame_id = FRAME_LEFT_CAM
        left_cam.transform.translation.y = self.half_baseline
        left_cam.transform.rotation.w = 1.0

        optical = TransformStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = FRAME_LEFT_CAM
        optical.child_frame_id = FRAME_OPTICAL
        (optical.transform.rotation.x,
         optical.transform.rotation.y,
         optical.transform.rotation.z,
         optical.transform.rotation.w) = (
            OPTICAL_QUAT_ROT180 if self.rotate_180 else OPTICAL_QUAT)

        self.tf_static.sendTransform([mount, left_cam, optical])

    # ────────────────────────────────────────────────────────────────────────
    # Per-frame work
    # ────────────────────────────────────────────────────────────────────────

    def _tick(self):
        sl = self.sl
        if self.cam.grab(self._runtime) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn("grab failed", throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()

        self.cam.retrieve_image(self._left, sl.VIEW.LEFT)
        self.pub_rgb.publish(self._rgb_msg(self._left, stamp))

        info = self._camera_info(stamp)
        self.pub_rgb_info.publish(info)
        self.pub_depth_info.publish(info)

        self.cam.retrieve_measure(self._depth, sl.MEASURE.DEPTH)
        self.pub_depth.publish(self._depth_msg(self._depth, stamp))

        if self.want_cloud:
            self.cam.retrieve_measure(self._cloud, sl.MEASURE.XYZ)
            self.pub_cloud.publish(self._cloud_msg(self._cloud, stamp))

        if self.want_odom:
            self._publish_odom(stamp)

    def _rgb_msg(self, mat, stamp) -> Image:
        """LEFT view as bgr8.

        The SDK hands back BGRA; the alpha channel is dropped rather than
        published, because every consumer in this stack reshapes the buffer by
        `step` and expects three channels — image_convert.py does the conversion
        in numpy precisely to avoid cv_bridge, and it does not know about BGRA.
        """
        bgra = mat.get_data()
        if self.rotate_180:
            bgra = bgra[::-1, ::-1]
        bgr = np.ascontiguousarray(bgra[:, :, :3])
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_OPTICAL
        msg.height, msg.width = bgr.shape[0], bgr.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        # array.array, NOT bytes. rclpy's uint8[] field converts a bytes
        # object element by element in Python: measured on the Jetson at
        # 361 ms for a 672x376x3 frame against 0.2 ms for an array.array,
        # which is the difference between 1.2 Hz and the camera's 15.
        msg.data = array.array("B", bgr.tobytes())
        return msg

    def _depth_msg(self, mat, stamp) -> Image:
        """Depth in metres, 32FC1, with the SDK's sentinels turned into NaN.

        The SDK marks unmeasurable pixels with +/-inf and NaN depending on why
        (too near, too far, occluded). REP 118 says NaN, and pad_detector_node's
        _depth_at filters on np.isfinite — an infinity that slipped through
        would be treated as a real reading and project a pad into the next
        county. zed_mimic_node does the same normalisation for the simulator's
        far-plane sentinel.
        """
        depth = np.asarray(mat.get_data(), dtype=np.float32)
        if self.rotate_180:
            depth = depth[::-1, ::-1]
        bad = ~np.isfinite(depth)
        bad |= (depth < self.min_depth) | (depth > self.max_depth)
        depth = np.where(bad, np.float32("nan"), depth)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_OPTICAL
        msg.height, msg.width = depth.shape[0], depth.shape[1]
        msg.encoding = "32FC1"
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        # array.array, NOT bytes. rclpy's uint8[] field converts a bytes
        # object element by element in Python: measured on the Jetson at
        # 361 ms for a 672x376x3 frame against 0.2 ms for an array.array,
        # which is the difference between 1.2 Hz and the camera's 15.
        msg.data = array.array("B",
                               np.ascontiguousarray(depth).tobytes())
        return msg

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = FRAME_OPTICAL
        info.width, info.height = int(self.width), int(self.height)
        # The SDK returns RECTIFIED images, so distortion is already removed and
        # the plumb-bob coefficients are zero. Publishing anything else here
        # would have the detector undistort an undistorted image.
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _cloud_msg(self, mat, stamp) -> PointCloud2:
        """XYZ cloud, subsampled, in the left lens frame.

        The SDK gives XYZ in init.coordinate_system (X fwd, Z up), NOT optical,
        so it is stamped zed_left_camera_frame. Never rotated with the images:
        it stays in the physical lens frame and TF places it.

        Strided rather than full: feature_map_node voxelises whatever it gets,
        so a quarter of the points cost a quarter of the bandwidth and change
        the resulting map very little — and on this board the bandwidth is the
        constraint.
        """
        xyz = np.asarray(mat.get_data(), dtype=np.float32)[..., :3]
        xyz = xyz[::self.cloud_stride, ::self.cloud_stride, :]
        xyz = xyz.reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_LEFT_CAM
        msg.height = 1
        msg.width = int(xyz.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        # array.array, NOT bytes. rclpy's uint8[] field converts a bytes
        # object element by element in Python: measured on the Jetson at
        # 361 ms for a 672x376x3 frame against 0.2 ms for an array.array,
        # which is the difference between 1.2 Hz and the camera's 15.
        msg.data = array.array("B", np.ascontiguousarray(xyz).tobytes())
        return msg

    def _publish_odom(self, stamp):
        """The SDK's visual tracking, as odom -> base_link.

        The pose the SDK reports is the LEFT LENS', tilted and maybe upside
        down. The mount (and the lever arm) is taken back out so this is the
        vehicle's — see camera_mount.base_from_camera_pose. With the IMU the
        world takes a few seconds after start to level (seen up to 10 deg off
        at 3 s, 0.3 deg from 6 s on the bench, 2026-11-05).
        """
        sl = self.sl
        state = self.cam.get_position(self._pose, sl.REFERENCE_FRAME.WORLD)
        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.get_logger().warn(
                f"positional tracking: {state}", throttle_duration_sec=5.0)
            return

        T_world_left = transform(self._pose.get_translation().get(),
                                 self._pose.get_orientation().get())
        T = base_from_camera_pose(T_world_left, self.T_base_left,
                                  self.gravity_world)
        base = T[:3, 3]
        q = quat_from_matrix(T[:3, :3])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = FRAME_ODOM
        odom.child_frame_id = FRAME_BASE
        odom.pose.pose.position.x = float(base[0])
        odom.pose.pose.position.y = float(base[1])
        odom.pose.pose.position.z = float(base[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        self.pub_odom.publish(odom)

        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = FRAME_ODOM
        tf.child_frame_id = FRAME_BASE
        tf.transform.translation.x = float(base[0])
        tf.transform.translation.y = float(base[1])
        tf.transform.translation.z = float(base[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        try:
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZedSdkNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
