#!/usr/bin/env python3
"""
down_cam_mimic_node — SIM-ONLY shim for the belly-mounted, down-facing camera.

Same idea as zed_mimic_node, one camera smaller: it takes what BiguaSim's bridge
publishes for the `DownCamera` sensor and re-emits it under the agnostic names
the autonomy layer subscribes to, with a proper stamp and frame.

    /biguasim/<agent>/DownCamera              ->  /down_cam/image_raw
    /biguasim/<agent>/DownCamera/camera_info  ->  /down_cam/camera_info

    TF (static):  base_link -> down_cam_link -> down_cam_optical_frame

Why the drone needs this camera at all
--------------------------------------
The ZED faces forward. It is what SPOTS a pad tens of metres ahead, but as the
drone closes in, the pad slides down and out of the forward frame precisely when
the alignment has to get tight. A nadir camera sees the pad the whole way down
and keeps seeing it while hovering over it. Detection is the same algorithm on
both — see hydrone_vision/pad_detector.py.

On the REAL drone this node does not run. A USB/CSI camera driver publishes
/down_cam/image_raw + /down_cam/camera_info directly, and a
static_transform_publisher supplies the same two TFs from the measured mount.
The contract is those two topics plus those two frames; nothing above cares who
produced them.

The mount rotation
------------------
`camera_rpy_deg` is the sensor's mounting rotation as written in BiguaSim's
config.yaml — [roll, pitch, yaw] in degrees, positive pitch = nose up. The
launch file reads it straight from that file so the simulated camera and the TF
can never disagree: one edit moves both.

ROS RPY about base_link (X fwd, Y left, Z up) is right-handed, so a nose-up
pitch is a NEGATIVE rotation about +Y there — hence the sign flip in
`_mount_quaternion`. If the sim's down camera ever comes out looking at the sky,
the fix is the single `rotation:` line in config.yaml, not this file.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


FRAME_BASE = "base_link"
FRAME_CAMERA = "down_cam_link"
FRAME_OPTICAL = "down_cam_optical_frame"

# camera_link -> camera_optical_frame: the standard ROS optical rotation,
# RPY(-90, 0, -90). Identical to zed_mimic's; kept here so the two nodes stay
# independent.
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)  # (x, y, z, w)


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    """RPY in radians (fixed-axis XYZ, i.e. R = Rz·Ry·Rx) -> (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class DownCamMimicNode(Node):

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy's Node so tests can pass
        # parameter_overrides; the parameters here are read once, in __init__.
        super().__init__("down_cam_mimic", **kwargs)

        self.declare_parameter("in_image", "/biguasim/uav0_id0/DownCamera")
        self.declare_parameter("in_info",
                               "/biguasim/uav0_id0/DownCamera/camera_info")
        self.declare_parameter("out_image", "/down_cam/image_raw")
        self.declare_parameter("out_info", "/down_cam/camera_info")
        # Mount pose on the body, from BiguaSim's config.yaml (see module docs).
        self.declare_parameter("camera_offset_xyz", [0.0, 0.0, -0.12])
        self.declare_parameter("camera_rpy_deg", [0.0, -90.0, 0.0])

        p = lambda name: self.get_parameter(name).value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_image = self.create_publisher(Image, p("out_image"), sensor_qos)
        self.pub_info = self.create_publisher(CameraInfo, p("out_info"),
                                              sensor_qos)

        self.tf_static = StaticTransformBroadcaster(self)
        self._send_static_tfs(p("camera_offset_xyz"), p("camera_rpy_deg"))

        # BiguaSim sends image and camera_info as separate messages of the same
        # frame, both with stamp 0. Cache the info and stamp the pair together on
        # the image, exactly as zed_mimic does, so a subscriber that
        # time-synchronizes them is not left waiting forever.
        self.last_info: CameraInfo | None = None
        self.create_subscription(CameraInfo, p("in_info"), self._cb_info, 10)
        self.create_subscription(Image, p("in_image"), self._cb_image, 10)

        self.get_logger().info(
            f"down_cam mimic up — {p('in_image')} -> {p('out_image')}")

    def _send_static_tfs(self, xyz, rpy_deg):
        stamp = self.get_clock().now().to_msg()

        mount = TransformStamped()
        mount.header.stamp = stamp
        mount.header.frame_id = FRAME_BASE
        mount.child_frame_id = FRAME_CAMERA
        mount.transform.translation.x = float(xyz[0])
        mount.transform.translation.y = float(xyz[1])
        mount.transform.translation.z = float(xyz[2])
        (mount.transform.rotation.x,
         mount.transform.rotation.y,
         mount.transform.rotation.z,
         mount.transform.rotation.w) = self._mount_quaternion(rpy_deg)

        optical = TransformStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = FRAME_CAMERA
        optical.child_frame_id = FRAME_OPTICAL
        (optical.transform.rotation.x,
         optical.transform.rotation.y,
         optical.transform.rotation.z,
         optical.transform.rotation.w) = OPTICAL_QUAT

        self.tf_static.sendTransform([mount, optical])
        self.get_logger().info(
            f"static TF {FRAME_BASE} -> {FRAME_CAMERA} at "
            f"({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}) rpy={list(rpy_deg)} deg "
            "(BiguaSim sense: +pitch = nose up)")

    @staticmethod
    def _mount_quaternion(rpy_deg):
        """BiguaSim mount RPY (degrees, +pitch = nose up) -> ROS quaternion.

        Only the pitch sign differs: ROS RPY about base_link is right-handed
        about +Y (left), where nose-up is negative. Roll and yaw already agree.
        """
        roll, pitch, yaw = (math.radians(float(v)) for v in rpy_deg)
        return _quat_from_rpy(roll, -pitch, yaw)

    def _cb_info(self, msg: CameraInfo):
        self.last_info = msg

    def _cb_image(self, msg: Image):
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_OPTICAL
        self.pub_image.publish(msg)

        if self.last_info is not None:
            self.last_info.header.stamp = stamp
            self.last_info.header.frame_id = FRAME_OPTICAL
            self.pub_info.publish(self.last_info)


def main(args=None):
    rclpy.init(args=args)
    node = DownCamMimicNode()
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
