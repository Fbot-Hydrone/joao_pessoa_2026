#!/usr/bin/env python3
"""
down_cam_usb_node — the real belly camera, publishing the bus down_cam_mimic fakes.

REAL HARDWARE ONLY. Opens a plain USB (V4L2/UVC) camera and publishes the
contract the belly detector already consumes:

    /dev/videoN  -->  /down_cam/image_raw    (bgr8)
                      /down_cam/camera_info
                      TF: base_link -> down_cam_link -> down_cam_optical_frame

Nothing above the sources can tell this apart from the simulator.

CALIBRATION — read this before trusting a landing
--------------------------------------------------
The belly camera has no depth. `pad_detector_node` places a pad by intersecting
the pixel's ray with the ground plane, and that ray is built entirely from
fx/fy/cx/cy. **The camera_info this node publishes decides where the drone
thinks the pad is**, and an error there is a lateral landing error, not a
detection failure — the pad is found, confidently, in the wrong place.

`calibrated` is false by default and the node says so, loudly, once a second.
In that state fx/fy come from `nominal_hfov_deg`, the manufacturer's figure for
the lens, which is a starting point and nothing more: it assumes zero
distortion, a perfectly centred sensor, and that the published FOV is the real
one. It is enough to prove that detection and mapping work on real images. It is
not enough to land on a 1 m pad.

To fix it properly, run the standard checkerboard calibration:

    ros2 run camera_calibration cameracalibrator \\
        --size 8x6 --square 0.025 image:=/down_cam/image_raw camera:=/down_cam

then pass the numbers it prints as `fx`, `fy`, `cx`, `cy`, `distortion` and set
`calibrated:=true`. The warning goes away and the projection means something.

Rolling shutter and exposure
----------------------------
A cheap webcam auto-exposes, and over a dark floor it will hunt — which changes
the pad's apparent saturation frame to frame, which is exactly the axis the HSV
thresholds are tuned on. `exposure_auto` is exposed so it can be pinned once the
arena's lighting is known; see docs/LANDING-SITES.md §3 for why saturation is
the fragile part of the detector.
"""

import array
import math

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


FRAME_BASE = "base_link"
FRAME_DOWN_CAM = "down_cam_link"
FRAME_DOWN_OPTICAL = "down_cam_optical_frame"

# Robot convention (X fwd, Y left, Z up) -> optical (Z fwd, X right, Y down).
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)   # (x, y, z, w)


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class DownCamUsbNode(Node):

    def __init__(self, **kwargs):
        super().__init__("down_cam_usb", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        # MJPG lets a USB 2.0 webcam reach full frame rate at 640x480; YUYV at
        # that size saturates the bus on some hubs. cv2 decodes either.
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("exposure_auto", True)

        # ── Calibration ─────────────────────────────────────────────────────
        self.declare_parameter("calibrated", False)
        # Logitech C270: 55 deg diagonal, ~60 deg horizontal at 4:3. A NOMINAL
        # figure, not a measurement — see the module docstring.
        self.declare_parameter("nominal_hfov_deg", 60.0)
        self.declare_parameter("fx", 0.0)
        self.declare_parameter("fy", 0.0)
        self.declare_parameter("cx", 0.0)
        self.declare_parameter("cy", 0.0)
        self.declare_parameter("distortion", [0.0, 0.0, 0.0, 0.0, 0.0])

        # ── Mount ───────────────────────────────────────────────────────────
        # Where the lens sits relative to base_link, and how it is turned. The
        # sim's belly camera is 0.12 m below the origin looking straight down;
        # these are the SAME numbers as a starting point, and the real mount is
        # whatever is actually bolted to the airframe. Get this wrong and every
        # pad lands in the map offset by the error, in the direction the camera
        # is really pointing.
        self.declare_parameter("mount_xyz", [0.0, 0.0, -0.12])
        self.declare_parameter("mount_rpy_deg", [0.0, 90.0, 0.0])

        p = lambda n: self.get_parameter(n).value
        self.width = int(p("width"))
        self.height = int(p("height"))
        self.calibrated = bool(p("calibrated"))

        # ── Open the device ─────────────────────────────────────────────────
        device = str(p("device"))
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"could not open {device}")
        fourcc = str(p("fourcc"))
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, int(p("fps")))
        # A driver may refuse the request and hand back its nearest mode. Trust
        # what it reports, not what we asked for: camera_info must describe the
        # image that actually arrives.
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or self.width
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or self.height
        if not bool(p("exposure_auto")):
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual on V4L2

        self._resolve_intrinsics()

        # ── I/O ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_image = self.create_publisher(
            Image, "/down_cam/image_raw", sensor_qos)
        self.pub_info = self.create_publisher(
            CameraInfo, "/down_cam/camera_info", 10)

        self.tf_static = StaticTransformBroadcaster(self)
        self._send_static_tf()

        self.create_timer(1.0 / max(float(p("fps")), 1.0), self._tick)
        if not self.calibrated:
            self.create_timer(1.0, self._nag)

        self.get_logger().info(
            f"down_cam_usb on {device} at {self.width}x{self.height} — "
            f"fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} "
            f"cy={self.cy:.1f} "
            f"({'calibrated' if self.calibrated else 'NOMINAL, uncalibrated'})")

    def _resolve_intrinsics(self):
        p = lambda n: self.get_parameter(n).value
        if self.calibrated:
            self.fx, self.fy = float(p("fx")), float(p("fy"))
            self.cx, self.cy = float(p("cx")), float(p("cy"))
            self.distortion = [float(v) for v in p("distortion")]
            if self.fx <= 0.0 or self.fy <= 0.0:
                raise ValueError(
                    "calibrated:=true but fx/fy are not set. Either supply the "
                    "calibration or leave calibrated false.")
            return
        # Nominal: a pinhole with the lens's published horizontal field of
        # view, principal point dead centre, no distortion.
        hfov = math.radians(float(p("nominal_hfov_deg")))
        self.fx = self.fy = (self.width / 2.0) / math.tan(hfov / 2.0)
        self.cx, self.cy = self.width / 2.0, self.height / 2.0
        self.distortion = [0.0] * 5

    def _nag(self):
        self.get_logger().warn(
            "belly camera is NOT CALIBRATED — camera_info is a nominal "
            f"{self.get_parameter('nominal_hfov_deg').value:.0f} deg pinhole. "
            "Pad positions from this camera are approximate and a landing on "
            "them will be off by the calibration error. Run "
            "camera_calibration and set calibrated:=true before flying.",
            throttle_duration_sec=30.0)

    def _send_static_tf(self):
        """base_link -> down_cam_link -> down_cam_optical_frame.

        Split in two exactly as the simulated mount is: the first transform is
        the physical mount in robot axes, which is the number a person measures
        off the airframe, and the second is the fixed robot->optical convention
        rotation that every ROS camera carries. Keeping them apart means the
        mount can be corrected without anybody having to reason about optical
        axes. test_pad_projection.py pins the sim mount against this one.
        """
        mx, my, mz = self.get_parameter("mount_xyz").value
        roll, pitch, yaw = (math.radians(float(v))
                            for v in self.get_parameter("mount_rpy_deg").value)
        stamp = self.get_clock().now().to_msg()

        mount = TransformStamped()
        mount.header.stamp = stamp
        mount.header.frame_id = FRAME_BASE
        mount.child_frame_id = FRAME_DOWN_CAM
        mount.transform.translation.x = float(mx)
        mount.transform.translation.y = float(my)
        mount.transform.translation.z = float(mz)
        (mount.transform.rotation.x,
         mount.transform.rotation.y,
         mount.transform.rotation.z,
         mount.transform.rotation.w) = quat_from_rpy(roll, pitch, yaw)

        optical = TransformStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = FRAME_DOWN_CAM
        optical.child_frame_id = FRAME_DOWN_OPTICAL
        (optical.transform.rotation.x,
         optical.transform.rotation.y,
         optical.transform.rotation.z,
         optical.transform.rotation.w) = OPTICAL_QUAT

        self.tf_static.sendTransform([mount, optical])

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("frame grab failed",
                                   throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_DOWN_OPTICAL
        msg.height, msg.width = frame.shape[0], frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        # array.array, NOT bytes. rclpy's uint8[] field converts a bytes
        # object element by element in Python: measured on the Jetson at
        # 361 ms for a 672x376x3 frame against 0.2 ms for an array.array,
        # which is the difference between 1.2 Hz and the camera's 15.
        msg.data = array.array("B",
                               np.ascontiguousarray(frame).tobytes())
        self.pub_image.publish(msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = FRAME_DOWN_OPTICAL
        info.width, info.height = int(msg.width), int(msg.height)
        info.distortion_model = "plumb_bob"
        info.d = list(self.distortion)
        info.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DownCamUsbNode()
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
