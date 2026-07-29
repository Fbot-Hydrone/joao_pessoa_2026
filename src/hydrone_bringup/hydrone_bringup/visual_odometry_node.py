#!/usr/bin/env python3
"""
visual_odometry_node — a REAL visual-odometry estimate on the ZED RGB-D stream.

Instead of republishing BiguaSim ground truth (that stays on /zed/zed_node/odom_GT
via zed_mimic), this node *computes* odometry the way the real ZED 2i does at its
core: the ZED SDK's positional tracking is a stereo visual-inertial SLAM whose
motion core is **Stereo Visual Odometry — it tracks visual features in 3D across
frames** (Stereolabs docs). We reproduce that core:

    detect features -> match to previous frame -> back-project matches to 3D using
    the depth image -> estimate the frame-to-frame camera motion (PnP + RANSAC) ->
    accumulate the pose.

Depth here IS the sim's stereo output, so this is an honest analogue of the ZED's
feature-tracking-in-3D odometry. IMU fusion and a sparse landmark map (the "I" and
the SLAM in ZED's VIO/SLAM) are documented next steps, not done here.

Interface (identical to the ground-truth odom, so vision_odom_bridge is unchanged):
  in:  /zed/zed_node/rgb/image_rect_color   sensor_msgs/Image  (bgr8)
       /zed/zed_node/depth/depth_registered sensor_msgs/Image  (32FC1, meters, NaN=invalid)
       /zed/zed_node/rgb/camera_info        sensor_msgs/CameraInfo
  out: /zed/zed_node/odom                    nav_msgs/Odometry  (odom -> base_link)

Frames
------
The VO runs in the camera OPTICAL frame (Z fwd, X right, Y down). Output must be in
base_link (X fwd, Y left, Z up) so the rest of the pipeline (which expects BiguaSim's
NWU-style odom) is unchanged. We fix the odom origin at the first frame (pose=identity)
and map optical motion into base via the constant optical<->base rotation:
    T_odom_base(t) = C · T_optical(t) · C⁻¹,   C = base_from_optical.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

FRAME_ODOM = "odom"
FRAME_BASE = "base_link"

# base_from_optical rotation (maps a vector in the optical frame into base_link).
# Inverse of zed_mimic's camera_link->optical quaternion (-0.5,0.5,-0.5,0.5): as a
# matrix it sends optical Z(fwd)->base X(fwd), optical X(right)->base -Y, optical
# Y(down)->base -Z. (Verified equal to the quaternion's rotation matrix.)
R_BASE_FROM_OPTICAL = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def _quat_from_matrix(R):
    """Rotation matrix (3x3) -> quaternion (x, y, z, w). Standard Shepperd method."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class VisualOdometryNode(Node):

    def __init__(self):
        super().__init__("visual_odometry")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("in_rgb", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("in_depth", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("in_info", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("out_odom", "/zed/zed_node/odom")
        # ORB budget. More features = steadier pose, more CPU.
        self.declare_parameter("max_features", 1000)
        # Lowe ratio for the 2-NN match filter (lower = stricter).
        self.declare_parameter("match_ratio", 0.75)
        # Depth gating (meters): reject back-projections outside a sane band.
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 20.0)
        # Minimum RANSAC inliers to trust a motion estimate.
        self.declare_parameter("min_inliers", 12)

        self.max_features = int(self.get_parameter("max_features").value)
        self.match_ratio = float(self.get_parameter("match_ratio").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=self.max_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        # ── VO state ────────────────────────────────────────────────────────
        self.K = None                 # 3x3 intrinsics, from camera_info
        self.last_depth = None        # latest depth frame (meters), cached
        self.prev_depth = None        # depth aligned with prev_kp/prev_des
        self.prev_pts = None          # Nx2 keypoint pixel coords of prev frame
        self.prev_des = None
        # Accumulated camera pose in the OPTICAL start-frame (4x4).
        self.pose_opt = np.eye(4)

        # ── I/O ─────────────────────────────────────────────────────────────
        p = lambda n: self.get_parameter(n).value
        self.pub_odom = self.create_publisher(Odometry, p("out_odom"), 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, p("in_info"), self._cb_info, 10)
        self.create_subscription(Image, p("in_depth"), self._cb_depth, 10)
        self.create_subscription(Image, p("in_rgb"), self._cb_rgb, 10)

        self.get_logger().info("Visual odometry ready — estimating /zed/zed_node/odom")

    # ─────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _cb_depth(self, msg: Image):
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def _backproject(self, pts, depth):
        """Nx2 pixel coords -> Nx3 optical-frame points + valid mask (vectorized)."""
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        h, w = depth.shape
        u = np.rint(pts[:, 0]).astype(np.int32)
        v = np.rint(pts[:, 1]).astype(np.int32)
        in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        uc = np.clip(u, 0, w - 1)
        vc = np.clip(v, 0, h - 1)
        d = depth[vc, uc]
        valid = in_img & np.isfinite(d) & (d >= self.min_depth) & (d <= self.max_depth)
        d_safe = np.where(valid, d, 1.0)  # avoid nan propagation; masked out anyway
        x = (u - cx) / fx * d_safe
        y = (v - cy) / fy * d_safe
        pts3d = np.stack([x, y, d_safe], axis=1)
        return pts3d, valid

    def _cb_rgb(self, msg: Image):
        if self.K is None or self.last_depth is None:
            return

        gray = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"),
            cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)
        depth = self.last_depth
        pts = (np.array([k.pt for k in kp], dtype=np.float64)
               if kp else np.empty((0, 2)))

        if des is not None and self.prev_des is not None and len(kp) >= self.min_inliers:
            # 3D points from PREVIOUS frame's depth; 2D from the current frame.
            prev_pts3d, valid = self._backproject(self.prev_pts, self.prev_depth)
            self._match_and_update(pts, des, prev_pts3d, valid)

        # Roll the reference frame forward and publish (holds pose if no update).
        self.prev_depth = depth
        self.prev_pts = pts
        self.prev_des = des
        self._publish(msg.header.stamp)

    def _match_and_update(self, cur_pts, des, prev_pts3d, valid):
        pairs = self.matcher.knnMatch(self.prev_des, des, k=2)
        obj, img = [], []
        for m_n in pairs:
            if len(m_n) < 2:
                continue
            m, n = m_n
            if m.distance > self.match_ratio * n.distance:
                continue
            if not valid[m.queryIdx]:
                continue
            obj.append(prev_pts3d[m.queryIdx])
            img.append(cur_pts[m.trainIdx])

        if len(obj) < self.min_inliers:
            self.get_logger().warn(
                f"VO: only {len(obj)} usable matches (<{self.min_inliers}); holding pose")
            return

        obj = np.asarray(obj, dtype=np.float64)
        img = np.asarray(img, dtype=np.float64)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj, img, self.K, None,
            iterationsCount=100, reprojectionError=2.0, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None or len(inliers) < self.min_inliers:
            n = 0 if inliers is None else len(inliers)
            self.get_logger().warn(f"VO: PnP failed / {n} inliers; holding pose")
            return

        # solvePnP gives T_cur_prev: p_cur = R·p_prev + t. The camera pose update
        # is the inverse of that relative motion.
        R, _ = cv2.Rodrigues(rvec)
        T_cur_prev = np.eye(4)
        T_cur_prev[:3, :3] = R
        T_cur_prev[:3, 3] = tvec.ravel()
        self.pose_opt = self.pose_opt @ np.linalg.inv(T_cur_prev)

    def _publish(self, stamp):
        # Map the accumulated optical-frame pose into base_link (NWU-style):
        # T_odom_base = C · T_optical · C⁻¹, C = base_from_optical (rotation only).
        C = R_BASE_FROM_OPTICAL
        R_base = C @ self.pose_opt[:3, :3] @ C.T
        t_base = C @ self.pose_opt[:3, 3]
        qx, qy, qz, qw = _quat_from_matrix(R_base)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = FRAME_ODOM
        odom.child_frame_id = FRAME_BASE
        odom.pose.pose.position.x = float(t_base[0])
        odom.pose.pose.position.y = float(t_base[1])
        odom.pose.pose.position.z = float(t_base[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = FRAME_BASE
        t.transform.translation.x = float(t_base[0])
        t.transform.translation.y = float(t_base[1])
        t.transform.translation.z = float(t_base[2])
        t.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
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
