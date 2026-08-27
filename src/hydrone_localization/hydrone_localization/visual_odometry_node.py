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

    def __init__(self, **kwargs):
        # **kwargs so a test can pass parameter_overrides, like every other
        # node in this stack does.
        super().__init__("visual_odometry", **kwargs)

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("in_rgb", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("in_depth", "/zed/zed_node/depth/depth_registered")
        # STEREO. The ZED 2i is a stereo camera: its tracker follows features
        # and gets their range by MATCHING the two eyes, not by reading a depth
        # image someone else computed. Consuming the sim's depth was a shortcut
        # the real drone does not have, and it hid the errors the real one
        # makes — a feature the right eye cannot match has NO depth here, where
        # the sim's depth image would have handed over a perfect number.
        #
        # This is for ODOMETRY ONLY. The mapping stack, the pad detectors and
        # odom_GT keep using the sim's depth exactly as before.
        #
        # in_right empty falls back to in_depth, which is the real drone's
        # configuration too (zed_wrapper publishes its own depth there).
        self.declare_parameter("in_right", "/zed/zed_node/right/image_rect_color")
        self.declare_parameter("in_right_info", "/zed/zed_node/right/camera_info")
        # Row tolerance for a stereo match, in pixels. The pair is rectified,
        # so a left feature's partner is on the same row; this allows for the
        # detector's own sub-pixel jitter and any small mounting error.
        self.declare_parameter("stereo_row_tol", 2.0)
        # Smallest disparity worth triangulating, in pixels. Range error is
        # dZ/Z = dd/d, so at 1 px of matching error a disparity of 4 px is
        # already 25% range error — and at fx=320, B=0.12 that is 9.6 m away.
        # Below this the feature is effectively at infinity and contributes
        # nothing but noise to PnP.
        self.declare_parameter("min_disparity", 4.0)
        # Largest disparity worth searching, in pixels. fx * B / d at d = 128
        # is 0.30 m, closer than anything the drone can be to a wall without
        # having hit it, and it bounds the per-feature search.
        self.declare_parameter("max_disparity", 128.0)
        self.declare_parameter("in_info", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("out_odom", "/zed/zed_node/odom")
        # ORB budget. More features = steadier pose, more CPU.
        self.declare_parameter("max_features", 1000)
        # FAST corner threshold. OpenCV's default is 20, which is tuned for
        # ordinary textured scenes and is far too strict for this arena: matte
        # white wall on blown-out white floor under a smooth sky. MEASURED on a
        # live frame 2026-08-20 (see feature_map_node): 46 keypoints in the
        # whole image, all of them inside a 30-pixel band on the horizon line,
        # because the horizon is the only intensity gradient there is. With so
        # few corners the tracker loses them the moment the drone turns, which
        # is the "only 0 usable matches" in the logs.
        self.declare_parameter("fast_threshold", 7)
        # Contrast-limited adaptive histogram equalisation, applied before
        # detection. The arena is not textureless — it is LOW CONTRAST, which
        # is a different problem with a standard answer. CLAHE stretches
        # contrast per tile, so the faint gradients on a white wall (panel
        # seams, scuffs, the shading of a corner) rise above the corner
        # threshold instead of being flattened by the bright floor and sky.
        # clip_limit 0 disables it.
        self.declare_parameter("clahe_clip", 3.0)
        self.declare_parameter("clahe_grid", 8)
        # Lowe ratio for the 2-NN match filter (lower = stricter).
        self.declare_parameter("match_ratio", 0.75)
        # Depth gating (meters): reject back-projections outside a sane band.
        self.declare_parameter("min_depth", 0.3)
        # 6 m, and this is a STEREO limit, not an arena one. Triangulated depth
        # degrades as Z^2: dZ = Z^2 * dd / (fx * B), and with the sim pair's
        # fx=320 px and B=0.12 m one pixel of disparity is worth 0.65 m at 5 m
        # and 2.6 m at 10 m. MEASURED against the sim's own depth image:
        #
        #     1-3 m   |error| median 0.305 m
        #     3-6 m   |error| median 2.535 m
        #     6-12 m  |error| median 1.864 m
        #
        # A feature at 10 m carries metres of range error into PnP, and PnP
        # weights it like any other. It was 20 m, which let the whole far half
        # of the arena vote. The real ZED 2i has the same physics with a longer
        # focal length, so this bound belongs here on the real drone too — it
        # just sits further out.
        self.declare_parameter("max_depth", 6.0)
        # Minimum RANSAC inliers to trust a motion estimate.
        self.declare_parameter("min_inliers", 12)
        # Minimum inlier RATIO (inliers / usable matches). 12 inliers out of 13
        # matches is a solid solution; 12 out of 80 is RANSAC scraping together
        # a story from noise. Measured 2026-08-18: one such frame moved the pose
        # 11 m / 95 deg in a single step while the drone sat still on the pad —
        # and with GPS off that jump WAS the EKF position, so the vehicle armed
        # 10 m from where it believed it was and flew away. The absolute count
        # alone cannot catch that; the ratio does.
        self.declare_parameter("min_inlier_ratio", 0.5)
        # Physical plausibility bounds on ONE frame-to-frame step. At 20 Hz
        # even 2 m/s of real motion is 0.1 m/frame; a solution claiming more
        # than max_step_m or max_step_deg in one frame is a degenerate PnP fit,
        # not motion. Rejected steps hold pose (exactly like a starved frame).
        self.declare_parameter("max_step_m", 0.5)
        self.declare_parameter("max_step_deg", 20.0)
        # ZERO-VELOCITY UPDATE. The bound above rejects steps too BIG to be
        # real; these reject steps too SMALL. PnP never returns exactly zero —
        # feature positions carry sub-pixel noise, so a perfectly still camera
        # still yields a few millimetres of "motion" every frame, with inliers
        # to spare. Every gate above passes it, and integrating it forever is
        # pure invented distance.
        #
        # MEASURED from odom_error_20260827_010810.csv, a 51.5 m flight:
        #   94% of samples the vehicle was LITERALLY still (ground truth moved
        #   < 0.01 mm) and the VO reported 441.8 m of travel across them —
        #   1.61 mm per frame, relentlessly. Total: 51.5 m flown, 527.3 m
        #   reported, a 10.2x inflation. While actually moving the same VO is
        #   fine: 50.6 m of ground truth against 58.2 m reported, 1.15x.
        #
        # So essentially ALL of the drift is invented while standing still, and
        # that is what these thresholds delete. 5 mm sits 3x above the measured
        # noise and ~10x below one frame of real flight (0.5 m/s at 10 Hz is
        # 50 mm/frame), so there is a wide margin on both sides.
        #
        # The real ZED does this too — the SDK detects a static state and holds
        # its pose, which is why the hardware looked better than the sim.
        self.declare_parameter("min_step_m", 0.005)
        # Angular counterpart. NOT measured yet — the CSV logs accumulated yaw
        # error, not per-frame rotation noise, so this is a starting value
        # chosen to be small against any real turn (ATC_SLEW_YAW-limited turns
        # are degrees per frame, not hundredths). Worth measuring.
        self.declare_parameter("min_step_deg", 0.25)
        # Whether this node owns the odom->base_link TF. False when it is
        # demoted to an observer publishing to a side topic while ground
        # truth flies the vehicle — two broadcasters of the same transform
        # is a corrupt TF tree. See odom_source in sources_sim.launch.py.
        self.declare_parameter("publish_tf", True)

        self.max_features = int(self.get_parameter("max_features").value)
        self.fast_threshold = int(self.get_parameter("fast_threshold").value)
        clip = float(self.get_parameter("clahe_clip").value)
        grid = int(self.get_parameter("clahe_grid").value)
        self.clahe = (cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))
                      if clip > 0 else None)
        self.match_ratio = float(self.get_parameter("match_ratio").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)
        self.min_inlier_ratio = float(self.get_parameter("min_inlier_ratio").value)
        self.max_step_m = float(self.get_parameter("max_step_m").value)
        self.max_step_rad = np.radians(
            float(self.get_parameter("max_step_deg").value))
        self.min_step_m = float(self.get_parameter("min_step_m").value)
        self.min_step_rad = np.radians(
            float(self.get_parameter("min_step_deg").value))
        self.n_static = 0
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=self.max_features,
                                  fastThreshold=self.fast_threshold)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        # ── VO state ────────────────────────────────────────────────────────
        self.K = None                 # 3x3 intrinsics, from camera_info
        self.last_depth = None        # latest depth frame (meters), cached
        self.last_right = None        # latest right-eye frame, for stereo
        self.baseline = None          # metres, from the right camera_info
        self.n_stereo_empty = 0       # frames where matching found nothing
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

        self._in_right = p("in_right")
        if self._in_right:
            self.create_subscription(Image, self._in_right, self._cb_right, 10)
            self.create_subscription(CameraInfo, p("in_right_info"),
                                     self._cb_right_info, 10)
        self.stereo_row_tol = float(p("stereo_row_tol"))
        self.min_disparity = float(p("min_disparity"))
        self.max_disparity = float(p("max_disparity"))

        self.get_logger().info("Visual odometry ready — estimating /zed/zed_node/odom")

    # ─────────────────────────────────────────────────────────────────────────

    def _cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _cb_right(self, msg: Image):
        self.last_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _cb_right_info(self, msg: CameraInfo):
        """Baseline from P[3] = -fx * B, which is where REP 104 puts it."""
        fx = msg.p[0]
        if fx:
            self.baseline = abs(msg.p[3] / fx)

    def _cb_depth(self, msg: Image):
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def _backproject(self, pts, depth):
        """Nx2 pixel coords -> Nx3 optical-frame points + valid mask.

        `depth` is either a per-KEYPOINT array (stereo) or a depth IMAGE (the
        fallback). Both end up as one range per point; only the lookup differs.
        """
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        u = np.rint(pts[:, 0]).astype(np.int32)
        v = np.rint(pts[:, 1]).astype(np.int32)
        if depth.ndim == 1:
            # Stereo: one range per keypoint, already aligned with pts.
            in_img = np.ones(len(pts), dtype=bool)
            d = depth if len(depth) == len(pts) else np.full(len(pts), np.nan)
        else:
            h, w = depth.shape
            in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            d = depth[np.clip(v, 0, h - 1), np.clip(u, 0, w - 1)]
        valid = in_img & np.isfinite(d) & (d >= self.min_depth) & (d <= self.max_depth)
        d_safe = np.where(valid, d, 1.0)  # avoid nan propagation; masked out anyway
        x = (u - cx) / fx * d_safe
        y = (v - cy) / fy * d_safe
        pts3d = np.stack([x, y, d_safe], axis=1)
        return pts3d, valid

    def _stereo_match(self, kp_left, des_left, left_gray):
        """Range for each left keypoint, by matching it in the RIGHT eye.

        Returns an array of depths in metres, NaN where the feature could not
        be matched, aligned with kp_left.

        SPARSE, not a dense disparity map, and the difference is the whole
        point. A dense matcher (SGBM) optimises over the image and does badly
        exactly where a corner detector likes to put keypoints: silhouettes,
        occlusion boundaries, thin structure. MEASURED on a live frame — of 981
        ORB keypoints, SGBM had disparity for only 267, so 73% of the features
        the tracker wanted were thrown away, and the ones left were biased
        toward flat interiors. That is why dense stereo made odometry WORSE
        than reading the sim's depth image.

        Matching the features themselves puts the range exactly where the
        tracker needs it, and it is what the ZED's own tracker does.

        The pair is rectified (identical intrinsics, pure horizontal offset),
        so a left feature's partner lies on the SAME row, at a smaller x. Both
        facts are used as filters — they are free and they reject most wrong
        matches before the descriptor has to.
        """
        if (self.last_right is None or self.baseline is None
                or self.K is None or des_left is None or not kp_left):
            return None

        right = cv2.cvtColor(self.last_right, cv2.COLOR_BGR2GRAY)
        if right.shape != left_gray.shape:
            self.get_logger().error(
                f"stereo pair mismatched: left {left_gray.shape} vs right "
                f"{right.shape}; the two eyes must share resolution",
                throttle_duration_sec=30.0)
            return None
        # The SAME equalisation on both eyes: ORB describes intensity
        # gradients, so equalising one and not the other breaks every match.
        if self.clahe is not None:
            right = self.clahe.apply(right)

        kp_r, des_r = self.orb.detectAndCompute(right, None)
        depths = np.full(len(kp_left), np.nan, dtype=np.float64)
        if des_r is None or len(kp_r) < 1:
            return depths

        # Bucket the right eye's features BY ROW, then search only the rows a
        # partner can be on. Doing it the other way round — matching against
        # the whole image and filtering by row afterwards — is what a naive
        # implementation does, and on this arena it throws away most of the
        # scene: MEASURED, 902 left keypoints against 901 right ones produced
        # only 126 Lowe-surviving matches, because a white wall repeats and the
        # second-best descriptor anywhere in the frame is nearly as good as the
        # best. Once the candidates are restricted to the epipolar line, the
        # runner-up is a genuinely different place and the ratio test means
        # what it is supposed to mean.
        rows = {}
        for i, k in enumerate(kp_r):
            rows.setdefault(int(round(k.pt[1])), []).append(i)

        tol = int(np.ceil(self.stereo_row_tol))
        fx = float(self.K[0, 0])
        for i, k in enumerate(kp_left):
            u, v = k.pt
            cand = []
            for r in range(int(round(v)) - tol, int(round(v)) + tol + 1):
                cand.extend(rows.get(r, ()))
            if not cand:
                continue
            # A partner is always to the LEFT in the right image (positive
            # disparity), and far enough over to be worth triangulating.
            cand = [j for j in cand
                    if self.min_disparity <= u - kp_r[j].pt[0] <= self.max_disparity]
            if not cand:
                continue

            d = np.asarray([cv2.norm(des_left[i], des_r[j], cv2.NORM_HAMMING)
                            for j in cand])
            order = np.argsort(d)
            best = order[0]
            if len(order) > 1 and d[best] > self.match_ratio * d[order[1]]:
                continue          # ambiguous ON THE LINE: genuinely unusable
            disparity = u - kp_r[cand[best]].pt[0]
            depths[i] = fx * self.baseline / disparity
        return depths

    def _cb_rgb(self, msg: Image):
        if self.K is None:
            return

        gray = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"),
            cv2.COLOR_BGR2GRAY)
        if self.clahe is not None:
            gray = self.clahe.apply(gray)

        kp, des = self.orb.detectAndCompute(gray, None)
        pts = (np.array([k.pt for k in kp], dtype=np.float64)
               if kp else np.empty((0, 2)))

        # Range for the features about to be tracked, from the RIGHT eye —
        # one depth per keypoint, NaN where it could not be matched. Falls back
        # to the depth image for a rig without a second eye, which is also how
        # the real drone runs (zed_wrapper computes depth on the camera).
        depth = self._stereo_match(kp, des, gray)
        if depth is None:
            if self.last_depth is None:
                return
            depth = self.last_depth
        elif not np.isfinite(depth).any():
            self.n_stereo_empty += 1
            self.get_logger().warn(
                f"VO: no feature matched in the right eye "
                f"[{self.n_stereo_empty}]; holding pose",
                throttle_duration_sec=10.0)
            self.prev_depth, self.prev_pts, self.prev_des = depth, pts, des
            self._publish(msg.header.stamp)
            return

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

        ratio = len(inliers) / len(obj)
        if ratio < self.min_inlier_ratio:
            self.get_logger().warn(
                f"VO: inlier ratio {ratio:.2f} ({len(inliers)}/{len(obj)}) "
                f"< {self.min_inlier_ratio:.2f}; holding pose")
            return

        # solvePnP gives T_cur_prev: p_cur = R·p_prev + t. The camera pose update
        # is the inverse of that relative motion.
        step_t = float(np.linalg.norm(tvec))
        step_r = float(np.linalg.norm(rvec))
        if step_t > self.max_step_m or step_r > self.max_step_rad:
            self.get_logger().warn(
                f"VO: implausible step {step_t:.2f} m / "
                f"{np.degrees(step_r):.1f} deg in one frame; holding pose")
            return

        # Zero-velocity update: too small to be motion, so it is noise, and
        # noise integrated is invented distance. BOTH have to be small — a
        # vehicle turning on the spot translates by nothing and must still have
        # its rotation integrated, which is exactly what Phase 1's search does.
        if step_t < self.min_step_m and step_r < self.min_step_rad:
            self.n_static += 1
            self.get_logger().info(
                f"VO: still ({step_t * 1000:.1f} mm / "
                f"{np.degrees(step_r):.2f} deg); holding pose "
                f"[{self.n_static} frames so far]",
                throttle_duration_sec=30.0)
            return

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

        if not self.publish_tf:
            return
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
