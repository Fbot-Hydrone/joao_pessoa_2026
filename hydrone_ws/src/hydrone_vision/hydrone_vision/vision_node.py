#!/usr/bin/env python3
"""
hydrone_vision — Vision node for the Hydrone competition stack.

Responsibilities:
  - Phase 1 & 2: Detect the 6 landing bases using the ZED2 camera
  - Phase 3:     Detect human operator and recognize gestures/poses for HRI
  - Phase 4:     Read QR codes (A-E) inside the confined maze environment

Subscribed topics:
  /zed2/zed_node/rgb/image_rect_color   (sensor_msgs/Image)
  /zed2/zed_node/depth/depth_registered (sensor_msgs/Image)
  /zed2/zed_node/point_cloud/cloud_registered (sensor_msgs/PointCloud2)
  /hydrone/mission_state                (hydrone_msgs/MissionState)

Published topics:
  /hydrone/vision/landing_bases         (hydrone_msgs/LandingBase[])  — array of detected bases
  /hydrone/vision/human_gesture         (hydrone_msgs/HumanGesture)   — Phase 3
  /hydrone/vision/qr_codes              (hydrone_msgs/QRCode)         — Phase 4
  /hydrone/vision/debug_image           (sensor_msgs/Image)           — annotated debug view
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point

# Lazy imports — only used when the matching phase is active
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False

try:
    from pyzbar import pyzbar
    PYZBAR_AVAILABLE = True
except ImportError:
    PYZBAR_AVAILABLE = False

# Project messages (generated from hydrone_msgs)
from hydrone_msgs.msg import LandingBase, MissionState, HumanGesture, QRCode


# ── Constants ────────────────────────────────────────────────────────────────

# AprilTag / ArUco marker IDs used on the landing bases
# Each base carries a unique ArUco ID so we can tell them apart.
BASE_ARUCO_IDS = {0, 1, 2, 3, 4, 5}  # one per base

# Yellow cross colour range in HSV (landing bases have a yellow cross on top)
YELLOW_HSV_LOW  = np.array([20,  100, 100])
YELLOW_HSV_HIGH = np.array([35,  255, 255])

# Blue border range in HSV (landing bases have a blue border)
BLUE_HSV_LOW  = np.array([100, 120, 70])
BLUE_HSV_HIGH = np.array([130, 255, 255])

# Minimum contour area (px²) to consider a blob as a base candidate
MIN_BASE_AREA = 2000

# Gesture label → friendly name mapping (for terminal output required by rules)
GESTURE_LABELS = {
    "both_arms_up":    "COMMAND_LAND_HERE",
    "right_arm_point": "COMMAND_MOVE_RIGHT",
    "left_arm_point":  "COMMAND_MOVE_LEFT",
    "arms_forward":    "COMMAND_MOVE_FORWARD",
    "arms_back":       "COMMAND_MOVE_BACK",
    "arms_cross":      "COMMAND_STOP",
    "thumbs_up":       "COMMAND_TAKEOFF",
    "thumbs_down":     "COMMAND_LAND",
}


# ── Node ─────────────────────────────────────────────────────────────────────

class HydroneVisionNode(Node):

    def __init__(self):
        super().__init__("hydrone_vision")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("camera_topic",
                               "/zed2/zed_node/rgb/image_rect_color")
        self.declare_parameter("depth_topic",
                               "/zed2/zed_node/depth/depth_registered")
        self.declare_parameter("debug_image", True)
        self.declare_parameter("aruco_dict",  cv2.aruco.DICT_4X4_50)

        cam_topic   = self.get_parameter("camera_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        self.debug  = self.get_parameter("debug_image").value
        aruco_id    = self.get_parameter("aruco_dict").value

        # ── Internal state ──────────────────────────────────────────────────
        self.bridge      = CvBridge()
        self.phase       = 1           # updated by mission state
        self.last_depth  = None        # latest depth frame (numpy float32)

        # Landing-base tracking: base_id → LandingBase msg
        self.detected_bases: dict[int, LandingBase] = {}

        # Phase-3 gesture detector (MediaPipe Holistic / Pose)
        self.mp_pose     = None
        self.mp_drawing  = None
        self.pose_model  = None
        self._init_mediapipe()

        # ArUco detector
        aruco_dict    = cv2.aruco.getPredefinedDictionary(aruco_id)
        aruco_params  = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ─────────────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image, cam_topic, self._cb_image, sensor_qos)

        self.sub_depth = self.create_subscription(
            Image, depth_topic, self._cb_depth, sensor_qos)

        self.sub_mission = self.create_subscription(
            MissionState, "/hydrone/mission_state",
            self._cb_mission, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_bases   = self.create_publisher(
            LandingBase, "/hydrone/vision/landing_bases", 10)
        self.pub_gesture = self.create_publisher(
            HumanGesture, "/hydrone/vision/human_gesture", 10)
        self.pub_qr      = self.create_publisher(
            QRCode, "/hydrone/vision/qr_codes", 10)

        if self.debug:
            self.pub_debug = self.create_publisher(
                Image, "/hydrone/vision/debug_image", sensor_qos)

        self.get_logger().info("HydroneVisionNode ready.")

    # ────────────────────────────────────────────────────────────────────────
    # Init helpers
    # ────────────────────────────────────────────────────────────────────────

    def _init_mediapipe(self):
        """Initialise MediaPipe Pose only if available (Phase 3)."""
        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().warn(
                "MediaPipe not found — gesture detection disabled. "
                "Install with: pip install mediapipe")
            return
        self.mp_pose    = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose_model = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
        self.get_logger().info("MediaPipe Pose initialised.")

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _cb_mission(self, msg: MissionState):
        """Track the active competition phase."""
        self.phase = msg.phase

    def _cb_depth(self, msg: Image):
        """Cache the latest depth frame."""
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def _cb_image(self, msg: Image):
        """Main image callback — dispatches to the active phase pipeline."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        debug = frame.copy() if self.debug else None
        header = msg.header

        if self.phase in (1, 2):
            self._detect_landing_bases(frame, debug, header)
        elif self.phase == 3:
            self._detect_gesture(frame, debug, header)
        elif self.phase == 4:
            self._detect_qr_codes(frame, debug, header)

        if self.debug and debug is not None:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    # ────────────────────────────────────────────────────────────────────────
    # Phase 1 & 2 — Landing base detection
    # ────────────────────────────────────────────────────────────────────────

    def _detect_landing_bases(self, frame: np.ndarray,
                               debug: np.ndarray | None,
                               header: Header):
        """
        Two-stage detection:
          1. ArUco markers  → precise ID and 6-DOF pose
          2. Colour fallback (yellow+blue) → when markers are hidden (Phase 3 uses this)

        Publishes one LandingBase message per detected base.
        """
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Stage 1 — ArUco
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        detected_ids: set[int] = set()

        if ids is not None:
            for i, marker_corners in enumerate(corners):
                marker_id = int(ids[i][0])
                if marker_id not in BASE_ARUCO_IDS:
                    continue
                detected_ids.add(marker_id)

                # Compute centre pixel
                cx = int(np.mean(marker_corners[0, :, 0]))
                cy = int(np.mean(marker_corners[0, :, 1]))

                # Depth lookup
                height_m = self._get_depth_at(cy, cx)

                base_msg = self._build_base_msg(header, marker_id, cx, cy,
                                                 height_m, w, h)
                self.detected_bases[marker_id] = base_msg
                self.pub_bases.publish(base_msg)

                if debug is not None:
                    cv2.aruco.drawDetectedMarkers(debug, [marker_corners])
                    cv2.putText(debug, f"Base {marker_id}",
                                (cx - 20, cy - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Stage 2 — Colour fallback for bases without visible ArUco
        self._colour_detect_bases(frame, debug, header, already_found=detected_ids)

    def _colour_detect_bases(self, frame: np.ndarray,
                              debug: np.ndarray | None,
                              header: Header,
                              already_found: set[int]):
        """
        Detect bases using yellow cross + blue border colour segmentation.
        Used as fallback when ArUco markers are occluded / face-down (Phase 3).
        IDs are assigned sequentially to unmatched blobs.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        # Yellow cross mask
        mask_y = cv2.inRange(hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH)
        # Blue border mask
        mask_b = cv2.inRange(hsv, BLUE_HSV_LOW, BLUE_HSV_HIGH)

        # Combine: look for regions that have both yellow and adjacent blue
        kernel = np.ones((15, 15), np.uint8)
        dilated_y = cv2.dilate(mask_y, kernel)
        combined  = cv2.bitwise_and(dilated_y, mask_b)
        combined  = cv2.dilate(combined, kernel)

        contours, _ = cv2.findContours(
            combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fake_id = 10  # start ID for colour-only detections (won't clash with ArUco 0-5)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BASE_AREA:
                continue

            M  = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            height_m = self._get_depth_at(cy, cx)
            base_msg = self._build_base_msg(
                header, fake_id, cx, cy, height_m, w, h)
            self.pub_bases.publish(base_msg)

            if debug is not None:
                cv2.drawContours(debug, [cnt], -1, (255, 0, 255), 2)
                cv2.putText(debug, f"Base? ({fake_id})",
                            (cx - 30, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            fake_id += 1

    def _build_base_msg(self, header: Header, base_id: int,
                         cx: int, cy: int, height_m: float,
                         img_w: int, img_h: int) -> LandingBase:
        """Build a LandingBase message from pixel coordinates and depth."""
        msg            = LandingBase()
        msg.header     = header
        msg.base_id    = base_id
        msg.height     = height_m if not np.isnan(height_m) else 0.0
        msg.is_suspended = msg.height > 0.3   # above 30 cm → suspended
        msg.is_visited = self.detected_bases.get(base_id, LandingBase()).is_visited
        msg.confidence = 0.9

        # Simple normalised position estimate (replace with proper projection)
        msg.pose.position.x = float(cx) / img_w
        msg.pose.position.y = float(cy) / img_h
        msg.pose.position.z = msg.height
        msg.pose.orientation.w = 1.0
        return msg

    # ────────────────────────────────────────────────────────────────────────
    # Phase 3 — Human gesture recognition
    # ────────────────────────────────────────────────────────────────────────

    def _detect_gesture(self, frame: np.ndarray,
                         debug: np.ndarray | None,
                         header: Header):
        """
        Detect human skeleton and classify the operator's gesture.
        Publishes to /hydrone/vision/human_gesture and prints to terminal
        as required by the competition rules.
        """
        if self.pose_model is None:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose_model.process(rgb)

        if not results.pose_landmarks:
            return

        landmarks = results.pose_landmarks.landmark
        gesture   = self._classify_gesture(landmarks)

        if debug is not None and self.mp_drawing is not None:
            self.mp_drawing.draw_landmarks(
                debug, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        # Build flat keypoints array
        kps = []
        for lm in landmarks:
            kps.extend([lm.x, lm.y, lm.z])

        # Human world position (hips midpoint as proxy)
        lhip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value]
        rhip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value]
        human_pos = Point(
            x=(lhip.x + rhip.x) / 2.0,
            y=(lhip.y + rhip.y) / 2.0,
            z=(lhip.z + rhip.z) / 2.0,
        )

        msg                    = HumanGesture()
        msg.header             = header
        msg.gesture_name       = gesture
        msg.confidence         = 0.85
        msg.human_position     = human_pos
        msg.skeleton_keypoints = kps

        self.pub_gesture.publish(msg)

        # ── Print to terminal (required by Phase 3 rules) ──
        friendly = GESTURE_LABELS.get(gesture, gesture)
        self.get_logger().info(
            f"[Phase3] Gesture detected: {gesture}  →  {friendly}  "
            f"| Human @ ({human_pos.x:.2f}, {human_pos.y:.2f})")

        if debug is not None:
            cv2.putText(debug, f"Gesture: {friendly}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

    def _classify_gesture(self, landmarks) -> str:
        """
        Rule-based gesture classifier from MediaPipe pose landmarks.
        Returns a gesture key from GESTURE_LABELS.

        Landmarks used (MediaPipe indices):
          11 = LEFT_SHOULDER   12 = RIGHT_SHOULDER
          13 = LEFT_ELBOW      14 = RIGHT_ELBOW
          15 = LEFT_WRIST      16 = RIGHT_WRIST
          23 = LEFT_HIP        24 = RIGHT_HIP
        """
        PL = self.mp_pose.PoseLandmark

        lw = landmarks[PL.LEFT_WRIST.value]
        rw = landmarks[PL.RIGHT_WRIST.value]
        ls = landmarks[PL.LEFT_SHOULDER.value]
        rs = landmarks[PL.RIGHT_SHOULDER.value]
        lh = landmarks[PL.LEFT_HIP.value]
        rh = landmarks[PL.RIGHT_HIP.value]
        le = landmarks[PL.LEFT_ELBOW.value]
        re = landmarks[PL.RIGHT_ELBOW.value]

        # In MediaPipe image-space, Y increases downward
        # "above shoulder" means smaller Y value
        left_up  = lw.y < ls.y - 0.05
        right_up = rw.y < rs.y - 0.05
        left_down  = lw.y > lh.y + 0.05
        right_down = rw.y > rh.y + 0.05

        if left_up and right_up:
            return "both_arms_up"

        # Pointing: one wrist significantly to the side of its shoulder
        if right_up and (rw.x > rs.x + 0.15):
            return "right_arm_point"
        if left_up and (lw.x < ls.x - 0.15):
            return "left_arm_point"

        # Both arms extended forward (z < shoulder z in 3D)
        if abs(lw.z) > abs(ls.z) + 0.1 and abs(rw.z) > abs(rs.z) + 0.1:
            return "arms_forward"

        # Both arms crossed (wrists cross the body midline)
        mid_x = (ls.x + rs.x) / 2.0
        if lw.x > mid_x and rw.x < mid_x:
            return "arms_cross"

        # Both arms down → thumbs down proxy
        if left_down and right_down:
            return "thumbs_down"

        return "unknown"

    # ────────────────────────────────────────────────────────────────────────
    # Phase 4 — QR Code detection
    # ────────────────────────────────────────────────────────────────────────

    def _detect_qr_codes(self, frame: np.ndarray,
                          debug: np.ndarray | None,
                          header: Header):
        """
        Scan the frame for the 5 competition QR codes (A-E).
        Publishes a QRCode message for every new detection.
        Prints to terminal as required by Phase 4 rules.
        """
        if not PYZBAR_AVAILABLE:
            # Fall back to OpenCV QRCodeDetector
            self._detect_qr_opencv(frame, debug, header)
            return

        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded = pyzbar.decode(gray)

        for obj in decoded:
            data = obj.data.decode("utf-8").strip().upper()
            if data not in ("A", "B", "C", "D", "E"):
                continue

            pts = np.array(obj.polygon, np.int32).reshape((-1, 1, 2))
            cx  = int(np.mean([p[0][0] for p in obj.polygon]))
            cy  = int(np.mean([p[0][1] for p in obj.polygon]))

            depth_m = self._get_depth_at(cy, cx)

            msg           = QRCode()
            msg.header    = header
            msg.qr_id     = data
            msg.pose.position.z = depth_m if not np.isnan(depth_m) else 0.0
            msg.pose.orientation.w = 1.0
            # is_new is set by the mission node which tracks unique detections
            msg.is_new    = True

            self.pub_qr.publish(msg)

            # ── Terminal output (required by Phase 4 rules) ──
            self.get_logger().info(
                f"[Phase4] QR Code detected: '{data}'  depth={depth_m:.2f}m")

            if debug is not None:
                cv2.polylines(debug, [pts], True, (0, 255, 0), 2)
                cv2.putText(debug, f"QR:{data}",
                            (cx - 15, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    def _detect_qr_opencv(self, frame: np.ndarray,
                           debug: np.ndarray | None,
                           header: Header):
        """OpenCV-based QR fallback when pyzbar is not installed."""
        qr_detector = cv2.QRCodeDetector()
        data, points, _ = qr_detector.detectAndDecode(frame)
        if not data:
            return

        data = data.strip().upper()
        if data not in ("A", "B", "C", "D", "E"):
            return

        msg           = QRCode()
        msg.header    = header
        msg.qr_id     = data
        msg.pose.orientation.w = 1.0
        msg.is_new    = True
        self.pub_qr.publish(msg)

        self.get_logger().info(f"[Phase4-CV] QR Code: '{data}'")

        if debug is not None and points is not None:
            pts = points.astype(np.int32)
            cv2.polylines(debug, [pts], True, (0, 200, 255), 2)

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _get_depth_at(self, row: int, col: int,
                       window: int = 5) -> float:
        """
        Return the median depth (metres) in a small window around (row, col).
        Returns NaN if depth data is unavailable.
        """
        if self.last_depth is None:
            return float("nan")
        h, w = self.last_depth.shape
        r0 = max(0, row - window)
        r1 = min(h, row + window)
        c0 = max(0, col - window)
        c1 = min(w, col + window)
        patch = self.last_depth[r0:r1, c0:c1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        return float(np.median(valid)) if len(valid) > 0 else float("nan")


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = HydroneVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
