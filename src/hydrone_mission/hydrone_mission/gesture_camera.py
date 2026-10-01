#!/usr/bin/env python3
"""
gesture_camera — the operator's gestures from a camera, through frl_core.

    ros2 run hydrone_mission gesture_camera                    # webcam /dev/video0
    ros2 run hydrone_mission gesture_camera --ros-args -p camera:=/dev/video2
    ros2 run hydrone_mission gesture_camera --ros-args -p camera:=gestos.mp4

  frame -> MediaPipe Pose (33 landmarks) -> COCO-17 keypoints
        -> frl_core.classify -> frl_core.Debouncer
        -> hydrone_msgs/HumanGesture on /hydrone/vision/human_gesture, every frame

The same topic and message gesture_terminal publishes, so phase3_hri_node
cannot tell the camera from the keyboard. gesture_name is the DEBOUNCED
frl_core gesture (HOVER, APROXIMAR, POUSAR, ...); the mission treats the
movement ones as hold-to-move. Nothing is published while nobody is in the
frame, so the mission brakes (vel_timeout_s).

Printed for the judge: every change of gesture, with both arm states and
angles. The preview window draws the skeleton; `q` closes it.

The classifier itself is frl_core.classify, untouched: this node only feeds
it keypoints. MediaPipe's landmarks are mapped to COCO-17 by MP_TO_COCO, with
`visibility` as the per-keypoint confidence.
"""

import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from hydrone_msgs.msg import HumanGesture

from hydrone_mission.phase3 import frl_core

# COCO-17 index -> MediaPipe Pose landmark index
MP_TO_COCO = (0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)
# COCO-17 limbs, for the preview
LIMBS = ((5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12),
         (11, 13), (13, 15), (12, 14), (14, 16))


def mediapipe_to_coco(landmarks, width, height):
    """MediaPipe's normalised landmarks -> ((17,2) pixels, (17,) confidence)."""
    kpts = np.zeros((17, 2), dtype=float)
    conf = np.zeros(17, dtype=float)
    for c, m in enumerate(MP_TO_COCO):
        lm = landmarks[m]
        kpts[c] = (lm.x * width, lm.y * height)
        conf[c] = lm.visibility
    return kpts, conf


class GestureCamera(Node):

    def __init__(self):
        super().__init__("gesture_camera")
        # index, /dev/videoN, or a video file; camera:=0 and camera:=/dev/video0 both work
        self.declare_parameter("camera", "0", ParameterDescriptor(dynamic_typing=True))
        p = self.declare_parameters("", [
            ("width", 640),
            ("height", 480),
            ("flip", False),              # mirror the image before classifying
            ("show", True),               # preview window (needs a display)
            ("model_complexity", 1),      # MediaPipe: 0 fast, 1 default, 2 accurate
            ("gesture_topic", "/hydrone/vision/human_gesture"),
        ])
        self.p = {q.name: q.value for q in p}
        self.p["camera"] = str(self.get_parameter("camera").value)
        self.pub = self.create_publisher(HumanGesture, self.p["gesture_topic"], 10)
        self.debouncer = frl_core.Debouncer()
        self._last = None
        self._stop = threading.Event()

    def run(self):
        import cv2
        import mediapipe as mp

        src = self.p["camera"]
        cap = cv2.VideoCapture(int(src) if str(src).isdigit() else src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.p["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.p["height"])
        if not cap.isOpened():
            self.get_logger().error(
                f"cannot open camera '{src}'. Is it plugged in, and was the container "
                "started with --webcam (docker_up.sh)? `ls /dev/video*` lists them.")
            return
        self.get_logger().info(f"camera '{src}' open — publishing on {self.p['gesture_topic']}")

        pose = mp.solutions.pose.Pose(model_complexity=self.p["model_complexity"],
                                      min_detection_confidence=0.5,
                                      min_tracking_confidence=0.5)
        try:
            while rclpy.ok() and not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn("camera returned no frame — stopping")
                    break
                if self.p["flip"]:
                    frame = cv2.flip(frame, 1)
                h, w = frame.shape[:2]
                res = pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                gesture, detail = None, None
                if res.pose_landmarks:
                    kpts, conf = mediapipe_to_coco(res.pose_landmarks.landmark, w, h)
                    raw, states, angles = frl_core.classify(kpts, conf)
                    gesture, _changed = self.debouncer.update(raw, time.monotonic())
                    detail = (raw, states, angles)
                    self._publish(gesture, kpts, conf)
                    self._report(gesture, detail)
                if self.p["show"]:
                    self._draw(cv2, frame, res, gesture, detail)
                    cv2.imshow("Fase 3 - gestos", frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        finally:
            pose.close()
            cap.release()
            if self.p["show"]:
                cv2.destroyAllWindows()

    def _publish(self, gesture, kpts, conf):
        msg = HumanGesture()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.gesture_name = gesture
        arm = [conf[i] for i in (frl_core.L_SH, frl_core.R_SH, frl_core.L_EL,
                                 frl_core.R_EL, frl_core.L_WR, frl_core.R_WR)]
        msg.confidence = float(min(arm))
        # the operator's hips, in image pixels (no depth from a webcam)
        msg.human_position.x = float((kpts[11][0] + kpts[12][0]) / 2)
        msg.human_position.y = float((kpts[11][1] + kpts[12][1]) / 2)
        msg.skeleton_keypoints = [float(v) for k, c in zip(kpts, conf) for v in (k[0], k[1], c)]
        self.pub.publish(msg)

    def _report(self, gesture, detail):
        if gesture == self._last:
            return
        self._last = gesture
        raw, (sl, sr), (al, ar) = detail
        ang = lambda a: "  -" if a is None else f"{a:3.0f}"
        self.get_logger().info(
            f"[FASE 3] GESTO: {gesture:<9}  braço E {sl:<9} {ang(al)}°  "
            f"braço D {sr:<9} {ang(ar)}°")

    @staticmethod
    def _draw(cv2, frame, res, gesture, detail):
        if res.pose_landmarks:
            h, w = frame.shape[:2]
            kpts, conf = mediapipe_to_coco(res.pose_landmarks.landmark, w, h)
            for a, b in LIMBS:
                if min(conf[a], conf[b]) >= frl_core.MIN_CONF:
                    cv2.line(frame, tuple(int(v) for v in kpts[a]),
                             tuple(int(v) for v in kpts[b]), (0, 255, 0), 2)
            for (x, y), c in zip(kpts, conf):
                if c >= frl_core.MIN_CONF:
                    cv2.circle(frame, (int(x), int(y)), 4, (0, 200, 255), -1)
        text = gesture or "sem operador"
        cv2.putText(frame, text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
        if detail is not None:
            raw, (sl, sr), _ = detail
            cv2.putText(frame, f"bruto {raw}  E {sl}  D {sr}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    rclpy.init()
    node = GestureCamera()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run()                       # OpenCV's window wants the main thread
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop.set()
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
