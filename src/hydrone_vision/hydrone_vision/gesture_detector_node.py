#!/usr/bin/env python3
"""
gesture_detector_node — Phase 3: camera -> skeleton -> gesture, every frame.

  in   a V4L2 camera opened here (source:=device, the real drone: no image ever
       crosses DDS on the Raspberry Pi), or an Image topic (source:=topic, the sim)
  out  /hydrone/vision/human_gesture   (hydrone_msgs/HumanGesture), one per
       processed frame, person or not — the mission reads a missing message
       as a dead camera, and "person_found: false" as an empty frame
  dbg  /hydrone/gesture/debug_image    (bgr8, debug_hz), arms and verdict drawn

The gesture is the RAW per-frame verdict of hydrone_vision.gestures.classify.
Holding it in time (debounce), and what the drone does with it, is the
mission's business: phase3_gesture_node.

Pose model: MediaPipe Pose (single person, CPU, in the Docker image), its 33
landmarks mapped to COCO-17. model_complexity 0 (lite) is the fast one, 1 is
~2x slower and steadier on the arms. Neither rate is measured on the Pi 5 yet:
the node logs frames/s every 10 s.
"""

import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from hydrone_msgs.msg import HumanGesture

from hydrone_vision.gestures import (L_EL, L_SH, L_WR, R_EL, R_SH, R_WR, classify,
                                     mediapipe_to_coco, person_center)
from hydrone_vision.image_convert import bgr_image_to_numpy, numpy_to_image


class MediaPipePose:
    """bgr image -> (kpts (17,2) px, conf (17,)) of the one person, or None."""

    def __init__(self, complexity=0, min_detection=0.5, min_tracking=0.5):
        import mediapipe as mp
        solutions = getattr(mp, "solutions", None)
        if solutions is None or not hasattr(solutions, "pose"):
            raise RuntimeError(
                f"mediapipe {getattr(mp, '__version__', '?')} has no mp.solutions.pose "
                "(dropped in recent releases): pip install 'mediapipe<0.10.22'")
        self.pose = solutions.pose.Pose(
            static_image_mode=False, model_complexity=int(complexity),
            smooth_landmarks=True, enable_segmentation=False,
            min_detection_confidence=float(min_detection),
            min_tracking_confidence=float(min_tracking))

    def infer(self, bgr):
        h, w = bgr.shape[:2]
        res = self.pose.process(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        if not res.pose_landmarks:
            return None
        return mediapipe_to_coco(res.pose_landmarks.landmark, w, h)


class LatestFrame(threading.Thread):
    """Reads the camera as fast as it delivers and keeps only the newest frame.

    Without this, a model slower than the camera works through the driver's
    queue and the drone reacts to where the arms WERE a second ago.
    """

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap, self.lock = cap, threading.Lock()
        self.frame, self.seq, self.running = None, 0, True

    def run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame, self.seq = frame, self.seq + 1

    def latest(self):
        with self.lock:
            return self.frame, self.seq


ROTATE = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
          270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class GestureDetectorNode(Node):

    def __init__(self):
        super().__init__("gesture_detector")
        p = self.declare_parameters("", [
            ("source", "device"),           # 'device' | 'topic'
            ("device", "/dev/video0"),
            ("width", 640),
            ("height", 480),
            ("fps", 30),
            ("fourcc", "MJPG"),
            ("image_topic", "/front_cam/image_raw"),
            ("rotate_deg", 0),              # camera mounted sideways/upside down
            ("mirror", False),              # undo a camera that mirrors (selfie mode):
                                            # it swaps DIREITA and ESQUERDA
            ("model_complexity", 0),
            ("min_detection_conf", 0.5),
            ("process_hz", 30.0),           # upper bound; the model sets the real rate
            ("debug_image", True),
            ("debug_hz", 3.0),
        ])
        self.par = {q.name: q.value for q in p}
        if int(self.par["rotate_deg"]) not in ROTATE:
            raise ValueError("rotate_deg must be 0, 90, 180 or 270")

        self.model = MediaPipePose(self.par["model_complexity"], self.par["min_detection_conf"])

        self.grabber, self.cap = None, None
        self._topic_frame, self._topic_seq = None, 0
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        if self.par["source"] == "device":
            self._open_device()
        elif self.par["source"] == "topic":
            self.create_subscription(Image, self.par["image_topic"], self._image_cb, sensor_qos)
        else:
            raise ValueError("source must be 'device' or 'topic'")

        self.pub = self.create_publisher(HumanGesture, "/hydrone/vision/human_gesture", 10)
        self.pub_debug = (self.create_publisher(Image, "/hydrone/gesture/debug_image", sensor_qos)
                          if self.par["debug_image"] else None)

        self._last_seq = 0
        self._last_raw = None
        self._t_debug = 0.0
        self._n, self._t_rate = 0, time.monotonic()
        self.create_timer(1.0 / max(float(self.par["process_hz"]), 1.0), self._tick)
        self.get_logger().info(
            f"gesture_detector up: MediaPipe Pose on "
            f"{self.par['device'] if self.par['source'] == 'device' else self.par['image_topic']}")

    # ── frames in ───────────────────────────────────────────────────────────
    def _open_device(self):
        dev = str(self.par["device"])
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"could not open {dev}")
        if self.par["fourcc"]:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(self.par["fourcc"])))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.par["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.par["height"]))
        cap.set(cv2.CAP_PROP_FPS, int(self.par["fps"]))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        self.grabber = LatestFrame(cap)
        self.grabber.start()
        self.get_logger().info(
            f"camera {dev}: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps")

    def _image_cb(self, msg):
        try:
            self._topic_frame = bgr_image_to_numpy(msg)
            self._topic_seq += 1
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)

    def _latest(self):
        if self.grabber is not None:
            return self.grabber.latest()
        return self._topic_frame, self._topic_seq

    # ── per frame ───────────────────────────────────────────────────────────
    def _tick(self):
        frame, seq = self._latest()
        if frame is None or seq == self._last_seq:
            return
        self._last_seq = seq
        rot = ROTATE[int(self.par["rotate_deg"])]
        if rot is not None:
            frame = cv2.rotate(frame, rot)
        if self.par["mirror"]:
            frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        person = self.model.infer(frame)
        msg = HumanGesture()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "front_cam_optical_frame"
        gesture, states, angles = "NENHUM", ("-", "-"), (None, None)
        if person is not None:
            kpts, conf = person
            gesture, states, angles = classify(kpts, conf)
            c = person_center(kpts, conf, w, h)
            msg.person_found = c is not None
            if c is not None:
                msg.image_x, msg.image_size = float(c[0]), float(c[1])
            used = (L_SH, R_SH, L_EL, R_EL, L_WR, R_WR)
            msg.confidence = float(min(conf[i] for i in used))
            msg.skeleton_keypoints = [float(v) for k, c_ in zip(kpts, conf)
                                      for v in (k[0], k[1], c_)]
        msg.gesture_name = gesture
        self.pub.publish(msg)

        if gesture != self._last_raw:
            self.get_logger().debug(f"raw gesture {gesture} {states}")
            self._last_raw = gesture
        self._rate()
        self._debug(frame, person, gesture, states, angles)

    def _rate(self):
        self._n += 1
        dt = time.monotonic() - self._t_rate
        if dt >= 10.0:
            self.get_logger().info(f"processing {self._n / dt:.1f} frames/s")
            self._n, self._t_rate = 0, time.monotonic()

    def _debug(self, frame, person, gesture, states, angles):
        if self.pub_debug is None:
            return
        now = time.monotonic()
        if now - self._t_debug < 1.0 / max(float(self.par["debug_hz"]), 0.1):
            return
        self._t_debug = now
        img = frame.copy()
        if person is not None:
            kpts, conf = person
            for a, b in ((L_SH, R_SH), (L_SH, L_EL), (L_EL, L_WR), (R_SH, R_EL), (R_EL, R_WR)):
                if conf[a] > 0.3 and conf[b] > 0.3:
                    cv2.line(img, tuple(int(v) for v in kpts[a]), tuple(int(v) for v in kpts[b]),
                             (0, 255, 0), 3)
        ang = " ".join("--" if a is None else f"{a:.0f}" for a in angles)
        cv2.putText(img, f"{gesture}  L:{states[0]} R:{states[1]}  {ang}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        out = numpy_to_image(np.ascontiguousarray(img))
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "front_cam_optical_frame"
        self.pub_debug.publish(out)

    def destroy_node(self):
        if self.grabber is not None:
            self.grabber.running = False
            self.grabber.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GestureDetectorNode()
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
