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

import http.server
import socketserver
import os
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
            ("vest", False),              # operador = quem veste o colete (cor em vest_file)
            ("vest_file", "/ws/maps/vest.json"),  # calibrar: touch maps/vest_calibrate (host)
            ("vest_min_frac", 0.20),
            ("view_port", 0),             # >0: MJPEG with the skeleton on http://<jetson>:<port>/ (bench only)
            ("gesture_topic", "/hydrone/vision/human_gesture"),
        ])
        self.p = {q.name: q.value for q in p}
        self.p["camera"] = str(self.get_parameter("camera").value)
        self.pub = self.create_publisher(HumanGesture, self.p["gesture_topic"], 10)
        self.debouncer = frl_core.Debouncer()
        self._last = None
        self._stop = threading.Event()
        self._jpeg = None
        self.vest = {"h_lo": 25, "h_hi": 50, "s_lo": 90, "v_lo": 90}  # verde-limão
        try:
            import json
            self.vest.update(json.load(open(self.p["vest_file"])))
        except (OSError, ValueError):
            pass
        self._calib_until, self._calib_px = 0.0, []
        self._vest_frac = None
        if self.p["vest"]:
            self.get_logger().info(f"[FASE 3] colete LIGADO {self.vest}")
        self._jpeg_lock = threading.Lock()
        if self.p["view_port"] > 0:
            self._start_view(self.p["view_port"])

    def run(self):
        if str(self.p["camera"]).startswith("udp:"):
            return self._run_udp(int(str(self.p["camera"])[4:]))
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

        if self.p["model_complexity"] == 0:
            import shutil
            dst = os.path.join(os.path.dirname(mp.__file__), "modules", "pose_landmark", "pose_landmark_lite.tflite")
            src_m = "/ws/src/hydrone_mission/models/pose_landmark_lite.tflite"
            if not os.path.exists(dst) and os.path.exists(src_m):
                shutil.copy(src_m, dst)
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
                    self._vest_step(cv2, frame, kpts, conf)
                    if self.p["vest"] and self._vest_frac < self.p["vest_min_frac"]:
                        pass   # sem colete: não é o operador, nada publicado (o drone freia)
                    else:
                        raw, states, angles = frl_core.classify(kpts, conf)
                        gesture, _changed = self.debouncer.update(raw, time.monotonic())
                        detail = (raw, states, angles)
                        self._publish(gesture, kpts, conf)
                        self._report(gesture, detail)
                if self.p["view_port"] > 0:
                    try:
                        view = frame.copy()
                        self._draw(cv2, view, res, gesture, detail)
                        ok_j, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 60])
                        if ok_j:
                            with self._jpeg_lock:
                                self._jpeg = buf.tobytes()
                    except Exception as e:  # the view must never stop the gestures
                        self.get_logger().warn(f"view: {e}", throttle_duration_sec=10.0)
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

    def _run_udp(self, port):
        """Keypoints from pose_gpu.py (GPU) by UDP; same classify/debounce/publish."""
        import json
        import socket
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", port))
        rx.settimeout(0.5)
        back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"keypoints da GPU (pose_gpu.py) em udp:{port} — publishing on {self.p['gesture_topic']}")
        while rclpy.ok() and not self._stop.is_set():
            try:
                data = rx.recv(65536)
            except socket.timeout:
                continue
            try:
                m = json.loads(data)
                kpts = np.array([[k[0], k[1]] for k in m["kp"]], dtype=float)
                conf = np.array([k[2] for k in m["kp"]], dtype=float)
                if self.p["flip"]:
                    kpts[:, 0] = m["w"] - kpts[:, 0]
                    kpts[[1, 3, 5, 7, 9, 11, 13, 15]], kpts[[2, 4, 6, 8, 10, 12, 14, 16]] = \
                        kpts[[2, 4, 6, 8, 10, 12, 14, 16]].copy(), kpts[[1, 3, 5, 7, 9, 11, 13, 15]].copy()
                    conf[[1, 3, 5, 7, 9, 11, 13, 15]], conf[[2, 4, 6, 8, 10, 12, 14, 16]] = \
                        conf[[2, 4, 6, 8, 10, 12, 14, 16]].copy(), conf[[1, 3, 5, 7, 9, 11, 13, 15]].copy()
            except (ValueError, KeyError, IndexError):
                continue
            raw, states, angles = frl_core.classify(kpts, conf)
            gesture, _changed = self.debouncer.update(raw, time.monotonic())
            self._publish(gesture, kpts, conf)
            self._report(gesture, (raw, states, angles))
            try:
                back.sendto(f"{gesture}  (bruto {raw})".encode(), ("127.0.0.1", port + 1))
            except OSError:
                pass

    def _torso_hsv(self, cv2, frame, kpts, conf):
        if min(conf[5], conf[6]) < frl_core.MIN_CONF:
            return None
        xs = [kpts[5][0], kpts[6][0]]
        y0 = min(kpts[5][1], kpts[6][1])
        hips = [kpts[i][1] for i in (11, 12) if conf[i] >= frl_core.MIN_CONF]
        w = abs(xs[1] - xs[0])
        y1 = max(hips) if hips else y0 + 1.3 * w
        x0, x1 = min(xs) + 0.15 * w, max(xs) - 0.15 * w
        y0, y1 = y0 + 0.10 * (y1 - y0), y1 - 0.10 * (y1 - y0)
        H, W_ = frame.shape[:2]
        x0, x1, y0, y1 = int(max(0, x0)), int(min(W_, x1)), int(max(0, y0)), int(min(H, y1))
        if x1 - x0 < 4 or y1 - y0 < 4:
            return None
        return cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)

    def _vest_step(self, cv2, frame, kpts, conf):
        """Fração do tronco com a cor do colete; calibração por arquivo-gatilho."""
        import json
        trig = os.path.join(os.path.dirname(self.p["vest_file"]), "vest_calibrate")
        if os.path.exists(trig):
            os.remove(trig)
            self._calib_until, self._calib_px = time.monotonic() + 3.0, []
            self.get_logger().info("[FASE 3] calibrando o colete por 3 s: operador SOZINHO na imagem")
        hsv = self._torso_hsv(cv2, frame, kpts, conf)
        if self._calib_until:
            if hsv is not None:
                self._calib_px.append(hsv.reshape(-1, 3))
            if time.monotonic() > self._calib_until:
                self._calib_until = 0.0
                px = np.concatenate(self._calib_px) if self._calib_px else np.zeros((0, 3))
                sat = px[(px[:, 1] > 60) & (px[:, 2] > 50)]
                if len(sat) < 50:
                    self.get_logger().error("[FASE 3] calibração do colete FALHOU: tronco sem cor forte")
                else:
                    h = int(np.median(sat[:, 0]))
                    self.vest.update({"h_lo": max(0, h - 10), "h_hi": min(179, h + 10),
                                      "s_lo": max(60, int(np.percentile(sat[:, 1], 20) * 0.7)),
                                      "v_lo": max(50, int(np.percentile(sat[:, 2], 20) * 0.6))})
                    try:
                        json.dump(self.vest, open(self.p["vest_file"], "w"))
                    except OSError as e:
                        self.get_logger().warn(f"vest_file: {e}")
                    self.get_logger().info(f"[FASE 3] colete calibrado: {self.vest}")
        if hsv is None:
            self._vest_frac = 0.0
        else:
            v = self.vest
            m = cv2.inRange(hsv, (v["h_lo"], v["s_lo"], v["v_lo"]), (v["h_hi"], 255, 255))
            self._vest_frac = float(np.count_nonzero(m)) / m.size

    def _start_view(self, port):
        node = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while not node._stop.is_set():
                        with node._jpeg_lock:
                            jpg = node._jpeg
                        if jpg is not None:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                        time.sleep(0.15)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        srv = Server(("0.0.0.0", port), Handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(f"[FASE 3] imagem com esqueleto em http://<jetson>:{port}/")

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
        text = gesture or "sem operador / sem colete"
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
