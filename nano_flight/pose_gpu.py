#!/usr/bin/env python3
"""pose_gpu.py - Fase 3: pose do operador na GPU da Nano (jetson-inference poseNet,
trt_pose ResNet18-body em TensorRT FP16), fora do Docker.

  camera -> poseNet -> 17 keypoints COCO (x, y, conf) -> UDP 127.0.0.1:5600 (JSON)

O gesture_camera do container (camera:=udp:5600) recebe os keypoints e roda o
mesmo frl_core.classify de sempre; ele devolve o gesto por UDP 5601 só para o
desenho da imagem. VIEW_PORT>0 serve a imagem com esqueleto + gesto em MJPEG.

  python3 pose_gpu.py [camera] [view_port]
"""
import glob
import os
import http.server
import json
import socket
import socketserver
import sys
import threading
import time

import numpy as np
import cv2  # ANTES do jetson.inference (senão: libgomp 'static TLS block')
try:
    import jetson_inference as ji
    import jetson_utils as ju
except ImportError:
    import jetson.inference as ji
    import jetson.utils as ju

CAM = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else (sorted(glob.glob("/dev/v4l/by-id/*video-index0")) or ["/dev/video0"])[0]
CAM = os.path.realpath(CAM)      # jetson-utils só aceita /dev/videoN, não o link by-id
VIEW_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 0
OUT = ("127.0.0.1", 5600)
BACK_PORT = 5601

# nomes do trt_pose -> índice COCO-17 (o pescoço, 17, fica de fora)
COCO = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
        "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
        "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]
LIMBS = ((5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12),
         (11, 13), (13, 15), (12, 14), (14, 16))

net = ji.poseNet("resnet18-body", threshold=0.15)
name_to_coco = {}
for i in range(net.GetNumKeypoints()):
    n = net.GetKeypointName(i)
    if n in COCO:
        name_to_coco[i] = COCO.index(n)

cam = ju.videoSource(f"v4l2://{CAM}", argv=["--input-width=640", "--input-height=480",
                                              "--input-rate=30", "--input-codec=mjpeg"])
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

gesture = {"g": ""}
jpeg = {"b": None}
lock = threading.Lock()


def listen_back():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", BACK_PORT))
    while True:
        gesture["g"] = rx.recv(256).decode(errors="ignore")


def serve_view(port):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        b = jpeg["b"]
                    if b:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + b + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass

    class S(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    threading.Thread(target=S(("0.0.0.0", port), H).serve_forever, daemon=True).start()
    return cv2


threading.Thread(target=listen_back, daemon=True).start()
cv2 = serve_view(VIEW_PORT) if VIEW_PORT > 0 else None
print(f"[pose_gpu] camera {CAM} -> UDP {OUT[1]}" + (f", imagem em :{VIEW_PORT}" if VIEW_PORT else ""), flush=True)

n, t_rate, last_view = 0, time.time(), 0.0
while True:
    img = cam.Capture(timeout=1000)
    if img is None:
        if not cam.IsStreaming():
            print("[pose_gpu] camera parou", flush=True)
            break
        continue
    poses = net.Process(img, overlay="none")
    best, kp = None, None
    if poses:
        # o operador = a pose com mais keypoints dos braços
        best = max(poses, key=lambda p: len(p.Keypoints))
        kp = [[0.0, 0.0, 0.0] for _ in range(17)]
        for k in best.Keypoints:
            j = name_to_coco.get(k.ID)
            if j is not None:
                kp[j] = [float(k.x), float(k.y), 1.0]
        tx.sendto(json.dumps({"t": time.time(), "w": img.width, "h": img.height, "kp": kp}).encode(), OUT)
    n += 1
    now = time.time()
    if now - t_rate >= 10.0:
        print(f"[pose_gpu] {n / (now - t_rate):.1f} fps", flush=True)
        n, t_rate = 0, now
    if cv2 is not None and now - last_view > 0.15:
      try:
          last_view = now
          ju.cudaDeviceSynchronize()
          frame = cv2.cvtColor(np.array(img, copy=False), cv2.COLOR_RGB2BGR)
          if kp:
              for a, b in LIMBS:
                  if kp[a][2] and kp[b][2]:
                      cv2.line(frame, (int(kp[a][0]), int(kp[a][1])), (int(kp[b][0]), int(kp[b][1])), (0, 255, 0), 2)
              for x, y, c in kp:
                  if c:
                      cv2.circle(frame, (int(x), int(y)), 4, (0, 200, 255), -1)
          cv2.putText(frame, gesture["g"] if kp else "sem operador", (10, 35),
                      cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
          ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
          if ok:
              with lock:
                  jpeg["b"] = buf.tobytes()
      except Exception as e:  # a imagem nunca derruba a pose
        if not getattr(ju, '_view_err', False):
            print(f'[pose_gpu] imagem desligada: {e}', flush=True)
            ju._view_err = True
        cv2 = None
