#!/usr/bin/env python3
"""pose_gpu.py - Fase 3: pose do operador na GPU da Nano (jetson-inference poseNet,
trt_pose ResNet18-body em TensorRT FP16), fora do Docker.

  camera -> poseNet -> 17 keypoints COCO (x, y, conf) -> UDP 127.0.0.1:5600 (JSON)

O gesture_camera do container (camera:=udp:5600) recebe os keypoints e roda o
mesmo frl_core.classify de sempre; ele devolve o gesto por UDP 5601 só para o
desenho da imagem. VIEW_PORT>0 serve a imagem com esqueleto + gesto em MJPEG.

Operador = quem veste o COLETE (VEST=1, padrão): de todas as pessoas que a
rede acha, só vale a que tem a cor do colete no tronco (ombros->quadril). Sem
colete na imagem nada é publicado e o drone freia. VEST=0 desliga o filtro.
Calibrar na arena, com o operador SOZINHO na imagem e a stack no ar:
  touch /tmp/vest_calibrate      (aprende a cor em ~3 s e salva em vest.json)

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
HERE = os.path.dirname(os.path.abspath(__file__))
VEST_ON = os.environ.get("VEST", "1") != "0"
VEST_FILE = os.path.join(HERE, "vest.json")
VEST_TRIGGER = "/tmp/vest_calibrate"
VEST_MIN_FRAC = 0.20          # fração mínima do tronco com a cor do colete
# verde-limão em HSV do OpenCV (H 0-180); substituído pelo vest.json da calibração
vest = {"h_lo": 25, "h_hi": 50, "s_lo": 90, "v_lo": 90}
try:
    vest.update(json.load(open(VEST_FILE)))
except (OSError, ValueError):
    pass

# nomes do trt_pose -> índice COCO-17 (o pescoço, 17, fica de fora)
COCO = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
        "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
        "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]
LIMBS = ((5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12),
         (11, 13), (13, 15), (12, 14), (14, 16))

POSE_NET = os.environ.get("POSE_NET", "yolo")   # yolo (padrão) | resnet18-body | densenet121-body
POSE_THRESH = float(os.environ.get("POSE_THRESH", "0.15"))
YOLO_ONNX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolov8n-pose-raw-256.onnx")
YOLO_H, YOLO_W, YOLO_STRIDES = 256, 320, (8, 16, 32)
name_to_coco = {}
if POSE_NET == "yolo":
    # YOLOv8n-pose (COCO-17) pelo OpenCV DNN em CUDA FP16. O ONNX sai SEM a decodificação
    # final (o OpenCV 4.6 da Nano não roda ela na GPU): logit da pessoa + 17*(x,y,v)
    # crus por âncora; a decodificação é numpy, abaixo. Exportado do ultralytics 8.4.
    net = cv2.dnn.readNetFromONNX(YOLO_ONNX)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
    _a, _s = [], []
    for st in YOLO_STRIDES:
        ny, nx = YOLO_H // st, YOLO_W // st
        yy, xx = np.meshgrid(np.arange(ny), np.arange(nx), indexing="ij")
        _a.append(np.stack([xx.ravel() + 0.5, yy.ravel() + 0.5], 1))
        _s.append(np.full(ny * nx, st))
    Y_ANCH, Y_STR = np.concatenate(_a), np.concatenate(_s)
else:
    net = ji.poseNet(POSE_NET, threshold=POSE_THRESH)
    for i in range(net.GetNumKeypoints()):
        n = net.GetKeypointName(i)
        if n in COCO:
            name_to_coco[i] = COCO.index(n)
print(f"[pose_gpu] rede {POSE_NET}", flush=True)


def yolo_people(rgb, min_score=0.35, min_kconf=0.3):
    """Todas as pessoas: lista de 17 [x, y, conf] em pixels da imagem."""
    h, w = rgb.shape[:2]
    r = min(YOLO_H / h, YOLO_W / w)
    nh, nw = round(h * r), round(w * r)
    pad = np.full((YOLO_H, YOLO_W, 3), 114, np.uint8)
    top, left = (YOLO_H - nh) // 2, (YOLO_W - nw) // 2
    pad[top:top + nh, left:left + nw] = cv2.resize(rgb, (nw, nh))
    net.setInput(cv2.dnn.blobFromImage(pad, 1 / 255.0, (YOLO_W, YOLO_H), swapRB=False))  # já é RGB
    o = net.forward().reshape(52, -1)
    score = 1 / (1 + np.exp(-o[0]))
    idx = np.where(score >= min_score)[0]
    idx = idx[np.argsort(-score[idx])]
    people, boxes = [], []
    for i in idx:
        k = o[1:, i].reshape(17, 3)
        x = ((k[:, 0] * 2.0 + (Y_ANCH[i, 0] - 0.5)) * Y_STR[i] - left) / r
        y = ((k[:, 1] * 2.0 + (Y_ANCH[i, 1] - 0.5)) * Y_STR[i] - top) / r
        c = 1 / (1 + np.exp(-k[:, 2]))
        vis = c >= min_kconf
        if vis.sum() < 3:
            continue
        # a mesma pessoa aparece em várias âncoras (às vezes com os braços errados):
        # mesmo tronco = mesma pessoa, fica a de maior confiança (a primeira, idx ordenado)
        tv = [j for j in (5, 6, 11, 12) if vis[j]]
        if not tv:
            continue
        cx, cy = float(np.mean(x[tv])), float(np.mean(y[tv]))
        sw = abs(x[5] - x[6]) if vis[5] and vis[6] else (y[vis].max() - y[vis].min()) / 4
        b = (cx, cy, max(sw, 10.0))
        if any(np.hypot(cx - bb[0], cy - bb[1]) < 0.6 * max(b[2], bb[2]) for bb in boxes):
            continue
        boxes.append(b)
        people.append([[float(x[j]), float(y[j]), float(c[j]) if vis[j] else 0.0] for j in range(17)])
        if len(people) >= 5:
            break
    return people


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


def torso_hsv(rgb, k):
    """Pixels HSV do tronco: retângulo entre os ombros e o quadril, encolhido nas bordas."""
    if not (k[5][2] and k[6][2]):
        return None
    xs = [k[5][0], k[6][0]]
    y0 = min(k[5][1], k[6][1])
    hips = [k[i][1] for i in (11, 12) if k[i][2]]
    w = abs(xs[1] - xs[0])
    y1 = max(hips) if hips else y0 + 1.3 * w
    x0, x1 = min(xs) + 0.15 * w, max(xs) - 0.15 * w
    y0, y1 = y0 + 0.10 * (y1 - y0), y1 - 0.10 * (y1 - y0)
    H, W = rgb.shape[:2]
    x0, x1 = int(max(0, x0)), int(min(W, x1))
    y0, y1 = int(max(0, y0)), int(min(H, y1))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    return cv2.cvtColor(np.ascontiguousarray(rgb[y0:y1, x0:x1]), cv2.COLOR_RGB2HSV)


def torso_frac(rgb, k):
    hsv = torso_hsv(rgb, k)
    if hsv is None:
        return 0.0
    m = cv2.inRange(hsv, (vest["h_lo"], vest["s_lo"], vest["v_lo"]), (vest["h_hi"], 255, 255))
    return float(np.count_nonzero(m)) / m.size


def size(k):
    """Tamanho aparente (px): largura dos ombros, ou altura ombro->quadril."""
    w = abs(k[5][0] - k[6][0]) if k[5][2] and k[6][2] else 0.0
    hs = [abs(k[i][1] - k[j][1]) for i, j in ((5, 11), (6, 12)) if k[i][2] and k[j][2]]
    return max([w] + hs)


calib = {"until": 0, "px": []}


def finish_calibration():
    if not calib["px"]:
        print("[pose_gpu] calibração FALHOU: ninguém com ombros visíveis", flush=True)
        return
    px = np.concatenate(calib["px"])
    # o colete é a cor dominante saturada do tronco
    sat = px[(px[:, 1] > 60) & (px[:, 2] > 50)]
    if len(sat) < 50:
        print("[pose_gpu] calibração FALHOU: tronco sem cor forte (está de colete?)", flush=True)
        return
    h = int(np.median(sat[:, 0]))
    s_ = int(np.percentile(sat[:, 1], 20))
    v_ = int(np.percentile(sat[:, 2], 20))
    vest.update({"h_lo": max(0, h - 10), "h_hi": min(179, h + 10),
                 "s_lo": max(60, int(s_ * 0.7)), "v_lo": max(50, int(v_ * 0.6))})
    json.dump(vest, open(VEST_FILE, "w"))
    print(f"[pose_gpu] colete calibrado: {vest} -> {VEST_FILE}", flush=True)


threading.Thread(target=listen_back, daemon=True).start()
cv2_view = serve_view(VIEW_PORT) if VIEW_PORT > 0 else None
VIEW_ON = cv2_view is not None
print(f"[pose_gpu] colete: {'LIGADO ' + str(vest) if VEST_ON else 'desligado (VEST=0)'}", flush=True)
print(f"[pose_gpu] camera {CAM} -> UDP {OUT[1]}" + (f", imagem em :{VIEW_PORT}" if VIEW_PORT else ""), flush=True)

# A câmera é lida numa thread própria: a rede nunca espera o próximo quadro
# chegar, sempre pega o mais novo (os anteriores não processados são descartados).
latest = {"img": None, "seq": 0, "dead": False}
new_frame = threading.Condition()


def capture_loop():
    while True:
        im = cam.Capture(timeout=1000)
        if im is None:
            if not cam.IsStreaming():
                with new_frame:
                    latest["dead"] = True
                    new_frame.notify()
                return
            continue
        with new_frame:
            latest["img"], latest["seq"] = im, latest["seq"] + 1
            new_frame.notify()


threading.Thread(target=capture_loop, daemon=True).start()

n, t_rate, last_view, seen = 0, time.time(), 0.0, 0
snap_left = 0
while True:
    with new_frame:
        new_frame.wait_for(lambda: latest["seq"] != seen or latest["dead"], timeout=1.0)
        if latest["dead"]:
            print("[pose_gpu] camera parou", flush=True)
            break
        if latest["seq"] == seen:
            continue
        img, seen = latest["img"], latest["seq"]
    if os.path.exists("/tmp/pose_snap"):     # bancada: salva o quadro cru (sem desenho) para comparar modelos
        try:
            n_snap = int(open("/tmp/pose_snap").read().strip() or "4")
        except ValueError:
            n_snap = 4
        os.remove("/tmp/pose_snap")
        snap_left = n_snap
    if snap_left > 0:
        ju.cudaDeviceSynchronize()
        cv2.imwrite(f"/tmp/snap_{snap_left}.jpg", cv2.cvtColor(np.array(img, copy=False), cv2.COLOR_RGB2BGR))
        snap_left -= 1
    rgb = None
    if POSE_NET == "yolo":
        ju.cudaDeviceSynchronize()
        rgb = np.array(img, copy=False)
        people = yolo_people(rgb)
    else:
        poses = net.Process(img, overlay="none")
        people = []
        for p in poses:
            k17 = [[0.0, 0.0, 0.0] for _ in range(17)]
            for k in p.Keypoints:
                j = name_to_coco.get(k.ID)
                if j is not None:
                    k17[j] = [float(k.x), float(k.y), 1.0]
            people.append(k17)
    if rgb is None and people and (VEST_ON or VIEW_ON or calib["until"]):
        ju.cudaDeviceSynchronize()
        rgb = np.array(img, copy=False)
    fracs = [torso_frac(rgb, k) if (VEST_ON and rgb is not None) else 1.0 for k in people]
    # calibração: o operador sozinho na imagem, colete no tronco
    if os.path.exists(VEST_TRIGGER):
        os.remove(VEST_TRIGGER)
        calib["until"], calib["px"] = time.time() + 3.0, []
        print("[pose_gpu] calibrando o colete por 3 s: operador SOZINHO na imagem", flush=True)
    if calib["until"]:
        if people and rgb is not None:
            hsv = torso_hsv(rgb, max(people, key=lambda k: sum(c for _, _, c in k)))
            if hsv is not None:
                calib["px"].append(hsv.reshape(-1, 3))
        if time.time() > calib["until"]:
            calib["until"] = 0
            finish_calibration()
    kp = None
    if people:
        if VEST_ON:
            cand = [(f, k) for f, k in zip(fracs, people) if f >= VEST_MIN_FRAC]
            if cand:
                # mais de um com a cor: o mais perto do drone = o maior na imagem
                big = max(size(k) for _, k in cand)
                kp = next(k for _, k in cand if size(k) >= 0.8 * big)   # o mais perto; empate = mais confiável
        else:
            kp = max(people, key=lambda k: sum(c for _, _, c in k))
    if kp is not None:
        tx.sendto(json.dumps({"t": time.time(), "w": img.width, "h": img.height, "kp": kp}).encode(), OUT)
    n += 1
    now = time.time()
    if now - t_rate >= 10.0:
        print(f"[pose_gpu] {n / (now - t_rate):.1f} fps  pessoas={len(people)}"
              + (f"  colete={max(fracs):.2f}" if VEST_ON and fracs else ""), flush=True)
        n, t_rate = 0, now
    if VIEW_ON and now - last_view > 0.15:
        try:
            last_view = now
            if rgb is None:
                ju.cudaDeviceSynchronize()
                rgb = np.array(img, copy=False)
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for k, f in zip(people, fracs):
                op = k is kp
                col = (0, 255, 0) if op else (150, 150, 150)
                for a, b in LIMBS:
                    if k[a][2] and k[b][2]:
                        cv2.line(frame, (int(k[a][0]), int(k[a][1])), (int(k[b][0]), int(k[b][1])), col, 3 if op else 1)
                for x, y, c in k:
                    if c:
                        cv2.circle(frame, (int(x), int(y)), 4, (0, 200, 255) if op else col, -1)
                if VEST_ON and k[5][2] and k[6][2]:
                    cv2.putText(frame, f"colete {f:.2f}", (int(min(k[5][0], k[6][0])), int(min(k[5][1], k[6][1])) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            cv2.putText(frame, gesture["g"] if kp is not None else ("sem colete" if people and VEST_ON else "sem operador"),
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
            if calib["until"]:
                cv2.putText(frame, "CALIBRANDO COLETE", (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok:
                with lock:
                    jpeg["b"] = buf.tobytes()
        except Exception as e:  # a imagem nunca derruba a pose
            print(f"[pose_gpu] imagem desligada: {e}", flush=True)
            VIEW_ON = False
