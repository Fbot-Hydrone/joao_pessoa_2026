#!/usr/bin/env python3
"""
yolo_pad_detector — detecta a base de pouso com o modelo YOLOv11-seg treinado
(1_train_yolo.py), com a MESMA interface pública de hydrone_vision.pad_detector
.PadDetector, para ser um substituto plug-and-play dentro de
pad_detector_node.py.
pad_detector_node.py já resolve tudo que não é "achar o pixel da base":
projeção por profundidade/plano do chão/octomap, TF do mount da camera, QoS,
publicação em hydrone_msgs/PadDetection, throttling de log, tudo isso já foi
medido e testado em campo. Reescrever esse pipeline do zero (como fazia o
3_landing_detection.py) joga fora tudo isso E não publica nada em ROS.

Este módulo faz só a parte que muda: bgr ndarray -> lista de PadDetection2D.
pad_detector_node.py ganha um parâmetro `detector_backend` ("cv" | "yolo") e
escolhe entre hydrone_vision.pad_detector.PadDetector (o de hoje) e esta
classe — o resto do nó não muda uma linha.

Este módulo é, como pad_detector.py, deliberadamente livre de ROS: recebe um
ndarray BGR e devolve dataclasses.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from hydrone_vision.pad_detector import PadDetection2D


class YoloPadDetector:
    """Stateless por fora — carrega o modelo uma vez, chama .detect() por frame.

    Mantém a MESMA API pública que PadDetector.detect() usa em
    pad_detector_node.py: `.detect(bgr) -> list[PadDetection2D]`, mais
    `.last_field_mask` / `.last_yellow_mask` (aqui sempre None — não há
    máscara de cor) e `.reject` / `.probe` (aqui vazios — a rejeição do YOLO
    já é por confiança, não por uma cascata de checks).
    """

    def __init__(
        self,
        weights_path: str,
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        target_classes: tuple[str, ...] | None = None,
        device: str = "cpu",
        imgsz: int = 640,
        max_detections: int = 8,
        # Mesma ideia de ignore_regions do pad_detector.py: partes do frame
        # que o próprio drone ocupa (trem de pouso etc.), como frações
        # x0,y0,x1,y1. Um objeto escuro com borda clara sobre chão azul passa
        # em qualquer teste — inclusive na rede — então isso continua sendo
        # resolvido por posição, não por aparência.
        ignore_regions=(),
    ):
        # Import tardio: ultralytics é pesado (torch) e só é preciso quando
        # este backend é de fato escolhido — o nó não deve pagar esse custo
        # de import quando roda com detector_backend:="cv".
        from ultralytics import YOLO

        self.model = YOLO(weights_path)
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.device = device
        self.imgsz = int(imgsz)
        self.max_detections = int(max_detections)
        self.ignore_regions = self._as_regions(ignore_regions)

        # class_id -> nome, do próprio modelo treinado no script 1.
        self.names = self.model.names
        if target_classes is None:
            self.target_class_ids = None  # aceita qualquer classe do modelo
        else:
            wanted = set(target_classes)
            self.target_class_ids = {
                cid for cid, name in self.names.items() if name in wanted
            }

        # Mantidos só para compatibilidade com quem lê essas propriedades do
        # PadDetector clássico (ex.: pad_detector_node não usa, mas testes/
        # debug podem).
        self.last_field_mask: np.ndarray | None = None
        self.last_yellow_mask: np.ndarray | None = None
        self.reject: dict = {}
        self.probe: list = []

    @staticmethod
    def _as_regions(spec) -> tuple:
        if not spec:
            return ()
        flat = list(spec)
        if len(flat) % 4:
            raise ValueError(
                "ignore_regions precisa vir em grupos de 4 frações "
                f"(x0,y0,x1,y1); recebi {len(flat)} valores")
        return tuple(tuple(float(v) for v in flat[i:i + 4])
                     for i in range(0, len(flat), 4))

    def _inside_ignore_region(self, cx: float, cy: float, w: int, h: int) -> bool:
        fx, fy = cx / w, cy / h
        return any(x0 <= fx <= x1 and y0 <= fy <= y1
                   for x0, y0, x1, y1 in self.ignore_regions)

    # ────────────────────────────────────────────────────────────────────────
    # API pública — mesma assinatura de PadDetector.detect
    # ────────────────────────────────────────────────────────────────────────

    def detect(self, bgr: np.ndarray) -> list[PadDetection2D]:
        """Roda o modelo e devolve uma detecção por máscara, melhor confiança primeiro."""
        if bgr is None or bgr.size == 0:
            return []

        h_img, w_img = bgr.shape[:2]

        results = self.model.predict(
            bgr,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )[0]

        found: list[PadDetection2D] = []
        if results.masks is None:
            return found

        masks = results.masks.data.cpu().numpy()  # (N, h, w) na resolução do modelo
        boxes = results.boxes

        for i in range(masks.shape[0]):
            class_id = int(boxes.cls[i].item())
            if (self.target_class_ids is not None
                    and class_id not in self.target_class_ids):
                continue
            confidence = float(boxes.conf[i].item())

            mask = masks[i]
            if mask.shape != (h_img, w_img):
                mask = cv2.resize(mask, (w_img, h_img),
                                  interpolation=cv2.INTER_NEAREST)
            mask_u8 = (mask > 0.5).astype(np.uint8) * 255

            contours, _ = cv2.findContours(
                mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)

            M = cv2.moments(mask_u8, binaryImage=True)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]

            if self.ignore_regions and self._inside_ignore_region(
                    cx, cy, w_img, h_img):
                continue

            area = float(M["m00"])
            radius_px = math.sqrt(area / math.pi)

            found.append(PadDetection2D(
                u=float(cx),
                v=float(cy),
                radius_px=radius_px,
                area_px=area,
                confidence=confidence,
                contour=contour,
                scores={"class": self.names.get(class_id, str(class_id)),
                       "yolo_conf": round(confidence, 3)},
            ))

        found.sort(key=lambda d: d.confidence, reverse=True)
        return found[: self.max_detections]
