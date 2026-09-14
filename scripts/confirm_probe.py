#!/usr/bin/env python3
"""
confirm_probe — quantos quadros a barriga produz durante uma pairagem de
confirmação, e como a confiança deles se distribui.

POR QUE ISTO EXISTE. Uma confirmação que falha diz `1/6 looks` e essa linha não
distingue três causas com consertos OPOSTOS:

  * a câmera mal produziu quadros (o simulador roda abaixo do tempo real e
    `confirm_timeout_s` é relógio de PAREDE — 25 s de parede podem ser 4 s de
    simulação)
  * produziu quadros e o detector cortou quase todos antes de publicar
    (`min_confidence` da barriga)
  * publicou e as confianças ficaram abaixo do gate da missão

Rode o stack com `down_min_confidence:=0.0` para que o detector publique TUDO —
sem isso a metade de baixo da distribuição não existe no tópico e a medição
responde a pergunta errada.

    docker exec joao_pessoa_2026-hydrone-1 bash -lc \\
      '. /ws/install/setup.sh && python3 - --out /ws/logs/confirm_probe.csv' \\
      < scripts/confirm_probe.py

Sai um CSV com uma linha por evento (`frame` da câmera, `det` de detecção),
com estampa de parede e confiança. Cruze com as janelas de CONFIRM do log da
missão para responder a pergunta.
"""

import argparse
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image

from hydrone_msgs.msg import PadDetection


SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)


class ConfirmProbe(Node):

    def __init__(self, args):
        super().__init__("confirm_probe")
        self.fh = open(args.out, "w", buffering=1)
        self.fh.write("wall,kind,conf,u,v\n")
        self.n_frames = 0
        self.n_dets = 0
        self.t0 = time.time()

        self.create_subscription(Image, args.image_topic, self._cb_img,
                                 SENSOR_QOS)
        self.create_subscription(PadDetection, args.det_topic, self._cb_det, 20)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"confirm_probe — {args.image_topic} + {args.det_topic} -> "
            f"{args.out}")

    def _cb_img(self, msg):
        self.n_frames += 1
        self.fh.write(f"{time.time():.3f},frame,,,\n")

    def _cb_det(self, msg):
        self.n_dets += 1
        self.fh.write(f"{time.time():.3f},det,{msg.confidence:.3f},"
                      f"{msg.u:.1f},{msg.v:.1f}\n")

    def _report(self):
        dt = max(time.time() - self.t0, 1e-6)
        self.get_logger().info(
            f"{self.n_frames} quadro(s) ({self.n_frames / dt:.1f}/s de "
            f"parede), {self.n_dets} deteccao(oes) publicada(s)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/ws/logs/confirm_probe.csv")
    ap.add_argument("--image-topic", default="/down_cam/image_raw")
    ap.add_argument("--det-topic", default="/hydrone/pads/down/detections")
    args = ap.parse_args()

    rclpy.init()
    node = ConfirmProbe(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
