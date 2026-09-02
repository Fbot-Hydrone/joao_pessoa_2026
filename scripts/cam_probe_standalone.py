#!/usr/bin/env python3
"""
cam_probe_standalone — abre a BiguaSim direto, sem ROS e sem o stack, mostra a
imagem da câmera e MEDE a saturação dela.

    python3 scripts/cam_probe_standalone.py
    python3 scripts/cam_probe_standalone.py --exposure -5      # testa o botão
    python3 scripts/cam_probe_standalone.py --exposure 4       # o default da engine
    python3 scripts/cam_probe_standalone.py --world Pier-Harbor --package SkyDive

Roda no ambiente da biguasim (conda biguasim_env), NÃO no distrobox do ROS —
este script não importa rclpy.

PARA QUE SERVE. Responde duas coisas que o stack não consegue responder:

1. A câmera funciona fora do nosso código? Se a imagem aparecer aqui, o
   problema não é a BiguaSim nem o sensor.

2. `ExposureCompensation` é honrado pelo `RGBCamera`, ou ignorado? O `config`
   vai INTEIRO para a engine (`SensorDefinition.get_config_json_string` faz
   `json.dumps(self.config)`, sem filtro), mas quem decide se a chave tem
   efeito é o binário UE, que não está no repositório. Então não dá para saber
   lendo código — só medindo.

   Rode com `--exposure -5` e depois `--exposure 4`. Se a saturação mediana
   impressa mudar, a chave é honrada e dá para consertar a imagem sem trocar o
   tipo do sensor. Se não mudar NADA, ela é ignorada e o caminho é migrar para
   `sensor_type: CameraSensor`, que documenta esses botões (e traz FovAngle de
   brinde).

POR QUE A SATURAÇÃO É O NÚMERO QUE IMPORTA. O detector de pads aceita azul com
`blue_hsv_low = [95, 30, 50]`, ou seja S >= 30. Superexposição empurra o pixel
para o branco, e branco é S = 0. Medido em bancada: um azul de pad saturado dá
S = 229; o mesmo azul estourado dá S = 36 — seis pontos acima do corte. O matiz
sobrevive, o valor satura, e é a SATURAÇÃO que decide se o pad existe ou não
para o detector.
"""

import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("precisa de opencv: pip install opencv-python")

import biguasim

# A banda do detector, para o probe reportar contra o número em vigor.
BLUE_LOW = np.array((95, 30, 50), np.uint8)
BLUE_HIGH = np.array((135, 255, 255), np.uint8)

CAM = "MinhaCam"


def scenario(args):
    cam_cfg = {"CaptureWidth": args.width, "CaptureHeight": args.height}
    if args.exposure is not None:
        # Estas são as chaves em teste. Se forem ignoradas, a imagem não muda.
        cam_cfg["ExposureMethod"] = "AEM_Manual"
        cam_cfg["ExposureCompensation"] = float(args.exposure)
    return {
        "package_name": args.package,
        "world": args.world,
        "main_agent": "uav0",
        "ticks_per_sec": 20,
        "frames_per_sec": False,
        "octree_min": 0.02,
        "octree_max": 5.0,
        "agents": [
            {
                "agent_name": "uav0",
                "agent_type": "DjiMatrice",
                "sensors": [
                    {
                        "sensor_type": "DynamicsSensor",
                        "socket": "IMUSocket",
                        "configuration": {"UseCOM": True, "UseRPY": False},
                    },
                    {
                        "sensor_type": "RGBCamera",
                        "sensor_name": CAM,
                        "socket": "CameraSocket",
                        # location/rotation são chaves DO SENSOR, irmãs de
                        # sensor_type — não vão dentro de `configuration`.
                        # environments.py monta sensor_config com elas nesse
                        # nível; postas lá dentro viram JSON para a engine e
                        # não posicionam nada.
                        "location": [1, 0, 0],
                        "rotation": [0, 0, 0],
                        "configuration": cam_cfg,
                    },
                    {
                        "sensor_type": "RangeFinderSensor",
                        "socket": "CameraSocket",
                        "configuration": {
                            "LaserMaxDistance": 1000,
                            "LaserCount": 1,
                            "LaserAngle": 0,
                            "LaserDebug": False,
                        },
                    },
                ],
                "dynamics": {"batch_size": 1},
                "control_abstraction": "cmd_vel",
                "location": [205, 25, 30],
                "rotation": [0.0, 0.0, 180],
            }
        ],
        "window_width": 1280,
        "window_height": 720,
    }


def find_image(state, name):
    """A imagem da câmera, seja qual for o aninhamento do state.

    `_get_single_state` devolve {sensor_name: array}, mas com batch/múltiplos
    agentes aparece um nível a mais ({agent: [ {sensor: array} ]}). Em vez de
    apostar num formato, procura o array HxWx3-ou-4 sob a chave da câmera.
    """
    def walk(node):
        if isinstance(node, np.ndarray):
            return node if node.ndim == 3 and node.shape[2] in (3, 4) else None
        if isinstance(node, dict):
            if name in node:
                got = walk(node[name])
                if got is not None:
                    return got
            for v in node.values():
                got = walk(v)
                if got is not None:
                    return got
        if isinstance(node, (list, tuple)):
            for v in node:
                got = walk(v)
                if got is not None:
                    return got
        return None
    return walk(state)


def report(bgr, tag):
    """Saturação e clipping — os dois números que decidem a questão."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    mask = cv2.inRange(hsv, BLUE_LOW, BLUE_HIGH)
    n_blue = int((mask > 0).sum())
    blown = float((v >= 250).mean())
    line = (f"[{tag}] S mediana {np.median(s):5.1f}  S p90 "
            f"{np.percentile(s, 90):5.1f} | V mediana {np.median(v):5.1f} | "
            f"clipado (V>=250) {100 * blown:4.1f}% | pixels na banda azul "
            f"{n_blue}")
    if n_blue:
        line += f" (S mediana neles {np.median(s[mask > 0]):.0f})"
    return line


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--exposure", type=float, default=None,
                    help="ExposureCompensation + AEM_Manual. Omita para usar "
                         "o default da engine (que é 4).")
    ap.add_argument("--package", default="SkyDive")
    ap.add_argument("--world", default="Pier-Harbor")
    ap.add_argument("--width", type=int, default=768)
    ap.add_argument("--height", type=int, default=768)
    ap.add_argument("--headless", action="store_true",
                    help="sem janela; só imprime os números")
    args = ap.parse_args()

    exp = "default da engine" if args.exposure is None else f"{args.exposure}"
    print(f"ExposureCompensation: {exp}   (q na janela, ou Ctrl-C, para sair)")

    hover = [0.0, 0.0, 0.0]        # cmd_vel é [vx, vy, vz]
    shown = False

    with biguasim.make(scenario_cfg=scenario(args), verbose=False) as env:
        while True:
            # step() EXIGE o comando e exige reset() antes — biguasim.make já
            # chama reset() no construtor, mas o comando não é opcional:
            # `env.step()` sem argumento é TypeError.
            state = env.step(hover)

            img = find_image(state, CAM)
            if img is None:
                if not shown:
                    print("nenhuma imagem no state. Chaves vistas:",
                          list(state.keys()) if isinstance(state, dict) else type(state))
                    shown = True
                continue

            # RGBCamera entrega RGBA; cv2 quer BGR.
            bgr = (cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if img.shape[2] == 4
                   else cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

            print(report(bgr, exp))

            if not args.headless:
                cv2.imshow("camera", bgr)      # imshow precisa da IMAGEM
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
