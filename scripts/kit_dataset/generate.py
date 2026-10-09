"""Gera imagens SIMULADAS do kit da Fase 2 para treinar a YOLO 'lipo'.

Roda no ambiente conda do BiguaSim (nao no container ROS):

    conda run -n biguasim-comp python scripts/kit_dataset/generate.py \
        --out ~/Documents/kit_dataset_sim/raw --views 120

COMO OS ROTULOS SAEM SEM ROTULAR A MAO. Para cada condicao de luz o simulador
sobe, fotografa N poses com a CENA VAZIA (so as bases), spawna os kits e
fotografa as MESMAS N poses de novo. O que mudou entre as duas fotos e o kit:
label.py tira a diferenca, separa a sombra e escreve a caixa. As fotos vazias
viram NEGATIVOS — base azul com reflexo da janela e sem kit, exatamente o que
o modelo atual confunde com o kit (2026-10-08, seed 1).

O QUE VARIA:
  luz       hora do dia (sol/reflexos), tempo (sunny/cloudy), neblina,
            ExposureBias da camera (uma rodada por exposicao)
  distancia altura da camera sobre o kit, 0.4 a 4 m
  angulo    yaw do drone 0-360 (o kit aparece girado na imagem), roll/pitch
            ate +-15 graus (vista obliqua), kit em qualquer ponto do quadro
  kit       yaw aleatorio no spawn (precisa da engine com rotacao no SpawnMesh;
            numa engine antiga o kit nasce reto), em cima das coletas do bloco,
            das entregas (alturas variadas) e no chao
"""
import argparse
import json
import math
import os
import random
import uuid

import re

from PIL import Image

import biguasim

biguasim.environments.BiguaSimEnvironment._timeout = property(lambda self: None)

AGENT = "uav0"
BASE_BP = "Blueprint'/Game/Maps/arena__2_/BP_base.BP_base_C'"
KIT_BP = ("Blueprint'/Game/HolodeckContent/Agents/HolybroX500/"
          "BP_Lipo_battery_safe_bag_fbx.BP_Lipo_battery_safe_bag_fbx_C'")
HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = os.path.join(HERE, "..", "..", "src", "hydrone_bringup", "config",
                      "phase2_bases.yaml")

# Uma rodada por linha: (hora, tempo, neblina, ExposureBias, ISO, obturador).
# hora/tempo/neblina so valem com --weather: neste mundo set_day_time e
# set_weather TRAVAM o ambiente (medido 2026-10-08). Sem eles a luz varia pela
# camera (exposicao/ISO/obturador, uma rodada cada) e os reflexos variam
# sozinhos com a pose — label.py ainda soma brilho/contraste/cor/ruido.
CONDITIONS = [
    (12, "sunny", 0.00, 10.0, 1600.0, 60.0),
    (9, "sunny", 0.00, 8.5, 1600.0, 60.0),
    (16, "sunny", 0.00, 11.5, 1600.0, 60.0),
    (18, "sunny", 0.02, 10.0, 800.0, 60.0),
    (11, "cloudy", 0.00, 10.0, 3200.0, 60.0),
    (14, "cloudy", 0.03, 9.0, 1600.0, 120.0),
    (8, "sunny", 0.00, 12.5, 1600.0, 30.0),
]


def load_layout(path):
    """The few keys of phase2_bases.yaml this needs, without pyyaml (the
    BiguaSim conda env does not have it). Lists are `- [a, b, c]` lines."""
    out, key = {"pickup": [], "delivery": []}, None
    for line in open(path):
        line = line.split("#", 1)[0].rstrip()
        if not line:
            continue
        m = re.match(r"^(\w+):\s*(.*)$", line)
        if m:
            key, val = m.group(1), m.group(2).strip()
            if val.startswith("["):
                out[key] = [float(v) for v in val.strip("[]").split(",")]
            continue
        m = re.match(r"^\s*-\s*\[(.*)\]$", line)
        if m and key in ("pickup", "delivery"):
            out[key].append([float(v) for v in m.group(1).split(",")])
    return out


def make_env(exposure, iso=1600.0, shutter=60.0):
    cam = {"sensor_type": "RGBCamera", "sensor_name": "DownCamera", "Hz": 30,
           "location": [0.0, 0, -0.1], "rotation": [0, 90, 0],
           "configuration": {"CaptureWidth": 640, "CaptureHeight": 480,
                             "FOV": 90, "ManualExposure": True,
                             "ExposureBias": exposure, "ShutterSpeed": shutter,
                             "ISO": iso}}
    agent = {"agent_name": AGENT, "agent_type": "HolybroX500",
             "sensors": [{"sensor_type": "DynamicsSensor", "socket": "IMUSocket",
                          "configuration": {"UseCOM": True, "UseRPY": False}},
                         cam],
             "dynamics": {"batch_size": 1}, "control_abstraction": "cmd_pos_yaw",
             "location": [0.0, 0.0, 4.5], "rotation": [0.0, 0.0, 0.0]}
    sc = {"package_name": "Competition", "world": "CompetionMap",
          "main_agent": AGENT, "frames_per_sec": 60, "agents": [agent]}
    env = biguasim.environments.BiguaSimEnvironment(
        scenario=sc,
        binary_path=biguasim.packagemanager.get_binary_path_for_package(
            "Competition"),
        show_viewport=True, verbose=False, uuid=str(uuid.uuid4()),
        ticks_per_sec=30)
    env.reset()
    return env


def spawn(env, bp, x, y, z, yaw=None):
    """SpawnMesh no frame do sim (Y invertido, como no interface.py)."""
    nums = [x, -y, z]
    if yaw is not None:
        nums += [0.0, 0.0, -yaw]
    env.send_world_command("CustomCommand", string_params=["SpawnMesh", bp],
                           num_params=nums)


def grab(env):
    s = None
    for _ in range(3):                    # teleport + render settle
        s = env.tick()
    img = s[AGENT][0]["DownCamera"]
    return img[:, :, 2::-1] if img.shape[2] == 4 else img[:, :, ::-1]


def scene(layout, rng):
    """Where the bases go and where kits will sit, in sim WORLD coordinates."""
    world = lambda x, y: (4.0 - x, 4.0 - y)  # noqa: E731
    bases, kits = [], []
    for x, y, z in (p[:3] for p in layout["pickup"]):
        wx, wy = world(x, y)
        bases.append((wx, wy, z))
        kits.append((wx, wy, z))
    lo, hi = layout.get("delivery_z_range", [0.0, 1.5])
    for d in layout["delivery"]:
        wx, wy = world(d[0], d[1])
        z = rng.uniform(lo, hi)
        bases.append((wx, wy, z))
        if rng.random() < 0.6:            # some deliveries carry a kit too
            kits.append((wx + rng.uniform(-0.25, 0.25),
                         wy + rng.uniform(-0.25, 0.25), z))
    for _ in range(2):                    # kits on the bare floor
        kits.append((rng.uniform(-3.0, 1.5), rng.uniform(-3.0, 1.0), 0.0))
    return bases, kits


def views(kits, n, rng):
    """Camera poses aimed so a kit lands anywhere in frame, at any range."""
    out = []
    for _ in range(n):
        kx, ky, kz = rng.choice(kits)
        h = rng.uniform(0.4, 4.0)                  # height over the kit top
        reach = h * math.tan(math.radians(38))     # keep it inside FOV 90
        out.append({
            "loc": [kx + rng.uniform(-reach, reach),
                    ky + rng.uniform(-reach, reach), kz + 0.07 + h],
            "rot": [rng.uniform(-15, 15), rng.uniform(-15, 15),
                    rng.uniform(0, 360)]})
    return out


def run(cond_i, cond, args, layout):
    hour, weather, fog, exposure, iso, shutter = cond
    rng = random.Random(args.seed * 100 + cond_i)
    tag = f"c{cond_i:02d}"
    os.makedirs(args.out, exist_ok=True)
    log = lambda m: print(f"[{tag}] {m}", flush=True)  # noqa: E731
    env = make_env(exposure, iso, shutter)
    log("env up")
    if args.weather:
        env.weather.set_weather(weather)
        env.weather.set_fog_density(fog)
        env.weather.set_day_time(hour)
        env.tick()
        log(f"weather {weather} hour {hour} fog {fog}")
    bases, kits = scene(layout, rng)
    for b in bases:
        spawn(env, BASE_BP, *b)
    for _ in range(60):
        env.tick()
    log(f"{len(bases)} bases spawned")
    agent = env.agents[AGENT + "-id0"]
    poses = views(kits, args.views, rng)

    def sweep(kind):
        for i, p in enumerate(poses):
            agent.teleport(p["loc"], p["rot"])
            Image.fromarray(grab(env)).save(
                os.path.join(args.out, f"{tag}_{i:04d}_{kind}.png"))

    sweep("empty")
    log("empty sweep done")
    for kx, ky, kz in kits:
        spawn(env, KIT_BP, kx, ky, kz + 0.15, rng.uniform(0, 360))
    for _ in range(120):                  # let the kits fall and settle
        agent.teleport([0.0, 0.0, 4.5], [0, 0, 0])
        env.tick()
    sweep("kit")
    meta = {"condition": {"exposure": exposure, "iso": iso,
                          "shutter": shutter,
                          "weather_cmds": bool(args.weather), "hour": hour,
                          "weather": weather, "fog": fog},
            "bases": bases, "kits_spawned": kits, "poses": poses}
    with open(os.path.join(args.out, f"{tag}_meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(f"{tag}: {len(poses)} poses, exposure {exposure} iso {iso} "
          f"shutter {shutter}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--views", type=int, default=120)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--weather", action="store_true",
                    help="also send set_weather/set_day_time (HANG in this "
                         "world as of 2026-10-08; off by default)")
    ap.add_argument("--conditions", default="all",
                    help="indices, e.g. 0,2,4 (default: all)")
    args = ap.parse_args()
    args.out = os.path.expanduser(args.out)
    layout = load_layout(LAYOUT)
    idx = (range(len(CONDITIONS)) if args.conditions == "all"
           else [int(i) for i in args.conditions.split(",")])
    for i in idx:
        run(i, CONDITIONS[i], args, layout)


if __name__ == "__main__":
    main()
