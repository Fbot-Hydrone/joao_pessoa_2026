#!/usr/bin/env python3
"""Map the BiguaSim Phase 4 structure exactly: teleport the Kopis around it, no SITL.

At each survey pose the agent is re-teleported every tick (so it can't fall),
the first --settle ticks are dropped (the lidar still carries points from the
previous pose for a few ticks: they show up as streaks), then a full Mid-360
sweep is collected. Only ticks where the body really sits where it was put
(ground truth within 1.5 cm / 0.4 deg, four ticks running) count, each
transformed to the world with its own ground-truth pose: teleported into
geometry, physics shoves the body and those scans would smear the map. Poses come from a grid over the rooms of a KnownMap (so every room and
passage gets looked at from inside) at several heights, plus a ring outside.

    python3 tools/phase4/survey_teleport.py OUT.npz [--map rulebook.yaml] [--headless]

The map -> BiguaSim world transform is a guess (from the maze bags):
    world = (MAP_TO_WORLD_X - mx, MAP_TO_WORLD_Y - my), floor at FLOOR_Z.
survey_to_map.py fits the real one from the data, so a rough guess is enough
as long as the poses land inside the rooms.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src', 'hydrone_confined'))

from hydrone_confined.core.known_map import KnownMap  # noqa: E402

# from the first surveys: floor from the outside returns (the bags' 0.16 came
# from the spawn height, a drop point), x from the structure's end walls
MAP_TO_WORLD_X, MAP_TO_WORLD_Y, FLOOR_Z = 4.73, 4.08, 0.0
SPAWN = [3.0, 3.4, 0.78]


def rpy_matrix(r, p, y):
    r, p, y = np.deg2rad([r, p, y])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def to_world(mx, my, h):
    return [MAP_TO_WORLD_X - mx, MAP_TO_WORLD_Y - my, FLOOR_Z + h]


def poses(km, heights=(0.2, 0.35, 0.5, 0.65, 0.8, 0.95, 1.1, 1.25), step=0.45):
    # many heights: with its lowest ray at -4 deg the Mid-360 sees only a few cm
    # below itself on walls 1 m away, so each wall band needs a sensor near it
    out = []
    for r in km.rooms:
        xs = np.arange(r.x[0] + 0.2, r.x[1] - 0.19, step) if r.x[1] - r.x[0] > 0.5 else [r.center[0]]
        ys = np.arange(r.y[0] + 0.2, r.y[1] - 0.19, step) if r.y[1] - r.y[0] > 0.5 else [r.center[1]]
        for x in xs:
            for y in ys:
                for h in heights:
                    out.append((x, y, h))
    # outside: along the front face, the entrance end, the far end
    x0, x1, y0, y1 = km.structure
    for x in np.arange(x0 - 1.0, x1 + 1.01, 0.75):
        for h in (0.6, 1.15, 1.9):
            out.append((x, y1 + 0.8, h))
    for y in np.arange(y0 + 0.3, y1 + 0.81, 0.5):
        for h in (0.9, 1.15):
            out.append((x0 - 0.7, y, h))
            out.append((x1 + 0.7, y, h))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--map', default=os.path.join(ROOT, 'src', 'hydrone_mission', 'config', 'phase4',
                                                   'known_maps', 'rulebook.yaml'))
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--ticks', type=int, default=36)
    ap.add_argument('--settle', type=int, default=12, help='ticks after a teleport before points count')
    a = ap.parse_args()

    import biguasim
    from biguasim.ardubridge import ArduBiguaSimRunner, VEHICLE_REGISTRY

    km = KnownMap.from_yaml(a.map)
    sc = ArduBiguaSimRunner.build_scenario(
        VEHICLE_REGISTRY['KopisX8'], package_name='Competition', world='CompetionMap',
        agent_name='uav0', location=SPAWN, rotation=[0, 0, -90], ticks_per_sec=200)
    sensors = sc['agents'][0]['sensors']
    names = {s.get('sensor_name', s['sensor_type']) for s in sensors}
    for st in ('LocationSensor', 'RotationSensor'):
        if st not in names:
            sensors.append({'sensor_type': st})
    sensors.append({
        'sensor_type': 'RaycastLidar', 'sensor_name': 'Mid360', 'location': [0, 0, 0.1], 'Hz': 200,
        'configuration': {'Channels': 40, 'Range': 70, 'PointsPerSecond': 200000, 'RotationFrequency': 10,
                          'UpperFovLimit': 49, 'LowerFovLimit': -4, 'HorizontalFov': 360,
                          'AtmospAttenRate': 0.0, 'NoiseStdDev': 0.0, 'DropOffGenRate': 0.0,
                          'DropOffIntensityLimit': 1.0, 'DropOffAtZeroIntensity': 0.0,
                          'ShowDebugPoints': False}})
    agent = sc['main_agent']
    plist = poses(km)
    print(f'{len(plist)} survey poses', flush=True)
    rec = {'pts': [], 'loc': [], 'rot': [], 'cmd': []}
    rejected = 0
    env = biguasim.make(scenario_cfg=sc, show_viewport=not a.headless, verbose=False)
    with env:
        ag = next(iter(env.agents.values()))
        env.step([0.0] * 4)
        for k, (mx, my, h) in enumerate(plist):
            loc = to_world(mx, my, h)
            scan, run_ok, gl, gr = [], 0, np.array(loc), np.array([0.0, 0.0, -90.0])
            for i in range(a.ticks):
                ag.teleport(loc, [0.0, 0.0, -90.0])
                s = env.step([0.0] * 4)[agent][0]
                gl = np.asarray(s.get('LocationSensor', loc), dtype=float)
                gr = np.asarray(s.get('RotationSensor', [0, 0, -90]), dtype=float)
                # teleported into something, physics shoves the body: those ticks are no good
                ok = (np.linalg.norm(gl - loc) < 0.015 and abs(((gr[2] + 90) + 180) % 360 - 180) < 0.4
                      and abs(gr[0]) < 0.4 and abs(gr[1]) < 0.4)
                run_ok = run_ok + 1 if ok else 0
                if i >= a.settle and run_ok >= 4 and 'Mid360' in s and s['Mid360'] is not None \
                        and len(s['Mid360']):
                    p = np.asarray(s['Mid360'], dtype=np.float32)
                    p = p.reshape(-1, p.shape[-1])[:, :3]
                    scan.append(p @ rpy_matrix(*gr).T + gl)
            rec['pts'].append(np.concatenate(scan).astype(np.float32) if scan else np.zeros((0, 3), np.float32))
            rec['loc'].append(gl)
            rec['rot'].append(gr)
            rec['cmd'].append([mx, my, h])
            if not scan:
                rejected += 1
            if k % 40 == 0:
                print(f'{k}/{len(plist)} map ({mx:.2f}, {my:.2f}, {h:.2f}) -> world {np.round(gl, 2)} '
                      f'rot {np.round(gr, 1)} pts {len(rec["pts"][-1])}', flush=True)
    print(f'{rejected} poses never settled (inside geometry?), skipped', flush=True)
    np.savez_compressed(a.out, n=len(plist), world_frame=True, loc=np.array(rec['loc']), rot=np.array(rec['rot']),
                        cmd=np.array(rec['cmd']), **{f'p{k}': p for k, p in enumerate(rec['pts'])})
    print('saved', a.out)


if __name__ == '__main__':
    main()
