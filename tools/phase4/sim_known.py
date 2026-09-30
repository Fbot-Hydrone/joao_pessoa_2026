#!/usr/bin/env python3
"""Fly the known-map Phase 4 stack in the kinematic sim, headless, no ROS.

    python3 tools/phase4/sim_known.py                        # rulebook maze, 5 QR codes
    python3 tools/phase4/sim_known.py --degen 0.15           # LIO slides 0.15 m/s inside
    python3 tools/phase4/sim_known.py --closed b2_t2 --viz   # a passage walled up, stream to the viewer
    python3 tools/phase4/sim_known.py --realtime --viz       # watch it at real speed

--viz streams to tools/maze_viewer (UDP 47800). Prints the mission log, the
result and the pose error against ground truth.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [os.path.join(ROOT, 'src', 'hydrone_confined'), os.path.join(ROOT, 'src', 'hydrone_mission')]

import numpy as np  # noqa: E402

from hydrone_confined.core.known_map import KnownMap  # noqa: E402
from hydrone_confined.core.viz_udp import VizUdp  # noqa: E402
from hydrone_confined.sim.runner import SimParams, run  # noqa: E402
from hydrone_confined.sim.world import from_known_map  # noqa: E402
from hydrone_mission.phase4.known_mission import KnownParams  # noqa: E402
from hydrone_mission.phase4.known_stack import KnownStack  # noqa: E402

DEFAULT_MAP = os.path.join(ROOT, 'src', 'hydrone_mission', 'config', 'phase4', 'known_maps', 'rulebook.yaml')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--world-map', default=None, help='build the world from another map (the drone gets --map)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--qr', type=int, default=5)
    ap.add_argument('--qr-seed', type=int, default=2)
    ap.add_argument('--degen', type=float, default=0.0, help='m/s of LIO sliding while inside')
    ap.add_argument('--closed', nargs='*', default=[], help='passages walled up in the world')
    ap.add_argument('--yaw-error', type=float, default=0.0, help='deg, start heading off the map\'s')
    ap.add_argument('--xy-error', type=float, nargs=2, default=(0.0, 0.0))
    ap.add_argument('--no-spin', action='store_true')
    ap.add_argument('--qr-target', type=int, default=5)
    ap.add_argument('--max-time', type=float, default=560.0)
    ap.add_argument('--rays', type=int, default=10000)
    ap.add_argument('--viz', action='store_true')
    ap.add_argument('--realtime', action='store_true')
    a = ap.parse_args()

    km = KnownMap.from_yaml(a.map)
    wkm = KnownMap.from_yaml(a.world_map) if a.world_map else km
    world = from_known_map(wkm, yaw_error=np.deg2rad(a.yaw_error), xy_error=tuple(a.xy_error),
                           perturb={n: None for n in a.closed}, qr=a.qr, seed=a.qr_seed)
    stack = KnownStack(km, KnownParams(spin=not a.no_spin, qr_target=a.qr_target))
    viz = None
    if a.viz:
        viz = VizUdp()
        viz.set_map(km)
        viz.reset()
    err = []
    wall0 = time.time()

    def on_step(t, p, st, cmd, f):
        pose = st.mux.pose()
        if pose is not None:
            err.append((t, float(np.linalg.norm(pose[:2] - f.truth[:2])), st.mux.mode, st.phase))
        if viz is not None and pose is not None:
            viz.frame(t, pose, phase=st.phase, mode=st.mux.mode, truth=f.truth, fix=st.fix,
                      particles=st.loc.loc.xy, path=st.mission.nav.path, goal=st.mission._goal,
                      qr_world=world.meta['qr'], qr_seen=st.mission.qr_seen, mount=st.locp.mount[:2])
        if a.realtime:
            lag = t - (time.time() - wall0)
            if lag > 0:
                time.sleep(lag)

    res = run(world, stack, SimParams(max_time=a.max_time, n_rays=a.rays, seed=a.seed, degen_rate=a.degen),
              on_step=on_step, landing_world=lambda s, f: s.mission.landing)
    for t, m in res.log:
        print(f'{t:6.1f} {m}')
    e = np.array([x[1] for x in err]) if err else np.zeros(1)
    land_err = np.linalg.norm(res.landed_at - km.landing) if res.landed_at is not None else None
    print(f'done={res.done} crashed={res.crashed} t={res.time:.0f}s wall={time.time() - wall0:.0f}s '
          f'min_clearance={res.min_clearance:.3f} at {res.min_clearance_at}')
    print(f'landed={None if res.landed_at is None else np.round(res.landed_at, 2)} landing_err={land_err} '
          f'qr_read={stack.mission.qr_seen} fallback={res.fallback_used}')
    print('crossings', [(t, w, np.round(xy, 2).tolist()) for t, w, xy in res.crossings])
    inside = [x[1] for x in err if x[2] == 'map']
    print(f'pose err mean {e.mean():.3f} max {e.max():.3f}; on map pose: mean '
          f'{np.mean(inside) if inside else float("nan"):.3f} max {max(inside) if inside else float("nan"):.3f}')
    print(f'localizer {np.mean(stack.loc.ms):.1f} ms (p95 {np.percentile(stack.loc.ms, 95):.1f}), '
          f'mission {np.mean(stack.mission.step_ms):.1f} ms')
    return 0 if res.done and not res.crashed else 1


if __name__ == '__main__':
    sys.exit(main())
