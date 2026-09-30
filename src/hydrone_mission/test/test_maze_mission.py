"""The whole Phase 4 mission in the headless sim (test/maze_sim), rules layout + variants.

Each scenario flies once (cached) and is checked against the design goals in
docs/Phase 4 Maze Explorer.md: in through the window and never the door, the
inside mapped, out through the far window, landing target at the arena middle,
no contact with any wall. Slow (~30-70 s each): `-m "not slow"` skips them.

    docker exec -u hydrone joao_pessoa_2026-hydrone-1 bash -lc 'source /opt/ros/humble/setup.bash; \
      cd /ws/src/hydrone_mission && python3 -m pytest -q -s -p no:cacheprovider test/test_maze_mission.py'
"""
import numpy as np
import pytest

from hydrone_mission.maze.mission import MissionParams
from maze_sim.runner import SimParams, opening_hit, run
from maze_sim.world import build

SCENARIOS = {
    # (a) rules layout, odom turned a little off the walls
    'nominal': (dict(layout='nominal', yaw0=0.0), dict()),
    # (b) mirrored and turned 90 deg, odom at 0.7 rad to the walls
    'mirrored': (dict(layout='nominal', mirror=True, rot90=1, yaw0=0.7), dict()),
    # (c) different interior: other passages and dead ends
    'variant': (dict(layout='variant', yaw0=-1.2), dict()),
    # (d) twice the pose noise, drift and lag
    'noisy': (dict(layout='nominal', yaw0=2.5),
              dict(tau=0.5, pose_noise=0.04, pose_drift=0.004, accel_noise=0.1)),
    # (e) the lidar never returns off a sill: the generic pipeline can't confirm
    # a window, the rulebook fallback has to pick the entrance
    'no_sills': (dict(layout='nominal', yaw0=0.4), dict(drop_tags=('sill',))),
}

_cache = {}


def fly(name):
    if name not in _cache:
        wkw, skw = SCENARIOS[name]
        world = build(**wkw)
        res = run(world, MissionParams(), SimParams(**skw))
        _cache[name] = (world, res)
        land_err = np.linalg.norm(res.landing_target - world.arena_center) if res.landing_target is not None else None
        print(f'\n[{name}] done={res.done} t={res.time:.0f}s coverage={res.coverage:.3f} '
              f'min_clearance={res.min_clearance:.3f} at {res.min_clearance_at} '
              f'landing_err={land_err} fallback={res.fallback_used} '
              f'scan_ms={res.scan_ms:.1f} (p95 {res.scan_ms_max:.1f}) '
              f'step_ms={res.step_ms:.1f} (p95 {res.step_ms_p95:.1f}) perceive_ms={res.perceive_ms:.1f}')
        for t, msg in res.log:
            print(f'    {t:6.1f} {msg}')
    return _cache[name]


def check_common(world, res, min_coverage=0.95):
    assert res.done, 'mission did not land'
    assert not res.crashed, f'hit a wall: {res.min_clearance:.3f} at {res.min_clearance_at}'
    assert res.min_clearance > 0.0
    # in through the entrance window, out through the exit window, nothing else
    assert res.entry_cross is not None and opening_hit(world, 'entrance', res.entry_cross)
    assert res.exit_cross is not None and opening_hit(world, 'exit', res.exit_cross)
    for t, way, xy in res.crossings:
        assert not opening_hit(world, 'door', xy, margin=0.2), f'door used at t={t}'
        assert opening_hit(world, 'entrance' if way == 'in' else 'exit', xy), (t, way, xy)
    assert res.coverage >= min_coverage
    assert np.linalg.norm(res.landing_target - world.arena_center) < 0.5
    assert np.linalg.norm(res.landed_at - world.arena_center) < 0.7


@pytest.mark.slow
def test_nominal():
    world, res = fly('nominal')
    check_common(world, res)
    assert 'entrance' not in res.fallback_used and 'exit' not in res.fallback_used


@pytest.mark.slow
def test_mirrored_and_rotated():
    world, res = fly('mirrored')
    check_common(world, res)
    assert 'entrance' not in res.fallback_used


@pytest.mark.slow
def test_variant_interior():
    world, res = fly('variant')
    check_common(world, res)
    assert 'entrance' not in res.fallback_used


@pytest.mark.slow
def test_more_noise_and_lag():
    world, res = fly('noisy')
    check_common(world, res)


@pytest.mark.slow
def test_geometry_fallback_picks_the_entrance():
    world, res = fly('no_sills')
    check_common(world, res)
    assert 'entrance' in res.fallback_used
    assert any('FALLBACK' in msg for _, msg in res.log)
