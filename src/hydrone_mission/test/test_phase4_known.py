"""The known-map Phase 4 stack in the kinematic sim, on the rulebook maze.

Localizer + pose mux + KnownMission wired like the ROS graph (known_stack.py),
flown against worlds built from the same YAML, then against worlds that
disagree with it. Checked: in through the entrance and out through the exit
only (never the door), all QR codes read, no wall contact, landing on the
pad, and the flying pose within 0.15 m of the truth inside (that includes
the lag of a pose at up to 0.25 m/s). Slow (~5 min each here):
`-m "not slow"` skips them.
"""
import os

import numpy as np
import pytest

from hydrone_confined.core.known_map import KnownMap
from hydrone_confined.sim.runner import SimParams, opening_hit, run
from hydrone_confined.sim.world import from_known_map
from hydrone_mission.phase4.known_mission import KnownParams
from hydrone_mission.phase4.known_stack import KnownStack

MAP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'config', 'phase4', 'known_maps', 'rulebook.yaml')

SCENARIOS = {
    # the world is the map
    'nominal': (dict(), dict()),
    # LIO slides 0.15 m/s along the maze while inside (a degenerate corridor)
    'lio_degenerate': (dict(), dict(degen_rate=0.15)),
    # a soft passage sits 0.4 m from where the rulebook render put it
    'passage_moved': (dict(perturb={'t2_r5': {'u': (0.68, 1.42)}}), dict()),
    # the drone was put down 12 deg and 15 cm off the takeoff prior
    'start_off': (dict(yaw_error=np.deg2rad(12.0), xy_error=(0.15, -0.1)), dict()),
    # LIO dies in the middle of the maze for 40 s and comes back 2.8 m off
    # (FAST-LIO "No Effective Points" in a small room, seen in BiguaSim)
    'lio_dies': (dict(), dict(lio_blackout=(95.0, 135.0), lio_jump=(2.8, 0.0))),
    # a passage the map has is walled up: no route left, the explorer takes over
    'passage_closed': (dict(perturb={'b2_t2': None}), dict(max_time=600.0)),
}

_cache = {}


def fly(name):
    if name not in _cache:
        km = KnownMap.from_yaml(MAP)
        wkw, skw = SCENARIOS[name]
        world = from_known_map(km, qr=5, seed=2, **wkw)
        stack = KnownStack(km, KnownParams())
        err = []

        def on_step(t, p, st, cmd, f):
            pose = st.mux.pose()
            if pose is not None and st.mux.mode == 'map':
                err.append(float(np.linalg.norm(pose[:2] - f.truth[:2])))
        res = run(world, stack, SimParams(**{'max_time': 560.0, 'n_rays': 10000, **skw}), on_step=on_step,
                  landing_world=lambda s, f: s.mission.landing)
        _cache[name] = (km, world, stack, res, np.array(err) if err else np.array([np.inf]))
        print(f'\n[{name}] done={res.done} t={res.time:.0f}s crashed={res.crashed} '
              f'min_clearance={res.min_clearance:.3f} at {res.min_clearance_at} landed={res.landed_at} '
              f'qr={stack.mission.qr_seen} fallback={res.fallback_used} '
              f'map-pose err mean={_cache[name][4].mean():.3f} max={_cache[name][4].max():.3f} '
              f'loc_ms={np.mean(stack.loc.ms):.1f}')
        for t, msg in res.log:
            print(f'    {t:6.1f} {msg}')
    return _cache[name]


def check_common(km, world, stack, res, err, qr=5, map_err=0.15):
    assert res.done, 'did not land'
    assert not res.crashed, f'hit a wall: {res.min_clearance:.3f} at {res.min_clearance_at}'
    assert res.entry_cross is not None and opening_hit(world, 'entrance', res.entry_cross)
    assert res.exit_cross is not None and opening_hit(world, 'exit', res.exit_cross)
    for t, way, xy in res.crossings:
        assert not opening_hit(world, 'door', xy, margin=0.2), f'door used at t={t}'
    assert len(stack.mission.qr_seen) >= qr
    assert np.linalg.norm(res.landed_at - km.landing) < 0.35
    assert err.max() < map_err, f'map pose off by {err.max():.3f} m while flying on it'


@pytest.mark.slow
def test_nominal():
    km, world, stack, res, err = fly('nominal')
    check_common(km, world, stack, res, err)
    assert 'enter-on-odom' not in res.fallback_used


@pytest.mark.slow
def test_lio_degenerate_inside():
    km, world, stack, res, err = fly('lio_degenerate')
    check_common(km, world, stack, res, err)


@pytest.mark.slow
def test_soft_passage_found_where_it_really_is():
    km, world, stack, res, err = fly('passage_moved')
    check_common(km, world, stack, res, err)
    assert 'moved:t2_r5' in res.fallback_used


@pytest.mark.slow
def test_start_pose_off_the_prior():
    km, world, stack, res, err = fly('start_off')
    check_common(km, world, stack, res, err)


@pytest.mark.slow
def test_lio_dying_inside_is_flown_through_on_the_map():
    km, world, stack, res, err = fly('lio_dies')
    check_common(km, world, stack, res, err)
    assert any('map pose alone' in m for _, m in res.log)


@pytest.mark.slow
def test_walled_up_passage_hands_over_to_the_explorer():
    km, world, stack, res, err = fly('passage_closed')
    assert 'blocked:b2_t2' in res.fallback_used and 'explore' in res.fallback_used
    assert not res.crashed
    # the explorer can't reach the exit either (the maze is a chain): it must
    # at least not leave through the door, and land
    assert res.done
    for t, way, xy in res.crossings:
        assert not opening_hit(world, 'door', xy, margin=0.2), f'door used at t={t}'
