"""Phase 4 competition logic: rulebook geometry fallback and the explorer's API."""
import math

import numpy as np

from hydrone_confined.core.openings import Opening
from hydrone_mission.phase4 import geometry_fallback as gf
from hydrone_mission.phase4.explore_mission import Mission


# ── fallback ────────────────────────────────────────────────────────────────
def _box_roof(near_end, d, across, seed=0):
    """Roof points of a 2 x 6 box starting at near_end, running along d."""
    rng = np.random.default_rng(seed)
    a = rng.uniform(0, 6, 800)
    b = rng.uniform(-1.0, 1.0, 800)
    return np.asarray(near_end) + a[:, None] * d + b[:, None] * across


def test_fallback_fit_and_predictions():
    yaw = 0.5
    d = np.array([math.cos(yaw), math.sin(yaw)])
    w = np.array([-d[1], d[0]])
    near = np.array([1.0, 0.5])
    roof = _box_roof(near, d, w)
    takeoff = near - d * 1.0
    box = gf.fit_box(roof, takeoff, free_score=lambda pts: int(sum((np.asarray(p) - near) @ w > 0 for p in pts)))
    assert np.linalg.norm(box.near_end - near) < 0.15
    assert box.d @ d > 0.99 and box.open_side @ w > 0.99
    pred = gf.predictions(box)
    assert np.linalg.norm(pred['entrance'].center - near) < 0.15
    assert pred['entrance'].normal @ d > 0.99
    assert np.linalg.norm(pred['exit'].center - (near + d * 5.4 + w)) < 0.2
    landing = near - w * 1.0 - d * 2.0 + d * 4.0 + w * 4.0
    assert np.linalg.norm(pred['landing'] - landing) < 0.2


def test_fallback_match_picks_the_gap_on_the_predicted_wall():
    pred = gf.predictions(gf.BoxFit(near_end=np.array([1.0, 0.0]), d=np.array([1.0, 0.0]),
                                    open_side=np.array([0.0, 1.0]), length=6.0, width=2.0))
    right = Opening(center=np.array([1.0, 0.45]), width=0.8, normal=np.array([1.0, 0.0]),
                    confidence=1.0, kind='unknown')
    door = Opening(center=np.array([2.5, 1.0]), width=0.8, normal=np.array([0.0, -1.0]),
                   confidence=1.0, kind='door')
    assert gf.match(pred['entrance'], [door, right], wall_span=1.0) is right
    assert gf.match(pred['entrance'], [door]) is None


# ── mission API ─────────────────────────────────────────────────────────────
def test_mission_first_step_is_takeoff():
    m = Mission()
    cmd = m.step(0.0, (np.zeros(3), 0.0), None)
    assert cmd.kind == 'takeoff' and cmd.z > m.p.flight_z
    assert cmd.debug['phase'] == 'TAKEOFF'
