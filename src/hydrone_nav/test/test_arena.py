"""arena: Figure 7 layout -> map frame, anchored at the takeoff base."""
import math

import pytest

from hydrone_nav.arena import arena_to_map, load_layout


def test_the_takeoff_maps_onto_home():
    assert arena_to_map((1.03, 0.75), (-3.39, 3.0), (1.03, 0.75)) == \
        pytest.approx((-3.39, 3.0))


def test_sim_rotation_matches_the_measured_frame():
    """Sim: map = (y' - 4, 4 - x'). With home where Fig. 7 puts the takeoff,
    pickup 1 (3.21, 1.35) lands at (-2.65, 0.79)."""
    home = (0.75 - 4.0, 4.0 - 1.03)
    assert arena_to_map((3.21, 1.35), home, (1.03, 0.75)) == \
        pytest.approx((1.35 - 4.0, 4.0 - 3.21))


def test_the_default_layout_loads(tmp_path=None):
    import os
    here = os.path.dirname(__file__)
    lay = load_layout(os.path.join(
        here, "..", "..", "hydrone_bringup", "config", "phase2_bases.yaml"))
    assert len(lay.pickup) == 3 and len(lay.delivery) == 3
    assert all(abs(p[2] - 1.6) < 1e-9 for p in lay.pickup)


# ── Phase 1: boxes from the walls, takeoff base in a corner ─────────────────

from hydrone_nav import coverage  # noqa: E402
from hydrone_nav.arena import arena_boxes, walls_box  # noqa: E402

# Rulebook Fig. 3: 8x8 m, the 2x1.5 m takeoff base in a corner, drone on its
# centre facing the far wall.
CORNER = [7.25, 0.75, 1.0, 7.0]     # front, back, left, right


def test_forward_is_plus_y_and_right_is_plus_x_on_the_real_drone():
    assert walls_box(CORNER) == pytest.approx((-1.0, -0.75, 7.0, 7.25))


def test_a_turned_start_turns_the_box():
    # nose along map +x: front -> +x, left -> +y
    assert walls_box(CORNER, start_yaw_deg=0.0) == \
        pytest.approx((-0.75, -7.0, 7.25, 1.0))


def test_off_means_the_given_boxes():
    assert arena_boxes([0.0]) is None
    assert arena_boxes([]) is None


def test_a_typo_stops_the_launch():
    with pytest.raises(ValueError):
        arena_boxes([7.25, 0.75, 1.0])
    with pytest.raises(ValueError):
        arena_boxes([7.25, -0.75, 1.0, 7.0])


def test_the_reject_box_is_the_walls_plus_slack():
    plan, reject = arena_boxes(CORNER, plan_z=(0.3, 2.5))
    assert plan == pytest.approx([-1.0, -0.75, 0.3, 7.0, 7.25, 2.5])
    assert reject == pytest.approx([-1.5, -1.25, 7.5, 7.75])


def _inside(path, walls, margin):
    x0, y0, x1, y1 = walls
    return all(x0 + margin <= p[0] <= x1 - margin
               and y0 + margin <= p[1] <= y1 - margin for p in path)


@pytest.mark.parametrize("corner", range(4))
def test_every_sweep_stays_off_the_walls_from_a_corner_start(corner):
    plan, _ = arena_boxes(CORNER)
    bounds = (tuple(plan[:3]), tuple(plan[3:]))
    walls = walls_box(CORNER)
    inset = 1.2                     # phase1.launch.py survey_inset_m
    for path in (
        coverage.perimeter_sweep(bounds, inset_m=inset, z=2.5,
                                 start_corner=corner),
        coverage.u_sweep(bounds, inset_m=inset, z=2.5, start_corner=corner),
        coverage.camera_lawnmower(bounds, swath_m=1.96, z=2.5,
                                  margin_m=inset),
    ):
        assert path
        # 0.5 m of rulebook error still leaves 0.7 m to the net
        assert _inside(path, walls, inset - 1e-6)


def test_the_sim_box_would_have_left_the_real_arena():
    """Why this exists: +-5 around a corner origin sends the sweep 3.8 m
    behind the takeoff base, outside the net."""
    sim = ((-5.0, -5.0, 0.3), (5.0, 5.0, 2.5))
    path = coverage.perimeter_sweep(sim, inset_m=1.2, z=2.5, start_corner=0)
    assert not _inside(path, walls_box(CORNER), 0.0)


# ── ... and the same through the mission's sweep frame (sweep_side) ─────────

def _framed(builder, plan, yaw, side, kw):
    """What phase1_mission_node flies: built on _sweep_bounds(), started at the
    framed nearest corner, mapped back with _sided()."""
    b = coverage.sweep_frame_bounds(plan, yaw, side)
    kw = dict(kw)
    if builder is not coverage.camera_lawnmower:
        (x0, y0, _), (x1, y1, _) = b
        i = kw["inset_m"]
        corners = [(x0 + i, y0 + i), (x1 - i, y0 + i),
                   (x1 - i, y1 - i), (x0 + i, y1 - i)]
        here = coverage.sweep_frame_xy((0.02, -0.03), yaw, side)
        kw["start_corner"] = coverage.nearest_corner_index(corners, here)
    return coverage.sweep_frame_path(builder(b, z=2.5, **kw), yaw, side)


@pytest.mark.parametrize("side", ["right", "left"])
@pytest.mark.parametrize("yaw_deg", [90.0, 91.0])     # 91: the drone, 11-17
@pytest.mark.parametrize("builder,kw", [
    (coverage.perimeter_sweep, dict(inset_m=1.2)),
    (coverage.u_sweep, dict(inset_m=1.2)),
    (coverage.camera_lawnmower, dict(swath_m=1.96, margin_m=1.2)),
])
def test_the_sided_sweeps_stay_in_a_corner_arena(side, yaw_deg, builder, kw):
    plan, _ = arena_boxes(CORNER)
    bounds = (tuple(plan[:3]), tuple(plan[3:]))
    path = _framed(builder, bounds, math.radians(yaw_deg), side, kw)
    assert path
    assert _inside(path, walls_box(CORNER), 1.2 - 1e-6)
