"""phase2_core: positions, scan, clamp, kit alignment and the pilot latch."""
import math

import pytest

from hydrone_mission import phase2_core as core

GRID = {"step_m": 0.5, "origin_m": [0.0, 0.0], "centre": "node"}


# ── positions ───────────────────────────────────────────────────────────────

def test_an_empty_parameter_keeps_the_layout_entry():
    assert core.base_entry("", (4.48, 6.92), GRID) == (4.48, 6.92)


def test_scan_marks_a_base_unknown_even_with_a_layout_entry():
    assert core.base_entry("scan", (4.48, 6.92), GRID) is None
    assert core.base_entry(" SCAN ", None, GRID) is None


def test_a_parameter_overrides_the_layout_in_metres_or_grid():
    assert core.base_entry("[1.0, 2.0, 1.5]", (9, 9), GRID) == (1.0, 2.0, 1.5)
    assert core.base_entry("{grid: [9, 14]}", None, GRID) == \
        pytest.approx((4.5, 7.0))


def test_no_parameter_and_no_layout_entry_is_missing():
    assert core.merge_positions(["", "", ""], [(1, 1)], GRID) == \
        [(1, 1), None, None]


# ── the scan ────────────────────────────────────────────────────────────────

PLAN = [-1.0, -0.75, 0.3, 7.0, 7.25, 3.5]       # a corner start, real drone


def test_the_scan_is_the_closed_perimeter_then_lanes_inside_the_box():
    path = core.scan_path(PLAN, math.pi / 2, (0.0, 0.0), z=3.0, swath_m=0.82)
    perim = core.scan_path(PLAN, math.pi / 2, (0.0, 0.0), z=3.0)
    assert len(perim) == 8                       # four legs, turn + run each
    assert path[:8] == perim
    assert len(path) > 8
    assert all(-1.0 + 1.2 - 1e-6 <= x <= 7.0 - 1.2 + 1e-6
               and -0.75 + 1.2 - 1e-6 <= y <= 7.25 - 1.2 + 1e-6
               for x, y, _, _ in path)
    assert all(z == 3.0 for _, _, z, _ in path)
    assert len({round(yaw, 6) for _, _, _, yaw in path[8:]}) == 1


def test_the_scan_starts_at_the_corner_nearest_the_drone():
    x, y, _, _ = core.scan_path(PLAN, math.pi / 2, (0.0, 0.0), z=3.0)[0]
    assert (x, y) == pytest.approx((0.2, 0.45))


# ── filling the missing bases ───────────────────────────────────────────────

HOME = (0.0, 0.0)
# (x, y, top, observations, takeoff)
PADS = [
    (0.05, 0.0, 0.0, 9, True),                   # where we took off
    (2.2, 0.6, 1.5, 6, False), (3.7, 0.6, 1.5, 5, False),
    (5.2, 0.6, 1.5, 7, False),                   # the block, kits on them
    (3.5, 6.2, 0.0, 8, False), (0.5, 3.7, 0.3, 4, False),
    (4.0, 3.7, 1.5, 6, False),                   # a delivery on a stand
    (6.0, 6.0, 0.0, 1, False),                   # one look: noise
]
KITS = [(2.2, 0.62)] * 4 + [(3.68, 0.6)] * 3 + [(5.25, 0.55)] * 5


def test_kits_make_pickups_and_empty_bases_make_deliveries():
    pick, deliv, _ = core.fill_missing([None] * 3, [None] * 3, PADS, KITS,
                                       home=HOME)
    assert sorted(pick) == [(2.2, 0.6), (3.7, 0.6), (5.2, 0.6)]
    assert sorted(deliv) == [(0.5, 3.7), (3.5, 6.2), (4.0, 3.7)]


def test_given_bases_stay_and_are_not_found_twice():
    pick, deliv, _ = core.fill_missing(
        [(2.25, 0.55), None, (5.2, 0.6)], [(3.5, 6.2), None, None], PADS,
        KITS, home=HOME)
    assert pick == [(2.25, 0.55), (3.7, 0.6), (5.2, 0.6)]
    assert deliv[0] == (3.5, 6.2)
    assert sorted(deliv[1:]) == [(0.5, 3.7), (4.0, 3.7)]


def test_with_no_kit_seen_the_tall_bases_are_the_pickups():
    pads = [p for p in PADS if p[:2] != (4.0, 3.7)]
    pick, deliv, _ = core.fill_missing([None] * 3, [None, None], pads, [],
                                       home=HOME)
    assert sorted(pick) == [(2.2, 0.6), (3.7, 0.6), (5.2, 0.6)]
    assert sorted(deliv) == [(0.5, 3.7), (3.5, 6.2)]


def test_what_was_not_found_stays_missing():
    pick, deliv, _ = core.fill_missing([None] * 3, [None] * 3, PADS[:3],
                                       KITS[:4], home=HOME)
    assert pick == [(2.2, 0.6), (3.7, 0.6), None]
    assert deliv == [None, None, None]


# ── touch point ─────────────────────────────────────────────────────────────

def test_a_kit_near_the_centre_is_left_alone():
    assert core.clamp_to_disc((0.1, 0.05), (0.0, 0.0), 0.22) == (0.1, 0.05)


def test_a_kit_near_the_edge_is_pulled_in_along_the_radius():
    x, y = core.clamp_to_disc((1.3, 2.0), (1.0, 2.0), 0.22)
    assert (x, y) == pytest.approx((1.22, 2.0))


# ── kit alignment ───────────────────────────────────────────────────────────

SIM_AXES = (-1.0, -1.0)     # image +u = body right, +v = body back


def test_a_kit_already_nose_to_tail_needs_no_turn():
    # nose to tail = vertical in the image
    assert core.turn_to_axis(math.pi / 2, SIM_AXES) == pytest.approx(0.0)
    assert core.turn_to_axis(-math.pi / 2, SIM_AXES) == pytest.approx(0.0)


def test_a_kit_across_turns_a_quarter():
    assert abs(core.turn_to_axis(0.0, SIM_AXES)) == pytest.approx(math.pi / 2)


def test_a_diagonal_kit_turns_the_short_way():
    # image axis +u+v: right and back in the body = 45 deg off the nose
    d = core.turn_to_axis(math.pi / 4, SIM_AXES)
    assert abs(d) == pytest.approx(math.pi / 4)
    # turning by d lays the kit along the body x: the residual is zero
    bx, by = -math.sin(math.pi / 4), -math.cos(math.pi / 4)
    rest = core.wrap_half_pi(math.atan2(by, bx) - d)
    assert rest == pytest.approx(0.0, abs=1e-9)


def test_the_gripper_axis_shifts_the_target():
    d = core.turn_to_axis(math.pi / 2, SIM_AXES, gripper_axis=math.pi / 2)
    assert abs(d) == pytest.approx(math.pi / 2)


def test_axis_mean_does_not_average_across_the_wrap():
    a = core.mean_axis([math.radians(89), math.radians(-89)])
    assert abs(core.wrap_half_pi(a - math.pi / 2)) < math.radians(1)


# ── the pilot latch ─────────────────────────────────────────────────────────

def test_our_own_flight_is_not_a_takeover():
    assert core.takeover("GOTO", True, "GUIDED", ours=True,
                         airborne=True) is None
    # LAND disarms by itself
    assert core.takeover("LAND", False, "LAND", ours=True,
                         airborne=False) is None
    assert core.takeover("ACT", False, "LAND", ours=False,
                         airborne=False) is None


def test_a_kill_in_flight_ends_the_run():
    assert core.takeover("DESCEND", False, "GUIDED", ours=True,
                         airborne=True)


def test_a_refused_takeoff_on_the_base_is_not_a_kill():
    assert core.takeover("TAKEOFF", False, "GUIDED", ours=True,
                         airborne=False) is None
    assert core.takeover("TAKEOFF", False, "GUIDED", ours=True,
                         airborne=True)


def test_a_mode_switch_in_flight_ends_the_run():
    assert core.takeover("TOUCH", True, "LOITER", ours=True, airborne=True)
    assert core.takeover("GOTO", True, "LAND", ours=True, airborne=True)


def test_someone_touching_the_drone_on_a_base_ends_the_run():
    assert core.takeover("ACT", True, "LAND", ours=False, airborne=False)
    assert core.takeover("ACT", False, "STABILIZE", ours=False,
                         airborne=False)
    assert core.takeover("WAIT", True, "STABILIZE", ours=False,
                         airborne=False)
    assert core.takeover("WAIT", False, "STABILIZE", ours=False,
                         airborne=False) is None
