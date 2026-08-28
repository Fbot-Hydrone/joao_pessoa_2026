"""Where to look next, so the arena stops having blind spots.

The map is a callable, so these run against arenas written as strings — no
ROS, no octomap, no simulator.

    python3 -m pytest src/hydrone_nav/test/test_coverage.py -q
"""

import math

import pytest

from hydrone_nav.coverage import (next_viewpoint, trip_cost, unobserved_cells,
                                  visible_from)

RES = 1.0
Z = 1.0


def arena(rows):
    """'#' occupied, '.' free, '?' unknown. Row 0 is y=0, growing downward."""
    def occupancy(p):
        x, y, _ = p
        col = int(round(x / RES))
        row = int(round(y / RES))
        if row < 0 or row >= len(rows) or col < 0 or col >= len(rows[row]):
            return "occupied"
        return {"#": "occupied", ".": "free", "?": "unknown"}[rows[row][col]]
    return occupancy


BOUNDS = ((0.0, 0.0, 0.0), (5.0, 3.0, 2.0))


# ── what the octomap contributes ────────────────────────────────────────────

def test_unobserved_cells_are_the_ones_no_ray_reached():
    """The one thing an occupancy map knows that a point cloud cannot: the
    difference between 'I looked and it is empty' and 'I never looked'."""
    occ = arena(["...??",
                 "....."])
    cells = unobserved_cells(occ, BOUNDS, RES, Z)
    assert set(cells) == {(3.0, 0.0), (4.0, 0.0)}


def test_a_fully_observed_arena_has_nothing_left_to_see():
    occ = arena([".....", "....."])
    assert unobserved_cells(occ, BOUNDS, RES, Z) == []


# ── visibility ──────────────────────────────────────────────────────────────

def test_a_cell_out_of_range_is_not_covered():
    """Counting a base at the far edge of the frame as seen would let the
    search tick off arena it only technically looked at."""
    occ = arena(["....?"])
    cells = [(4.0, 0.0)]
    assert visible_from(occ, (0.0, 0.0), cells, Z, sensor_range=2.0) == []
    assert visible_from(occ, (0.0, 0.0), cells, Z, sensor_range=9.0) == cells


def test_a_cell_behind_a_wall_is_not_covered():
    """Standing on the other side of the house does not count as looking
    behind it."""
    occ = arena(["..#.?"])
    cells = [(4.0, 0.0)]
    assert visible_from(occ, (0.0, 0.0), cells, Z, resolution=RES) == []


def test_unknown_space_does_not_block_the_view():
    """The cells being counted are themselves unknown. If unknown were opaque
    every one of them would be invisible and every viewpoint would score
    zero."""
    occ = arena(["..??"])
    cells = [(3.0, 0.0)]
    assert visible_from(occ, (0.0, 0.0), cells, Z, resolution=RES) == cells


# ── the cost that stands in for drift ───────────────────────────────────────

def test_turning_costs_more_than_flying_straight():
    """MEASURED on this stack: the 60 deg yaw jumps all happened while turning
    with path_len frozen. Turning is where the odometry breaks, so the search
    has to be told that, not just how far things are."""
    ahead = trip_cost((0.0, 0.0), (2.0, 0.0), yaw=0.0)
    behind = trip_cost((0.0, 0.0), (-2.0, 0.0), yaw=0.0)
    assert behind > ahead


def test_a_turn_is_measured_the_short_way_round():
    a = trip_cost((0.0, 0.0), (-1.0, 0.0), yaw=math.pi)     # already facing it
    b = trip_cost((0.0, 0.0), (1.0, 0.0), yaw=math.pi)      # 180 deg away
    assert a < b


def test_standing_still_costs_nothing():
    assert trip_cost((1.0, 1.0), (1.0, 1.0), yaw=0.0) == 0.0


# ── the choice ──────────────────────────────────────────────────────────────

def test_it_goes_where_the_unseen_arena_is():
    """Unknown space OUT OF RANGE from here is what makes a trip necessary.
    Unknown space already within reach needs a turn, not a flight — and the
    scorer saying "stay put" for that case is correct, not a miss: ROTATE
    covers what is visible from where the vehicle already is."""
    occ = arena(["......",
                 "...???",
                 "......"])
    got = next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                         cell_m=RES, viewpoint_m=RES, min_gain=1,
                         sensor_range=1.5)
    assert got is not None
    (x, y), gain = got
    assert gain >= 1
    assert x >= 2.0, "picked a viewpoint that sees nothing new"


def test_unknown_space_already_in_view_does_not_earn_a_flight():
    """The counterpart, and the reason the mission still rotates: if the
    unseen arena is already in range and unoccluded, the cheapest viewpoint is
    the one the vehicle is standing on."""
    occ = arena(["......",
                 "...???",
                 "......"])
    got = next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                         cell_m=RES, viewpoint_m=RES, min_gain=1,
                         sensor_range=9.0)
    assert got is not None
    (x, _), _ = got
    assert x <= 1.0, "flew somewhere to see what it could already see"


def test_a_covered_arena_returns_none():
    """Not 'no turns left' — genuinely nothing worth flying to. That is what
    tells the mission the search is FINISHED rather than merely exhausted."""
    occ = arena(["......", "......"])
    assert next_viewpoint(occ, frm=(0.0, 0.0), yaw=0.0, bounds=BOUNDS, z=Z,
                          cell_m=RES, viewpoint_m=RES) is None


def test_a_trivial_gain_is_not_worth_the_drift():
    """One new cell does not pay for the flight that reveals it."""
    occ = arena(["....?",
                 "....."])
    assert next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                          cell_m=RES, viewpoint_m=RES, min_gain=4) is None


def test_it_never_stands_where_the_map_does_not_say_free():
    """Flying to a viewpoint chosen because nothing was KNOWN about it is how
    a search walks into a wall."""
    occ = arena(["##??",
                 "....."])
    got = next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                         cell_m=RES, viewpoint_m=RES, min_gain=1)
    if got is not None:
        (x, y), _ = got
        assert occ((x, y, Z)) == "free"


def test_the_cheaper_of_two_equal_viewpoints_wins():
    """Same arena revealed either way; the one that costs less turning is the
    one that costs less drift."""
    occ = arena(["?....?"])
    # Facing +x, so the right-hand viewpoint needs no turn and the left one
    # needs a full reversal.
    got = next_viewpoint(occ, frm=(2.5, 0.0), yaw=0.0, bounds=BOUNDS, z=Z,
                         cell_m=RES, viewpoint_m=RES, min_gain=1)
    assert got is not None
    (x, _), _ = got
    assert x >= 2.5, "chose the viewpoint behind it over the one ahead"


def test_a_viewpoint_that_could_not_be_reached_is_not_offered_again():
    """Without this the search loops: the best viewpoint is still the best one
    after the trip to it failed, so it is chosen again, and the mission
    alternates between a full sweep of turns and the same failed flight."""
    occ = arena(["......",
                 "...???",
                 "......"])
    first = next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                           cell_m=RES, viewpoint_m=RES, min_gain=1,
                           sensor_range=1.5)
    assert first is not None
    again = next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                           cell_m=RES, viewpoint_m=RES, min_gain=1,
                           sensor_range=1.5, avoid=[first[0]])
    assert again is None or again[0] != first[0]


def test_avoiding_everything_reachable_ends_the_search():
    occ = arena(["......",
                 "...???",
                 "......"])
    avoid = [(float(x), float(y)) for x in range(6) for y in range(3)]
    assert next_viewpoint(occ, frm=(0.0, 1.0), yaw=0.0, bounds=BOUNDS, z=Z,
                          cell_m=RES, viewpoint_m=RES, min_gain=1,
                          sensor_range=1.5, avoid=avoid) is None


# ── the rectangular sweep ────────────────────────────────────────────────────
#
# Standing still and turning is the wrong shape. MEASURED 2026-08-28: asked
# which directions had unobserved arena behind them, an open arena answers
# "all of them" — the directed sweep came back 22, -22, 68, -68, 112, -112,
# 158, -158, a full circle with the turns merely reordered.

def test_the_circuit_stays_inside_the_arena():
    from hydrone_nav.coverage import rectangle_survey
    pts = rectangle_survey(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.0)), inset_m=1.0)
    assert pts
    for x, y, _, _ in pts:
        assert -3.0 - 1e-9 <= x <= 3.0 + 1e-9
        assert -3.0 - 1e-9 <= y <= 3.0 + 1e-9


def test_the_camera_looks_at_the_arena_not_at_the_wall():
    """Aimed outward this would spend the whole flight photographing the four
    walls, which is the complaint that started it."""
    from hydrone_nav.coverage import rectangle_survey
    pts = rectangle_survey(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.0)), inset_m=1.0)
    for x, y, _, yaw in pts:
        to_centre = math.atan2(-y, -x)
        err = abs((yaw - to_centre + math.pi) % (2 * math.pi) - math.pi)
        assert err < 1e-6, f"({x}, {y}) looks away from the arena"


def test_it_visits_all_four_sides():
    from hydrone_nav.coverage import rectangle_survey
    pts = rectangle_survey(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.0)), inset_m=1.0,
                           step_m=2.0)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert min(xs) == pytest.approx(-3.0)
    assert max(xs) == pytest.approx(3.0)
    assert min(ys) == pytest.approx(-3.0)
    assert max(ys) == pytest.approx(3.0)


def test_a_finer_step_puts_more_points_on_each_edge():
    """The yaw is re-aimed as the vehicle travels, not only at the corners."""
    from hydrone_nav.coverage import rectangle_survey
    b = ((-4.0, -4.0, 0.0), (4.0, 4.0, 2.0))
    assert len(rectangle_survey(b, step_m=1.0)) > len(
        rectangle_survey(b, step_m=3.0))


def test_an_arena_smaller_than_the_inset_yields_no_circuit():
    """Better an empty plan than a rectangle turned inside out."""
    from hydrone_nav.coverage import rectangle_survey
    assert rectangle_survey(((-1.0, -1.0, 0.0), (1.0, 1.0, 2.0)),
                            inset_m=2.0) == []


def test_the_altitude_is_carried_through():
    from hydrone_nav.coverage import rectangle_survey
    pts = rectangle_survey(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.0)), z=1.4)
    assert all(p[2] == pytest.approx(1.4) for p in pts)
