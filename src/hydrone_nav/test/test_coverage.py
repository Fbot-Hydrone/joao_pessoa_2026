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


# ── the lateral sweep ────────────────────────────────────────────────────────
#
# Two straight passes, fixed heading in each. The rectangle it replaced
# re-aimed the camera at the arena centre at every step, so it turned
# CONTINUOUSLY along every edge — which is what "he is still spinning" meant.

def test_the_heading_never_changes_during_a_pass():
    """The whole point. A fixed heading gives the detector a stable scene, the
    depth camera a clean band, and never asks the odometry to do the one thing
    this arena breaks it on."""
    from hydrone_nav.coverage import lateral_sweep
    pts = lateral_sweep(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5)), inset_m=1.0)
    yaws = [p[3] for p in pts]
    changes = sum(1 for a, b in zip(yaws, yaws[1:]) if abs(a - b) > 1e-9)
    assert changes == 1, f"turned {changes} times; a two-pass sweep turns once"


def test_the_two_passes_look_in_opposite_directions():
    """A pad that is edge-on or back-lit on one pass is face-on on the other."""
    from hydrone_nav.coverage import lateral_sweep
    pts = lateral_sweep(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5)), inset_m=1.0)
    yaws = sorted({round(p[3], 6) for p in pts})
    assert len(yaws) == 2
    assert abs(abs(yaws[1] - yaws[0]) - math.pi) < 1e-6


def test_each_pass_looks_ACROSS_the_arena_not_along_it():
    from hydrone_nav.coverage import lateral_sweep
    b = ((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5))
    for x, y, _, yaw in lateral_sweep(b, inset_m=1.0):
        # the ray from (x, y) along yaw must head towards y = 0
        assert (y > 0 and math.sin(yaw) < 0) or (y < 0 and math.sin(yaw) > 0)


def test_the_passes_run_along_opposite_sides():
    from hydrone_nav.coverage import lateral_sweep
    pts = lateral_sweep(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5)), inset_m=1.0)
    ys = sorted({round(p[1], 6) for p in pts})
    assert ys == [-3.0, 3.0]


def test_the_sweep_altitude_is_explicit_and_not_the_cruise_height():
    """The top pass runs over the house, whose roof is at 1.5 m. A sweep at the
    usual 1 m cruise would fly into it."""
    from hydrone_nav.coverage import lateral_sweep
    pts = lateral_sweep(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5)), z=2.0)
    assert all(p[2] == pytest.approx(2.0) for p in pts)


def test_an_arena_smaller_than_the_inset_yields_no_sweep():
    from hydrone_nav.coverage import lateral_sweep
    assert lateral_sweep(((-1.0, -1.0, 0.0), (1.0, 1.0, 2.0)),
                         inset_m=2.0) == []


# ── LEVEL 1: the U ───────────────────────────────────────────────────────────
#
# Take off, run one edge without rotating at all, 90 degrees at the corner, run
# the next, 90 degrees again, run the third. Two turns in the whole sweep, both
# at corners, camera facing into the arena throughout.

B8 = ((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5))


def test_the_u_turns_only_at_its_two_corners():
    """Every earlier shape failed on rotation: spinning has no parallax, and a
    circuit that re-aims at the centre turns continuously along every edge."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    yaws = [p[3] for p in pts]
    turns = sum(1 for a, b in zip(yaws, yaws[1:]) if abs(a - b) > 1e-9)
    assert turns == 2, f"turned {turns} times; the U turns twice"


def test_each_leg_of_the_u_turns_ninety_degrees():
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    yaws = [p[3] for p in pts]
    for a, b in zip(yaws, yaws[1:]):
        d = abs((b - a + math.pi) % (2 * math.pi) - math.pi)
        assert d < 1e-9 or abs(d - math.pi / 2) < 1e-6


def test_the_camera_faces_into_the_arena_on_every_leg():
    """Aimed outward the sweep photographs the four walls."""
    from hydrone_nav.coverage import u_sweep
    for x, y, _, yaw in u_sweep(B8, inset_m=1.0):
        to_centre = math.atan2(-y, -x)
        err = abs((yaw - to_centre + math.pi) % (2 * math.pi) - math.pi)
        assert err < math.pi / 2, f"({x:.1f}, {y:.1f}) looks out of the arena"


def test_the_u_covers_three_sides_not_four():
    """From the third edge the camera already looks back across what a fourth
    would cover, so the fourth is battery spent re-photographing the map."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    sides = {round(p[3], 6) for p in pts}
    assert len(sides) == 3


def test_the_u_starts_at_the_corner_it_is_told_to():
    """It should be the one nearest where the drone took off, so the sweep
    starts without a transit leg."""
    from hydrone_nav.coverage import u_sweep
    for k in range(4):
        pts = u_sweep(B8, inset_m=1.0, start_corner=k)
        corners = [(-3.0, -3.0), (3.0, -3.0), (3.0, 3.0), (-3.0, 3.0)]
        assert pts[0][:2] == pytest.approx(corners[k])


def test_the_u_stays_inside_the_arena():
    from hydrone_nav.coverage import u_sweep
    for x, y, _, _ in u_sweep(B8, inset_m=1.0):
        assert -3.0 - 1e-9 <= x <= 3.0 + 1e-9
        assert -3.0 - 1e-9 <= y <= 3.0 + 1e-9


def test_the_u_carries_its_altitude():
    """LEVEL 2 is the same shape half a metre higher, so this has to be a
    parameter and not a constant."""
    from hydrone_nav.coverage import u_sweep
    assert all(p[2] == pytest.approx(2.5) for p in u_sweep(B8, z=2.5))


# ── LAST RESORT: the lawnmower ───────────────────────────────────────────────

def test_the_lawnmower_lays_parallel_lanes():
    from hydrone_nav.coverage import lawnmower
    pts = lawnmower(B8, inset_m=1.0, lane_m=1.5)
    lanes = sorted({round(p[1], 6) for p in pts})
    assert len(lanes) >= 4
    gaps = [b - a for a, b in zip(lanes, lanes[1:])]
    assert all(g == pytest.approx(1.5) for g in gaps)


def test_the_lawnmower_alternates_direction():
    """Boustrophedon: it does not fly back to the start of every lane."""
    from hydrone_nav.coverage import lawnmower
    pts = lawnmower(B8, inset_m=1.0, lane_m=1.5)
    yaws = sorted({round(p[3], 6) for p in pts})
    assert len(yaws) == 2
    assert abs(abs(yaws[1] - yaws[0]) - math.pi) < 1e-6


def test_the_lawnmower_costs_more_than_the_u_and_more_as_it_tightens():
    """Which is exactly why it is the last level and not the first. At the
    lane spacing that actually guarantees coverage it is several times the
    flight."""
    from hydrone_nav.coverage import lawnmower, u_sweep
    u = len(u_sweep(B8, inset_m=1.0))
    assert len(lawnmower(B8, inset_m=1.0, lane_m=1.5)) > u
    assert len(lawnmower(B8, inset_m=1.0, lane_m=0.8)) > 2 * u


def test_a_finer_lane_spacing_covers_more_thoroughly():
    from hydrone_nav.coverage import lawnmower
    assert len(lawnmower(B8, lane_m=1.0)) > len(lawnmower(B8, lane_m=2.0))


def test_neither_shape_survives_an_arena_smaller_than_the_inset():
    from hydrone_nav.coverage import lawnmower, u_sweep
    tiny = ((-1.0, -1.0, 0.0), (1.0, 1.0, 2.0))
    assert u_sweep(tiny, inset_m=2.0) == []
    assert lawnmower(tiny, inset_m=2.0) == []


def test_the_u_never_translates_and_turns_at_the_same_time():
    """Yawing under translation is where this arena's odometry loses the most:
    the camera sweeps, matching fails, and the rotation that really happened is
    never recorded. Every heading change must happen at a standstill."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    for a, b in zip(pts, pts[1:]):
        moved = (a[0], a[1]) != (b[0], b[1])
        turned = abs((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi) > 1e-9
        assert not (moved and turned), \
            f"moved and turned together at {a[:2]} -> {b[:2]}"


def test_each_corner_is_visited_twice_to_stop_before_turning():
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    seen = {}
    for x, y, _, _ in pts:
        seen[(round(x, 6), round(y, 6))] = seen.get((round(x, 6), round(y, 6)), 0) + 1
    assert sum(1 for n in seen.values() if n > 1) == 2, \
        "the two corners must each be held twice: arrive, then turn"


def test_the_u_has_no_intermediate_points_on_a_leg():
    """A leg is a straight line flown on one heading, and GUIDED stops dead at
    every position target — so a point in the middle only costs a full
    decelerate-and-accelerate. MEASURED with WP_SPD 1.5 and WP_ACC 1.5: at the
    old 1.5 m spacing the vehicle NEVER REACHED CRUISE anywhere in the sweep,
    averaging 0.75 m/s against an airframe capable of twice that."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0)
    assert len(pts) == 6, "three legs, two setpoints each"
    assert len({p[:2] for p in pts}) == 4, "a leg has a point in the middle"


def test_the_u_never_translates_and_turns_at_the_same_time():
    """Yawing under translation is where this arena's odometry loses the most:
    the camera sweeps, matching fails, and the rotation that really happened is
    never recorded."""
    from hydrone_nav.coverage import u_sweep
    for a, b in zip(u_sweep(B8, inset_m=1.0), u_sweep(B8, inset_m=1.0)[1:]):
        moved = (a[0], a[1]) != (b[0], b[1])
        turned = abs((b[3] - a[3] + math.pi) % (2 * math.pi) - math.pi) > 1e-9
        assert not (moved and turned), f"moved and turned at once {a[:2]}"


# ── The leg length, stated outright ──────────────────────────────────────────

def test_the_side_can_be_given_in_metres():
    """Set it when the sweep should be a particular size for a reason the
    arena dimensions do not express."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, side_x_m=4.0, side_y_m=3.0)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    assert max(xs) - min(xs) == pytest.approx(4.0)
    assert max(ys) - min(ys) == pytest.approx(3.0)


def test_a_stated_side_is_centred_in_the_arena():
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(((-4.0, -4.0, 0.0), (4.0, 4.0, 2.5)), side_x_m=4.0,
                  side_y_m=4.0)
    xs = [p[0] for p in pts]
    assert min(xs) == pytest.approx(-2.0)
    assert max(xs) == pytest.approx(2.0)


def test_zero_falls_back_to_the_inset():
    """leg = arena_size - 2 * inset, which is what it always did."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0, side_x_m=0.0, side_y_m=0.0)
    xs = [p[0] for p in pts]
    assert max(xs) - min(xs) == pytest.approx(8.0 - 2 * 1.0)


def test_the_two_axes_are_independent():
    """The competition arena is 8 x 8 and the team's own is 5 x 6."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, inset_m=1.0, side_x_m=3.0)     # y still derived
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    assert max(xs) - min(xs) == pytest.approx(3.0)
    assert max(ys) - min(ys) == pytest.approx(6.0)


def test_a_side_longer_than_the_arena_is_clamped_to_it():
    """Outside the arena is where the competition ENDS THE ATTEMPT. A typed
    number must not be able to fly the drone over the line."""
    from hydrone_nav.coverage import u_sweep
    pts = u_sweep(B8, side_x_m=40.0, side_y_m=40.0)
    for x, y, _, _ in pts:
        assert -4.0 - 1e-9 <= x <= 4.0 + 1e-9
        assert -4.0 - 1e-9 <= y <= 4.0 + 1e-9


# ── The map-sweep shapes ────────────────────────────────────────────────────
# A different strategy: the forward camera maps and the BELLY camera detects.
# What has to hold is that the perimeter closes (its product is the map, so the
# fourth side is not spare) and that the lane pitch comes from the camera
# rather than from a constant.


def test_the_perimeter_closes_where_it_started():
    """The U's fourth side is spare because the third looks back across it.
    This pass builds a map out of a depth camera's own band, so it is not —
    and closing the loop puts the vehicle back over the takeoff base."""
    from hydrone_nav.coverage import perimeter_sweep
    pts = perimeter_sweep(B8, inset_m=1.0, z=2.5, start_corner=0)
    assert pts[0][:2] == pytest.approx(pts[-1][:2])
    corners = {(round(x, 6), round(y, 6)) for x, y, _, _ in pts}
    assert len(corners) == 4, "four corners, each visited once"


def test_the_perimeter_flies_one_setpoint_per_leg():
    """A mid-leg setpoint only tells GUIDED to stop and start again."""
    from hydrone_nav.coverage import perimeter_sweep
    pts = perimeter_sweep(B8, inset_m=1.0, z=2.5)
    assert len(pts) == 8, "four legs, each a turn then the leg"
    for i in range(0, len(pts), 2):
        assert pts[i][3] == pytest.approx(pts[i + 1][3]), (
            "a leg is flown on ONE heading; the turn happens standing still")


def test_the_swath_is_the_camera_not_a_constant():
    """The number differs by 2.5x between the simulator and the drone, which
    is the whole reason it may not be hard-coded."""
    from hydrone_nav.coverage import ground_swath
    sim = ground_swath(320.0, 320.0, 640, 480, 2.5)      # 90 deg
    real = ground_swath(814.6, 814.6, 640, 480, 2.5)     # measured, ~43 deg
    assert sim[0] == pytest.approx(5.0)
    assert real[0] == pytest.approx(1.96, abs=0.01)
    assert ground_swath(0.0, 0.0, 640, 480, 2.5) == (0.0, 0.0)
    assert ground_swath(320.0, 320.0, 640, 480, 0.0) == (0.0, 0.0)


def test_lanes_are_spaced_by_the_swath_so_nothing_is_missed():
    """Adjacent swaths must overlap, or the gap between two lanes flown
    minutes apart is the drift accumulated in between."""
    from hydrone_nav.coverage import camera_lawnmower
    swath = 2.0
    pts = camera_lawnmower(B8, swath_m=swath, z=2.5, overlap=0.25,
                           margin_m=0.5)
    lanes = sorted({round(p[1], 6) for p in pts})
    assert len(lanes) > 1
    for a, b in zip(lanes, lanes[1:]):
        assert b - a < swath, "a gap wider than the swath is unseen floor"


def test_a_narrower_camera_is_given_more_lanes():
    """The real belly camera sees a third of what the simulated one does."""
    from hydrone_nav.coverage import camera_lawnmower
    wide = camera_lawnmower(B8, swath_m=3.75, z=2.5)
    narrow = camera_lawnmower(B8, swath_m=1.47, z=2.5)
    assert len(narrow) > len(wide)


def test_the_swept_band_reaches_both_walls():
    """The first lane centre sits half a swath in, not `inset_m` in: what has
    to stay inside the arena is the STRIP the camera sees, and its centre is
    the vehicle."""
    from hydrone_nav.coverage import camera_lawnmower
    swath = 2.0
    pts = camera_lawnmower(B8, swath_m=swath, z=2.5, margin_m=0.5)
    lanes = sorted({round(p[1], 6) for p in pts})
    assert lanes[0] - swath / 2.0 <= -4.0 + 1e-6
    assert lanes[-1] + swath / 2.0 >= 4.0 - 1e-6


def test_a_camera_wider_than_the_arena_flies_one_lane():
    from hydrone_nav.coverage import camera_lawnmower
    pts = camera_lawnmower(B8, swath_m=20.0, z=2.5)
    assert len({round(p[1], 6) for p in pts}) == 1


def test_lanes_stay_inside_the_arena():
    from hydrone_nav.coverage import camera_lawnmower
    for swath in (0.5, 2.0, 3.75, 9.0):
        for x, y, _, _ in camera_lawnmower(B8, swath_m=swath, z=2.5):
            assert -4.0 - 1e-9 <= x <= 4.0 + 1e-9
            assert -4.0 - 1e-9 <= y <= 4.0 + 1e-9


def test_lanes_spaced_for_the_floor_leave_a_hole_over_a_raised_surface():
    """The defect six seeds found, stated as arithmetic.

    Lane pitch comes from the camera's footprint, and a footprint is only as
    wide as the height above WHAT IS UNDER IT. Space lanes for the floor and
    the strip over a 1.5 m structure is never scanned — which is where the
    competition puts raised bases. MEASURED: bases found tracked the number on
    the house roof, monotonically, 6/6 -> 5/6 -> 3/6.
    """
    from hydrone_nav.coverage import ground_swath, lane_spacing
    cruise_above_floor = 3.2
    roof = 1.5

    over_floor = min(ground_swath(320.0, 320.0, 640, 480, cruise_above_floor))
    over_roof = min(ground_swath(320.0, 320.0, 640, 480,
                                 cruise_above_floor - roof))
    pitch = lane_spacing(over_floor)

    assert pitch > over_roof, (
        "this test no longer reproduces the hole it exists to pin")
    assert pitch - over_roof > 1.0, "the gap measured over a metre"

    # Sizing the pitch for the ROOF closes it, at the cost of more lanes.
    safe = lane_spacing(over_roof)
    assert safe < over_roof, "adjacent swaths must overlap over the roof too"
