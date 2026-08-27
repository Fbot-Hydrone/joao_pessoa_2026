"""A* over an occupancy map.

The map is a callable, so these run against mazes written as strings — no ROS,
no octomap, no simulator, and a full run in well under a second.

    python3 -m pytest src/hydrone_nav/test/test_planner.py -q
"""

import math

import pytest

from hydrone_nav.planner import Occupancy, plan, simplify

RES = 0.5


def grid_map(rows, *, z_free=(0.0,)):
    """An occupancy callable from an ASCII floor plan.

    '#' occupied, '.' free, '?' unknown. Row 0 is y=0 and grows downward, so
    the picture reads the way it is written. Any z outside `z_free` is
    occupied, which stands in for the floor and the arena's net.
    """
    def occupancy(p):
        x, y, z = p
        if not any(abs(z - zf) < RES / 2 for zf in z_free):
            return Occupancy.OCCUPIED
        col = int(round(x / RES))
        row = int(round(y / RES))
        if row < 0 or row >= len(rows) or col < 0 or col >= len(rows[row]):
            return Occupancy.OCCUPIED
        c = rows[row][col]
        return {"#": Occupancy.OCCUPIED,
                ".": Occupancy.FREE,
                "?": Occupancy.UNKNOWN}[c]
    return occupancy


def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


# ── the basics ──────────────────────────────────────────────────────────────

def test_an_open_room_gives_the_straight_line():
    occ = grid_map(["....",
                    "....",
                    "...."])
    path = plan(occ, (0.0, 0.0, 0.0), (1.5, 1.0, 0.0), resolution=RES)
    assert path is not None
    assert length(path) == pytest.approx(math.dist((0, 0, 0), (1.5, 1.0, 0)),
                                         abs=0.4)


def test_the_path_starts_at_the_start_and_ends_at_the_goal():
    occ = grid_map(["....", "....", "...."])
    path = plan(occ, (0.0, 0.0, 0.0), (1.5, 1.0, 0.0), resolution=RES)
    assert path[0] == pytest.approx((0.0, 0.0, 0.0))
    assert math.dist(path[-1], (1.5, 1.0, 0.0)) < RES


def test_it_goes_around_a_wall_instead_of_through_it():
    occ = grid_map(["..#..",
                    "..#..",
                    "....."])
    path = plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES)
    assert path is not None
    for p in path:
        assert occ(p) == Occupancy.FREE, f"routed through {p}"


def test_a_sealed_goal_is_none_and_not_a_path_through_the_wall():
    occ = grid_map(["..#..",
                    "..#..",
                    "..#.."])
    assert plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES) is None


def test_planning_out_of_a_wall_is_refused():
    """A start that is not itself free means the pose is wrong or the map is.
    Planning anyway would produce a confident path from a place the drone
    is not."""
    occ = grid_map(["#....", "....."])
    assert plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES) is None


def test_planning_into_a_wall_is_refused():
    occ = grid_map([".....", "....#"])
    assert plan(occ, (0.0, 0.0, 0.0), (2.0, 0.5, 0.0), resolution=RES) is None


def test_start_equals_goal_is_a_one_point_path_not_none():
    occ = grid_map(["....."])
    path = plan(occ, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), resolution=RES)
    assert path == [(0.0, 0.0, 0.0)]


# ── unknown space ───────────────────────────────────────────────────────────

def test_unknown_space_is_not_flown_through_by_default():
    """The default that keeps a drone alive: 'never measured' is not 'empty'.
    Phase 4's map is mostly unknown early on."""
    occ = grid_map(["..?..",
                    "..?..",
                    "..?.."])
    assert plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES) is None


def test_unknown_space_can_be_opened_up_explicitly():
    """A frontier explorer needs the opposite behaviour, and has to ask."""
    occ = grid_map(["..?..",
                    "..?..",
                    "..?.."])
    path = plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES,
                allow_unknown=True)
    assert path is not None


# ── shape of the answer ─────────────────────────────────────────────────────

def test_bounds_keep_the_search_inside_the_arena():
    """Without bounds a search that cannot reach the goal expands outwards
    through open space until the expansion cap stops it."""
    occ = lambda p: Occupancy.FREE
    path = plan(occ, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), resolution=RES,
                bounds=((-0.1, -0.1, -0.1), (2.0, 2.0, 2.0)))
    assert path is not None
    for p in path:
        assert -0.1 <= p[0] <= 2.0


def test_a_goal_outside_the_bounds_is_none():
    occ = lambda p: Occupancy.FREE
    assert plan(occ, (0.0, 0.0, 0.0), (5.0, 0.0, 0.0), resolution=RES,
                bounds=((-1, -1, -1), (2, 2, 2))) is None


def test_climbing_is_costed_higher_than_going_around():
    """Vertical motion burns battery, tips the camera off the features the VO
    tracks, and runs out of room first under a net. It stays possible."""
    occ = grid_map(["..#..",
                    "....."], z_free=(0.0, 0.5))
    path = plan(occ, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), resolution=RES)
    assert path is not None
    assert all(abs(p[2]) < 1e-6 for p in path), "climbed when it could go round"


def test_a_positive_resolution_is_required():
    with pytest.raises(ValueError):
        plan(lambda p: Occupancy.FREE, (0, 0, 0), (1, 0, 0), resolution=0.0)


# ── simplify ────────────────────────────────────────────────────────────────

def test_a_straight_path_collapses_to_its_endpoints():
    """A* returns one point per cell. Each would otherwise become a setpoint
    the vehicle decelerates into."""
    path = [(float(i), 0.0, 0.0) for i in range(10)]
    assert simplify(path, lambda a, b: True) == [path[0], path[-1]]


def test_a_corner_is_kept():
    path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0),
            (2.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
    clear = lambda a, b: a[0] == b[0] or a[1] == b[1]
    assert simplify(path, clear) == [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0),
                                     (2.0, 2.0, 0.0)]


def test_simplify_leaves_a_short_path_alone():
    assert simplify([(0, 0, 0), (1, 0, 0)], lambda a, b: False) == \
        [(0, 0, 0), (1, 0, 0)]


def test_simplify_never_invents_a_shortcut_the_map_refuses():
    """Every retained segment has to have been tested clear by the same map
    that produced the path."""
    path = [(float(i), 0.0, 0.0) for i in range(5)]
    out = simplify(path, lambda a, b: math.dist(a, b) <= 1.0)
    assert out == path
