"""Relief: things standing above the floor that might be a base.

Arenas written as strings, no ROS and no octomap.

    python3 -m pytest src/hydrone_map/test/test_relief.py -q
"""

import pytest

from hydrone_map.relief import cluster, occupied_cells, relief_candidates

RES = 0.5
BOUNDS = ((0.0, 0.0, 0.0), (5.0, 5.0, 2.0))


def arena(rows, *, occupied_z=(0.75,)):
    """'#' something standing here, '.' clear. Row 0 is y=0, growing down."""
    def occupancy(p):
        x, y, z = p
        if not any(abs(z - zz) < RES / 2 for zz in occupied_z):
            return "free"
        col = int(round(x / RES))
        row = int(round(y / RES))
        if row < 0 or row >= len(rows) or col < 0 or col >= len(rows[row]):
            return "free"
        return "occupied" if rows[row][col] == "#" else "free"
    return occupancy


# ── the band ────────────────────────────────────────────────────────────────

def test_the_floor_itself_is_not_relief():
    """Everything would be a candidate otherwise."""
    occ = arena(["###", "###"], occupied_z=(0.0,))
    assert occupied_cells(occ, BOUNDS, RES) == set()


def test_something_standing_in_the_band_is_found():
    occ = arena(["...", ".#.", "..."], occupied_z=(0.75,))
    assert len(occupied_cells(occ, BOUNDS, RES)) == 1


def test_a_base_low_down_and_a_base_high_up_both_count():
    """Competition bases sit anywhere from 0 to 1.5 m. Scanning the column is
    what makes the height irrelevant to the question being asked — and an
    elevated base is exactly the one the ground-plane projection places
    wrongly, so it is the one this has to catch."""
    low = arena(["...", ".#.", "..."], occupied_z=(0.25,))
    high = arena(["...", ".#.", "..."], occupied_z=(1.25,))
    assert len(occupied_cells(low, BOUNDS, RES)) == 1
    assert len(occupied_cells(high, BOUNDS, RES)) == 1


def test_an_excluded_region_is_ignored():
    """The house is known, big, and would otherwise dominate every cluster."""
    occ = arena(["##.", "##.", "..."])
    assert occupied_cells(occ, BOUNDS, RES,
                          exclude=[(-1.0, -1.0, 1.0, 1.0)]) == set()


# ── clustering ──────────────────────────────────────────────────────────────

def test_touching_cells_are_one_thing():
    assert len(cluster({(0, 0), (0, 1), (1, 1)})) == 1


def test_separate_cells_are_separate_things():
    assert len(cluster({(0, 0), (5, 5)})) == 2


def test_diagonal_counts_as_touching():
    assert len(cluster({(0, 0), (1, 1)})) == 1


# ── what survives as a candidate ────────────────────────────────────────────

def test_a_base_sized_lump_in_open_floor_is_a_candidate():
    occ = arena(["......",
                 "......",
                 "..##..",
                 "..##..",
                 "......",
                 "......"])
    got = relief_candidates(occ, BOUNDS, RES)
    assert len(got) == 1
    cx, cy = got[0]
    assert cx == pytest.approx(1.25, abs=0.3)
    assert cy == pytest.approx(1.25, abs=0.3)


def test_a_single_stray_cell_is_noise_not_a_base():
    occ = arena(["......", "......", "...#..", "......", "......", "......"])
    assert relief_candidates(occ, BOUNDS, RES) == []


def test_something_far_too_wide_is_not_a_base():
    """The house, a wall, or two things the flood fill joined."""
    occ = arena(["......",
                 "######",
                 "######",
                 "......",
                 "......",
                 "......"])
    assert relief_candidates(occ, BOUNDS, RES) == []


def test_relief_against_the_arena_edge_is_the_wall():
    """The margin is what separates 'a base near the wall' from 'the wall'."""
    occ = arena(["##....",
                 "##....",
                 "......",
                 "......",
                 "......",
                 "......"])
    assert relief_candidates(occ, BOUNDS, RES) == []


def test_the_house_is_excluded_and_a_base_beside_it_is_not():
    occ = arena(["......",
                 ".###..",
                 ".###..",
                 "......",
                 "...##.",
                 "...##."])
    got = relief_candidates(occ, BOUNDS, RES,
                            exclude=[(0.0, 0.0, 2.0, 1.5)])
    assert len(got) == 1
    assert got[0][1] > 1.5, "kept the house instead of the base beside it"


def test_an_empty_arena_yields_nothing():
    assert relief_candidates(arena(["......"] * 6), BOUNDS, RES) == []
