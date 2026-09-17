import numpy as np

from hydrone_mission.phase4_maze_node import astar, closest_reachable, inflate, nearest_free


def test_astar_goes_around_a_wall():
    grid = np.zeros((10, 10), dtype=bool)
    grid[5, 0:9] = True          # wall with a gap at column 9
    path = astar(grid, (0, 0), (9, 0))
    assert path is not None and path[0] == (0, 0) and path[-1] == (9, 0)
    assert all(not grid[c] for c in path)
    assert any(c[1] == 9 for c in path)


def test_astar_no_path():
    grid = np.zeros((6, 6), dtype=bool)
    grid[3, :] = True
    assert astar(grid, (0, 0), (5, 5)) is None


def test_no_corner_cutting():
    grid = np.zeros((3, 3), dtype=bool)
    grid[0, 1] = grid[1, 0] = True
    assert astar(grid, (0, 0), (1, 1)) is None


def test_inflate_disc():
    occ = np.zeros((9, 9), dtype=bool)
    occ[4, 4] = True
    big = inflate(occ, 2)
    assert big[4, 6] and big[6, 4] and not big[6, 6]


def test_nearest_free():
    grid = np.ones((5, 5), dtype=bool)
    grid[4, 4] = False
    assert nearest_free(grid, (0, 0)) == (4, 4)
    assert nearest_free(np.zeros((3, 3), dtype=bool), (1, 1)) == (1, 1)


def test_closest_reachable_stops_at_the_wall():
    grid = np.zeros((10, 10), dtype=bool)
    grid[6, :] = True
    assert closest_reachable(grid, (0, 5), (9, 5)) == (5, 5)
