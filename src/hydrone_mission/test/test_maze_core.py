"""Unit tests for the ROS-free maze core (hydrone_mission/maze).

Run inside the stack container:

    docker exec -u hydrone joao_pessoa_2026-hydrone-1 bash -lc 'source /opt/ros/humble/setup.bash; \
      cd /ws/src/hydrone_mission && python3 -m pytest -q -p no:cacheprovider test/test_maze_core.py'
"""
import math
import time

import numpy as np
import pytest

from hydrone_mission.maze import frontiers as fr
from hydrone_mission.maze import geometry_fallback as gf
from hydrone_mission.maze.arena import min_area_rect, robust_rect, supported_rect
from hydrone_mission.maze.grid import OccupancyGrid
from hydrone_mission.maze.mission import Mission
from hydrone_mission.maze.morph import close, dilate, fill_holes, label
from hydrone_mission.maze.openings import detect_openings, window_extent
from hydrone_mission.maze.planner import (GridPlanner, astar, cost_field, descend, inflate,
                                          nearest_free, shortcut)
from maze_sim import lidar
from maze_sim.world import build


def wall_points(x, y0, y1, z0, z1, step=0.02):
    yy, zz = np.meshgrid(np.arange(y0, y1, step), np.arange(z0, z1, step))
    return np.column_stack([np.full(yy.size, x), yy.ravel(), zz.ravel()])


def survey(world, n=20000, heights=None, seed=0, drop_tags=()):
    """Grid built from scans taken while climbing over the takeoff point."""
    rng = np.random.default_rng(seed)
    g = OccupancyGrid(flight_z=0.5)
    heights = heights if heights is not None else list(np.arange(0.0, 1.1, 0.1)) + [0.5] * 10
    ms = []
    for z in heights:
        o = world.takeoff + np.array([0, 0, z + 0.1])
        pts = lidar.scan(o, world.lo, world.hi, n, rng, drop_tags=drop_tags, tags=world.tags)
        p_odom, o_odom = world.to_odom(pts), world.to_odom(o)
        c0 = time.perf_counter()
        g.update(p_odom, o_odom)
        ms.append((time.perf_counter() - c0) * 1e3)
    return g, ms


# ── morph ───────────────────────────────────────────────────────────────────
def test_fill_holes_and_label():
    ring = np.zeros((9, 9), dtype=bool)
    ring[2, 2:7] = ring[6, 2:7] = ring[2:7, 2] = ring[2:7, 6] = True
    filled = fill_holes(ring)
    assert filled[4, 4] and not filled[0, 0]
    lab, n = label(ring | (np.arange(81).reshape(9, 9) == 80))
    assert n == 2


def test_close_bridges_a_gap_without_growing():
    m = np.zeros((30, 30), dtype=bool)
    m[8:22, 2:12] = m[8:22, 16:28] = True    # two roof patches, 4-cell gap
    c = close(m, 3)
    assert c[15, 12:16].all() and not c[5, 14] and not c[15, 0]


# ── grid ────────────────────────────────────────────────────────────────────
def test_grid_free_occupied_unknown():
    g = OccupancyGrid(flight_z=0.5, floor_z=-0.6)
    wall = wall_points(2.0, -1.0, 1.0, -0.6, 1.5)
    for _ in range(3):
        g.update(wall, np.array([0.0, 0.0, 0.6]))
    c = lambda x, y: tuple(g.to_cell([x, y]))  # noqa: E731
    assert g.free()[c(1.0, 0.0)]
    assert g.occupied()[c(2.0, 0.0)]
    assert g.unknown()[c(3.0, 0.0)]            # behind the wall
    assert g.unknown()[c(-1.0, 0.0)]           # nothing was seen there
    assert g.roof[c(2.0, 0.0)] >= 3            # wall top above the band
    assert g.low[c(2.0, 0.0)] >= 3             # vertical face in the low band


def test_grid_low_layer_ignores_horizontal_tops():
    g = OccupancyGrid(flight_z=0.5, floor_z=-0.6)
    xx, yy = np.meshgrid(np.arange(1.0, 2.0, 0.02), np.arange(-0.5, 0.5, 0.02))
    top = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -0.1)])   # a pad top
    g.update(top, np.array([0.0, 0.0, 0.1]))
    assert g.low.sum() == 0


def test_grid_far_returns_carve_free_but_do_not_hit():
    g = OccupancyGrid(flight_z=0.5, floor_z=-0.6, hit_range=5.0, max_range=6.0)
    far = wall_points(20.0, -0.2, 0.2, 0.4, 0.6)
    g.update(far, np.array([0.0, 0.0, 0.5]))
    assert g.free()[tuple(g.to_cell([4.0, 0.0]))]
    assert not g.occupied().any()


def test_floor_estimate_is_the_low_horizontal_surface():
    w = build('nominal')
    g, _ = survey(w, n=12000, heights=[0.0, 0.2, 0.5, 0.5, 0.5])
    assert g.floor_z == pytest.approx(-0.62, abs=0.06)


def test_grid_update_is_cheap():
    w = build('nominal')
    _, ms = survey(w, n=20000, heights=[0.5] * 15)
    # target < 50 ms on a desktop core (Jetson Nano budget)
    assert np.median(ms) < 50.0


# ── planner ─────────────────────────────────────────────────────────────────
def test_astar_and_cost_field_agree():
    grid = np.zeros((20, 20), dtype=bool)
    grid[10, 0:18] = True
    p = astar(grid, (0, 0), (19, 0))
    assert p is not None and all(not grid[c] for c in p)
    cost = cost_field(grid, (0, 0))
    q = descend(cost, (19, 0), grid)
    assert q[0] == (0, 0) and q[-1] == (19, 0)
    length = lambda path: sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:]))  # noqa: E731
    assert length(q) == pytest.approx(length(p), abs=1e-6)


def test_no_path_and_no_corner_cutting():
    grid = np.zeros((6, 6), dtype=bool)
    grid[3, :] = True
    assert astar(grid, (0, 0), (5, 5)) is None
    assert not np.isfinite(cost_field(grid, (0, 0))[5, 5])
    g2 = np.zeros((3, 3), dtype=bool)
    g2[0, 1] = g2[1, 0] = True
    assert astar(g2, (0, 0), (1, 1)) is None


def test_inflate_does_not_wrap_and_shortcut_keeps_clear():
    occ = np.zeros((10, 10), dtype=bool)
    occ[0, 0] = True
    big = inflate(occ, 2)
    assert big[2, 0] and not big[9, 0] and not big[0, 9]
    grid = np.zeros((10, 10), dtype=bool)
    grid[5, 2:10] = True
    path = shortcut(grid, astar(grid, (0, 9), (9, 9)))
    assert len(path) <= 4
    assert nearest_free(np.ones((3, 3), dtype=bool), (1, 1)) is None


def test_grid_planner_world_coordinates():
    passable = np.ones((40, 40), dtype=bool)
    passable[20, 0:35] = False
    pl = GridPlanner(passable, offset=(180, 180), origin=-20.0, res=0.1)
    assert pl.start_from([-1.5, -1.5])
    path = pl.path_to([1.5, -1.5])
    assert path is not None
    assert np.allclose(path[0], [-1.45, -1.45]) and np.linalg.norm(path[-1] - [1.5, -1.5]) < 0.1
    assert max(p[1] for p in path) > 1.4          # went round the end of the wall


# ── openings ────────────────────────────────────────────────────────────────
@pytest.fixture(scope='module')
def surveyed():
    w = build('nominal', yaw0=0.7)
    g, _ = survey(w)
    return w, g


def test_openings_window_and_door(surveyed):
    w, g = surveyed
    ops, covered, _ = detect_openings(g, viewer=np.zeros(2))
    world = lambda o: w.to_world(np.r_[o.center, 0.0])[:2]  # noqa: E731
    win = [o for o in ops if np.linalg.norm(world(o) - [2.0, 1.45]) < 0.3]
    door = [o for o in ops if np.linalg.norm(world(o) - [3.7, 2.0]) < 0.35]
    assert win and win[0].kind == 'window'
    assert 0.6 <= win[0].width <= 1.0
    # inward normal points into the structure (world +x for the end wall)
    assert w.vec_to_world(np.r_[win[0].normal, 0])[0] > 0.9
    assert door and door[0].kind == 'door'
    assert w.vec_to_world(np.r_[door[0].normal, 0])[1] < -0.9
    assert covered.any()


def test_openings_without_sill_evidence_is_not_a_window():
    w = build('nominal', yaw0=0.4)
    g, _ = survey(w, drop_tags=('sill',))
    ops, _, _ = detect_openings(g, viewer=np.zeros(2))
    near = [o for o in ops if np.linalg.norm(w.to_world(np.r_[o.center, 0])[:2] - [2.0, 1.45]) < 0.3]
    assert near and near[0].kind != 'window'


def test_openings_partial_view_single_wall():
    """A roofed wall with one gap, seen from one side only."""
    g = OccupancyGrid(flight_z=0.5, floor_z=-0.6)
    rng = np.random.default_rng(3)
    # boxes in a floor-at-0 frame for the ray caster, odom = that - 0.6
    lo = np.array([[2.0, -2.0, 0.0], [2.0, 0.4, 0.0], [2.0, -0.4, 0.0], [2.0, -2.0, 1.5]])
    hi = np.array([[2.08, -0.4, 1.5], [2.08, 2.0, 1.5], [2.08, 0.4, 0.7], [4.0, 2.0, 1.55]])
    for z in (0.1, 0.3, 0.5, 0.5, 0.5):
        o = np.array([0.0, 0.0, z])
        g.update(lidar.scan(o + [0, 0, 0.6], lo, hi, 15000, rng) - [0, 0, 0.6], o)
    ops, _, _ = detect_openings(g, viewer=np.zeros(2))
    assert len(ops) == 1
    o = ops[0]
    assert np.linalg.norm(o.center - [2.04, 0.0]) < 0.15 and o.kind == 'window'
    assert o.normal[0] > 0.9


def test_window_extent_gives_the_window_centre_height(surveyed):
    w, g = surveyed
    ops, _, _ = detect_openings(g, viewer=np.zeros(2))
    win = min(ops, key=lambda o: np.linalg.norm(w.to_world(np.r_[o.center, 0])[:2] - [2.0, 1.45]))
    rng = np.random.default_rng(1)
    pts = []
    for z in (0.0, 0.2, 0.5):
        o = w.takeoff + [0, 0, z + 0.1]
        pts.append(w.to_odom(lidar.scan(o, w.lo, w.hi, 20000, rng)))
    bot, top = window_extent(np.vstack(pts), win, 0.5)
    assert bot == pytest.approx(0.08, abs=0.06)       # sill top: floor + 0.7
    assert top == pytest.approx(0.88, abs=0.08)       # roof
    assert (bot + top) / 2 == pytest.approx(0.48, abs=0.06)


# ── frontiers ───────────────────────────────────────────────────────────────
def _room_grid():
    """A 2 x 2 m room seen from inside with its +x side open into unseen space."""
    g = OccupancyGrid(flight_z=0.5, floor_z=-0.6)
    c = g.to_cell([0.0, 0.0])
    i, j = c
    g.logodds[i - 10:i + 10, j - 10:j + 10] = -1.0
    g.observed[i - 10:i + 10, j - 10:j + 10] = True
    g.logodds[i - 11, j - 11:j + 11] = 2.0         # walls on three sides
    g.logodds[i - 11:i + 11, j - 11] = 2.0
    g.logodds[i - 11:i + 11, j + 10] = 2.0
    g.observed[i - 11:i + 11, j - 11:j + 11] = True
    g.observed[i + 10:, :] = False                 # +x beyond the room: never seen
    g.logodds[i + 10:, :] = 0.0
    g.bbox = (i - 12, i + 12, j - 12, j + 12)
    return g


def _frontiers(g, pos, covered=None):
    crop = g.crop(1.0)
    i0, i1, j0, j1 = crop
    if covered is None:
        covered = np.ones((i1 - i0, j1 - j0), dtype=bool)
    passable = g.free()[i0:i1, j0:j1] & ~inflate(g.occupied()[i0:i1, j0:j1], 2.2)
    pl = GridPlanner(passable, (i0, j0), g.origin, g.p.res)
    assert pl.start_from(pos)
    return fr.find_frontiers(g, crop, covered, pl.cost, planner=pl)


def test_frontier_cluster_found_and_reachable():
    g = _room_grid()
    fs, mask = _frontiers(g, [0.0, 0.0])
    assert len(fs) == 1
    f = fs[0]
    assert f.centroid[0] == pytest.approx(0.95, abs=0.1) and f.size >= 15
    assert np.linalg.norm(f.goal - f.centroid) <= 0.7
    assert f.unseen_dir[0] > 0.9


def test_frontier_completion_and_small_clusters_dropped():
    g = _room_grid()
    i, j = g.to_cell([0.0, 0.0])
    g.logodds[i + 10, j - 11:j + 11] = 2.0          # close the open side
    g.observed[i + 10, j - 11:j + 11] = True
    fs, _ = _frontiers(g, [0.0, 0.0])
    assert fr.exploration_done(fs)
    g.observed[i + 11, j] = False                   # a 1-cell hole behind the wall
    g.logodds[i + 10, j] = -1.0
    fs, _ = _frontiers(g, [0.0, 0.0])
    assert fr.exploration_done(fs)                  # below min_cluster


def test_frontier_outside_footprint_ignored():
    g = _room_grid()
    crop = g.crop(1.0)
    covered = np.zeros((crop[1] - crop[0], crop[3] - crop[2]), dtype=bool)
    fs, _ = _frontiers(g, [0.0, 0.0], covered)
    assert fs == []


# ── arena ───────────────────────────────────────────────────────────────────
def _rect_points(cx, cy, L, W, yaw, n=400, seed=0):
    rng = np.random.default_rng(seed)
    s = rng.uniform(-0.5, 0.5, (n, 1)) * np.array([[L, W]])
    side = rng.integers(0, 4, n)
    s[side == 0, 0] = L / 2
    s[side == 1, 0] = -L / 2
    s[side == 2, 1] = W / 2
    s[side == 3, 1] = -W / 2
    c, sn = math.cos(yaw), math.sin(yaw)
    return s @ np.array([[c, -sn], [sn, c]]).T + [cx, cy], side


def test_min_area_rect_rotated():
    pts, _ = _rect_points(3.0, -2.0, 8.0, 6.0, 0.6)
    r = min_area_rect(pts)
    assert np.allclose(r.center, [3.0, -2.0], atol=0.05)
    assert sorted(r.size) == pytest.approx([6.0, 8.0], abs=0.05)


def test_robust_rect_trims_outliers():
    pts, _ = _rect_points(0.0, 0.0, 8.0, 8.0, 0.3)
    pts = np.vstack([pts, [[15.0, 15.0], [-14.0, 3.0]]])
    r = robust_rect(pts, trim=1.0)
    assert np.allclose(r.center, [0.0, 0.0], atol=0.2)


def test_supported_rect_with_a_hidden_wall():
    pts, side = _rect_points(4.0, 4.0, 8.0, 8.0, 0.0, n=600)
    seen = pts[side != 3]                            # the y = 0 wall is hidden
    structure = np.column_stack([np.random.default_rng(1).uniform(2, 8, 200),
                                 np.random.default_rng(2).uniform(0.05, 2, 200)])
    r = supported_rect(seen, inside=structure)
    assert np.allclose(r.center, [4.0, 4.0], atol=0.1)


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
    from hydrone_mission.maze.openings import Opening
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
