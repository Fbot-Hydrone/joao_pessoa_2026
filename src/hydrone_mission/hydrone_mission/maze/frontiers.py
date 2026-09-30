"""Frontiers: known-free cells next to never-observed ones, inside the structure footprint.

Clusters below min_cluster cells are noise (wall corners, a diagonal gap in a
wall). A cluster is a goal only if some passable cell within view_radius of it
is reachable; the lidar is 360 deg, so getting close is enough to see it.
"""
from dataclasses import dataclass, field

import numpy as np

from hydrone_mission.maze.morph import dilate, dilate4, label
from hydrone_mission.maze.planner import line_free


@dataclass
class FrontierParams:
    footprint_margin: float = 0.5   # covered region grown by this much
    min_cluster: int = 5            # cells
    view_radius: float = 0.6        # a cluster counts as visited from this close
    view_cos: float = 0.5           # a view cell must look toward the unseen side
    bad_view_radius: float = 0.35   # given-up view cells block this much around them


@dataclass
class Frontier:
    cells: np.ndarray               # (k,2) crop cells
    centroid: np.ndarray            # (2,) odom xy
    size: int
    unseen_dir: np.ndarray = None   # (2,) cell units, toward the unobserved side
    cost: float = np.inf            # path cost to the view cell, m
    goal_cell: tuple = None         # crop cell to fly to
    goal: np.ndarray = field(default=None)


def footprint(covered, res, margin):
    return dilate(covered, margin / res) if covered.any() else covered


def frontier_mask(grid, crop, foot):
    i0, i1, j0, j1 = crop
    free = grid.free()[i0:i1, j0:j1]
    # never observed, not just undecided: thin wall cells flip between hit and
    # miss and would be frontiers forever. 4-connected, so a free cell hugging a
    # one-cell wall doesn't see the unobserved side through its corners
    unseen = ~grid.observed[i0:i1, j0:j1]
    return free & dilate4(unseen) & foot


def clusters(mask, grid, crop, min_cluster):
    lab, n = label(mask, conn=8)
    i0, i1, j0, j1 = crop
    unseen = ~grid.observed[i0:i1, j0:j1]
    h, w = unseen.shape
    out = []
    for k in range(1, n + 1):
        cells = np.argwhere(lab == k)
        if len(cells) < min_cluster:
            continue
        xy = grid.origin + (cells + np.array([i0, j0]) + 0.5) * grid.p.res
        d = np.zeros(2)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = np.clip(cells[:, 0] + di, 0, h - 1), np.clip(cells[:, 1] + dj, 0, w - 1)
            d += np.array([di, dj]) * unseen[a, b].sum()
        n_d = np.linalg.norm(d)
        out.append(Frontier(cells=cells, centroid=xy.mean(axis=0), size=len(cells),
                            unseen_dir=d / n_d if n_d > 0 else None))
    return out


def rank(frontiers, cost_cells, res, view_radius, planner=None, occ=None, tries=60,
         view_cos=0.5, bad_views=(), bad_view_radius=0.35):
    """Fill in cost/goal for each cluster from a cost field (cells). Sorted cheapest first.

    The goal is the cheapest reachable cell within view_radius that sees the
    cluster (no occupied cell on the line, if occ is given), looks toward its
    unseen side, and isn't near a view cell that already failed (bad_views, xy).
    """
    h, w = cost_cells.shape
    r = int(np.ceil(view_radius / res))
    for f in frontiers:
        a0, b0 = np.maximum(f.cells.min(axis=0) - r, 0)
        a1, b1 = np.minimum(f.cells.max(axis=0) + r + 1, (h, w))
        m = np.zeros((a1 - a0, b1 - b0), dtype=bool)
        m[f.cells[:, 0] - a0, f.cells[:, 1] - b0] = True
        near = dilate(m, view_radius / res)
        sub = np.where(near, cost_cells[a0:a1, b0:b1], np.inf)
        order = np.argsort(sub, axis=None)[:tries]
        for k in order:
            if not np.isfinite(sub.flat[k]):
                break
            cell = (int(a0 + k // sub.shape[1]), int(b0 + k % sub.shape[1]))
            if occ is not None and not _sees(occ, cell, f.cells, r):
                continue
            if f.unseen_dir is not None:
                # looking past the cluster into the unknown, not grazing along it
                near = f.cells[np.argmin(((f.cells - np.array(cell)) ** 2).sum(axis=1))]
                look = near - np.array(cell)
                nl = np.linalg.norm(look)
                if nl > 0 and look @ f.unseen_dir / nl < view_cos:
                    continue
            if planner is not None and bad_views:
                xy = planner.xy(cell)
                if any(np.linalg.norm(xy - b) < bad_view_radius for b in bad_views):
                    continue
            f.cost = float(sub.flat[k]) * res
            f.goal_cell = cell
            if planner is not None:
                f.goal = planner.xy(cell)
            break
    return sorted(frontiers, key=lambda f: f.cost)


def _sees(occ, cell, cells, r):
    """Line of sight from cell to any of the nearby cluster cells."""
    d2 = ((cells - np.array(cell)) ** 2).sum(axis=1)
    for k in np.argsort(d2)[:6]:
        if d2[k] > (r + 1) ** 2:
            break
        if line_free(occ, cell, tuple(cells[k])):
            return True
    return False


def find_frontiers(grid, crop, covered, cost_cells, params=None, planner=None, ignore=(),
                   bad_views=()):
    """Reachable frontier clusters, cheapest first, plus the raw frontier mask.

    ignore     xy of given-up clusters: clusters centred within view_radius are skipped
    bad_views  xy of view cells that didn't clear their cluster; not used again
    """
    p = params or FrontierParams()
    foot = footprint(covered, grid.p.res, p.footprint_margin)
    mask = frontier_mask(grid, crop, foot)
    fs = clusters(mask, grid, crop, p.min_cluster)
    fs = [f for f in fs if all(np.linalg.norm(f.centroid - q) > p.view_radius for q in ignore)]
    occ = grid.occupied()[crop[0]:crop[1], crop[2]:crop[3]]
    fs = rank(fs, cost_cells, grid.p.res, p.view_radius, planner, occ,
              view_cos=p.view_cos, bad_views=bad_views, bad_view_radius=p.bad_view_radius)
    return [f for f in fs if np.isfinite(f.cost)], mask


def exploration_done(reachable_frontiers):
    return len(reachable_frontiers) == 0
