"""Velocity-level navigation: hover, follow a path, replan to a goal.

Everything a mission needs to move, and nothing about *where* to go. The
obstacle model is pluggable:
  GridObstacles   slow down near and push off occupied cells of a live grid
  BeamObstacles   the beam guard (guard.py): no map, no pose, just ranges
Planning is pluggable too: go_to() takes a factory returning a GridPlanner.

Velocities are in the planning frame, (vx, vy, vz), capped.
"""
from dataclasses import dataclass
import math

import numpy as np

from hydrone_confined.core.guard import GuardParams, guard


@dataclass
class NavParams:
    v_z: float = 0.3
    kp_xy: float = 0.8
    kp_z: float = 1.0
    lookahead: float = 0.4
    goal_tol: float = 0.15
    replan_s: float = 1.0


class GridObstacles:
    """Repulsion from a live occupancy grid (the explorer's own map)."""

    def __init__(self, grid, wall_slow=0.35, repulse_gain=0.15):
        self.grid = grid
        self.wall_slow = wall_slow
        self.repulse_gain = repulse_gain

    def walls(self, pos, radius):
        """(min distance to an occupied cell, repulsion vector) within radius."""
        g = self.grid
        r = int(math.ceil(radius / g.p.res))
        ci, cj = g.to_cell(pos[:2])
        i0, i1, j0, j1 = max(ci - r, 0), min(ci + r + 1, g.n), max(cj - r, 0), min(cj + r + 1, g.n)
        cells = np.argwhere(g.occupied()[i0:i1, j0:j1]) + np.array([i0, j0])
        if not len(cells):
            return np.inf, np.zeros(2)
        d = pos[:2] - g.cell_center(cells)
        dist = np.linalg.norm(d, axis=1)
        m = dist < radius
        if not m.any():
            return float(dist.min()), np.zeros(2)
        w = (radius - dist[m]) / radius
        push = (d[m] / np.maximum(dist[m], 1e-3)[:, None] * w[:, None]).sum(axis=0)
        n = np.linalg.norm(push)
        push = push / n * min(n, 1.0) if n > 0 else push
        return float(dist.min()), push * self.repulse_gain

    def filter(self, v_xy, pos):
        dmin, push = self.walls(pos, self.wall_slow)
        v = np.asarray(v_xy, dtype=float)
        if dmin < self.wall_slow:
            v = v * max(0.4, dmin / self.wall_slow)
        s = np.linalg.norm(v)
        if s > 1e-6:
            # slide along the wall instead of being pushed back off it: in a
            # 0.6 m doorway the jambs push straight back down the path, and the
            # drone sat in front of the gap for a minute and a half
            d = v / s
            against = float(push @ d)
            if against < 0:
                push = push - against * d
        return v + push


class BeamObstacles:
    """The beam guard as an obstacle model. Feed it every scan with update()."""

    def __init__(self, params=None):
        self.p = params or GuardParams()
        self.sectors = None
        self.yaw = 0.0
        self.closest = np.inf

    def update(self, sectors, yaw):
        self.sectors, self.yaw = sectors, yaw

    def filter(self, v_xy, pos):
        if self.sectors is None:
            return np.asarray(v_xy, dtype=float)
        v, self.closest, _ = guard(v_xy, self.yaw, self.sectors, self.p)
        return v


class Navigator:

    def __init__(self, params=None, obstacles=None):
        self.p = params or NavParams()
        self.obstacles = obstacles
        self.path = None
        self.t_plan = -1e9

    def reset(self):
        self.path = None
        self.t_plan = -1e9

    def z_rate(self, pos, z):
        return float(np.clip(self.p.kp_z * (z - pos[2]), -self.p.v_z, self.p.v_z))

    def velocity(self, v_xy, pos, z, cap):
        v = np.asarray(v_xy, dtype=float)
        if self.obstacles is not None:
            v = self.obstacles.filter(v, pos)
        s = np.linalg.norm(v)
        if s > cap:
            v = v / s * cap
        return np.array([v[0], v[1], self.z_rate(pos, z)])

    def hover(self, pos, xy, z, cap):
        v = self.p.kp_xy * (np.asarray(xy) - pos[:2])
        return self.velocity(v, pos, z, cap)

    def follow(self, pos, path, z, cap):
        """Pure pursuit on a polyline; slows to a stop at its end."""
        pts = np.asarray(path)
        seg = np.diff(pts, axis=0)
        if len(pts) < 2:
            return self.hover(pos, pts[-1], z, cap)
        # closest point on the polyline, then walk lookahead along it
        best_k, best_s, best_d = 0, 0.0, np.inf
        for k, (a, s) in enumerate(zip(pts[:-1], seg)):
            L2 = float(s @ s) or 1e-9
            u = float(np.clip((pos[:2] - a) @ s / L2, 0, 1))
            d = np.linalg.norm(a + u * s - pos[:2])
            if d < best_d:
                best_k, best_s, best_d = k, u, d
        left = self.p.lookahead
        k, u = best_k, best_s
        carrot = pts[-1]
        while k < len(seg):
            L = np.linalg.norm(seg[k])
            rest = (1 - u) * L
            if rest >= left:
                carrot = pts[k] + seg[k] * (u + left / max(L, 1e-9))
                break
            left -= rest
            k, u = k + 1, 0.0
        to_goal = np.linalg.norm(pts[-1] - pos[:2])
        d = carrot - pos[:2]
        n = np.linalg.norm(d)
        speed = min(cap, 0.8 * to_goal + 0.03)
        v = d / n * speed if n > 1e-6 else np.zeros(2)
        return self.velocity(v, pos, z, cap)

    @staticmethod
    def path_blocked(pl, path, pos, ahead=1.5):
        if path is None:
            return True
        walked = 0.0
        prev = pos[:2]
        for q in path:
            c = pl.cell(q)
            h, w = pl.blocked.shape
            if walked > 0.25 and 0 <= c[0] < h and 0 <= c[1] < w and pl.blocked[c]:
                return True
            walked += np.linalg.norm(q - prev)
            prev = q
            if walked > ahead:
                break
        return False

    def go_to(self, t, pos, goal, z, cap, make_planner):
        """Replanning path follower. Returns (velocity, arrived, planned_ok).

        make_planner(pos) -> GridPlanner already started from pos.
        """
        if np.linalg.norm(goal - pos[:2]) < self.p.goal_tol:
            return self.hover(pos, goal, z, cap), True, True
        if t - self.t_plan >= self.p.replan_s or self.path is None:
            self.t_plan = t
            pl = make_planner(pos)
            path = pl.path_to(goal, snap=0.4)
            if path is not None:
                path[-1] = goal if np.linalg.norm(path[-1] - goal) < 0.4 else path[-1]
                self.path = np.vstack([pos[:2], path])
            elif self.path is not None and not self.path_blocked(pl, self.path, pos):
                pass    # keep the old one while it's still clear
            else:
                self.path = None
        if self.path is None:
            return self.hover(pos, pos[:2], z, cap), False, False
        return self.follow(pos, self.path, z, cap), False, True
