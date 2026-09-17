"""Grid planning: inflation, Dijkstra cost fields, A*, shortcutting.

Paths only go through cells the caller marks passable (known free, not inflated).
All grids here are small crops, indexed (i, j) with i along x and j along y.
"""
import heapq
import math

import numpy as np

from hydrone_mission.maze.morph import dilate

SQ2 = math.sqrt(2.0)
MOVES = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2))


def inflate(occ, radius_cells):
    """Grow obstacles by a disc (no wrap-around at the edges)."""
    return dilate(occ, radius_cells)


def _neighbours(blocked, i, j):
    h, w = blocked.shape
    for di, dj, c in MOVES:
        a, b = i + di, j + dj
        if not (0 <= a < h and 0 <= b < w) or blocked[a, b]:
            continue
        # no corner cutting past a blocked cell
        if di and dj and (blocked[i + di, j] or blocked[i, j + dj]):
            continue
        yield a, b, c


def cost_field(blocked, start):
    """Dijkstra from start; path cost in cells, inf where unreachable."""
    cost = np.full(blocked.shape, np.inf)
    if blocked[start]:
        return cost
    cost[start] = 0.0
    heap = [(0.0, start[0], start[1])]
    bl = blocked.tolist()      # python lists index much faster than numpy scalars
    h, w = blocked.shape
    cl = cost.tolist()
    while heap:
        g, i, j = heapq.heappop(heap)
        if g > cl[i][j]:
            continue
        for di, dj, c in MOVES:
            a, b = i + di, j + dj
            if a < 0 or b < 0 or a >= h or b >= w or bl[a][b]:
                continue
            if di and dj and (bl[i + di][j] or bl[i][j + dj]):
                continue
            ng = g + c
            if ng < cl[a][b]:
                cl[a][b] = ng
                heapq.heappush(heap, (ng, a, b))
    return np.array(cl)


def descend(cost, goal, blocked=None):
    """Walk a cost field back from goal to the start. Returns cells start..goal.

    Each step goes to a neighbour the Dijkstra could have come from (its cost
    plus the move equals ours), with the same no-corner-cutting rule.
    """
    if not np.isfinite(cost[goal]):
        return None
    h, w = cost.shape
    path = [goal]
    cur = goal
    while cost[cur] > 0:
        best, best_c = None, np.inf
        i, j = cur
        for di, dj, c in MOVES:
            a, b = i + di, j + dj
            if not (0 <= a < h and 0 <= b < w) or not np.isfinite(cost[a, b]):
                continue
            if abs(cost[a, b] + c - cost[cur]) > 1e-6:
                continue
            if di and dj and blocked is not None and (blocked[a, j] or blocked[i, b]):
                continue
            if cost[a, b] < best_c:
                best, best_c = (a, b), cost[a, b]
        if best is None:
            return None
        cur = best
        path.append(cur)
    return path[::-1]


def astar(blocked, start, goal):
    """8-connected A*. Returns [(i, j), ...] or None."""
    h, w = blocked.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and 0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if blocked[start] or blocked[goal]:
        return None
    g = {start: 0.0}
    came = {}
    heap = [(0.0, start)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for a, b, c in _neighbours(blocked, *cur):
            ng = g[cur] + c
            if ng < g.get((a, b), math.inf):
                g[(a, b)], came[(a, b)] = ng, cur
                heapq.heappush(heap, (ng + math.hypot(goal[0] - a, goal[1] - b), (a, b)))
    return None


def nearest_free(blocked, c, max_cells=None):
    """The passable cell closest to c (c itself if passable), or None."""
    c = (int(c[0]), int(c[1]))
    h, w = blocked.shape
    if 0 <= c[0] < h and 0 <= c[1] < w and not blocked[c]:
        return c
    free = np.argwhere(~blocked)
    if not len(free):
        return None
    d2 = ((free - np.array(c)) ** 2).sum(axis=1)
    k = int(np.argmin(d2))
    if max_cells is not None and d2[k] > max_cells ** 2:
        return None
    return tuple(int(v) for v in free[k])


def line_free(blocked, a, b):
    """True if the straight segment between two cells stays passable."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    ii = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    jj = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return not blocked[ii, jj].any()


def shortcut(blocked, path):
    """Greedy line-of-sight pruning of a cell path."""
    if path is None or len(path) < 3:
        return path
    out = [path[0]]
    k = 0
    while k < len(path) - 1:
        nxt = k + 1
        for m in range(len(path) - 1, k, -1):
            if line_free(blocked, path[k], path[m]):
                nxt = m
                break
        out.append(path[nxt])
        k = nxt
    return out


def densify(points, step):
    """Resample a polyline to at most `step` spacing."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


class GridPlanner:
    """Plans in world xy over a passable mask cropped out of an OccupancyGrid.

    `passable` is a bool crop (i0.., j0..) of the grid; origin/res map cells to metres.
    """

    def __init__(self, passable, offset, origin, res):
        self.blocked = ~passable
        self.offset = offset          # (i0, j0) of the crop inside the full grid
        self.origin = origin
        self.res = res
        self._cost = None
        self._start = None

    def cell(self, xy):
        ij = np.floor((np.asarray(xy, dtype=float) - self.origin) / self.res).astype(int)
        return int(ij[0] - self.offset[0]), int(ij[1] - self.offset[1])

    def xy(self, cell):
        return self.origin + (np.array(cell, dtype=float) + np.array(self.offset) + 0.5) * self.res

    def start_from(self, xy, snap=0.4):
        s = nearest_free(self.blocked, self.cell(xy), max_cells=snap / self.res)
        self._start = s
        self._cost = None if s is None else cost_field(self.blocked, s)
        return s is not None

    @property
    def cost(self):
        return self._cost

    def path_to_cell(self, goal):
        if self._cost is None:
            return None
        cells = descend(self._cost, goal, self.blocked)
        if cells is None:
            return None
        cells = shortcut(self.blocked, cells)
        return np.array([self.xy(c) for c in cells])

    def path_to(self, xy, snap=0.4):
        if self._cost is None:
            return None
        g = nearest_free(self.blocked, self.cell(xy), max_cells=snap / self.res)
        if g is None:
            return None
        return self.path_to_cell(g)

    def path_length(self, path):
        if path is None or len(path) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
