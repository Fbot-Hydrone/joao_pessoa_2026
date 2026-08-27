"""planner — A* over an occupancy map, as a plain function.

`path_is_clear` in hydrone_map.octree answers "is this straight line clear?".
It does not generate the lines. This does, and it is the piece that turns the
occupancy map into navigation.

Deliberately free of both ROS and octomap: the map arrives as a CALLABLE that
takes a point and returns 'occupied' / 'free' / 'unknown'. So a test can plan
through a maze written as a string in a second, the octree can be handed in as
`lambda p: inflated_state(tree, p)` from a node, and neither the library nor
its tests need a simulator running. The same rule that keeps route.py and
cloud_filter.py testable.

    from hydrone_nav.planner import plan

    path = plan(occupancy, start=(0, 0, 1.0), goal=(3, -2, 1.0),
                resolution=0.15, bounds=((-4, -4, 0.3), (4, 4, 2.5)))

Why A* and not RRT
------------------
The arena is 8x8x2.5 m and the map is already a grid, so the search space is
small and discrete — the regime A* was made for. It returns the shortest path
or proves there is none, and it returns the SAME path twice for the same map,
which matters when the thing has to be debugged from a log and defended to a
judge. RRT earns its randomness in high-dimensional spaces; here it would only
cost reproducibility.

Unknown space
-------------
Refused by default, and this is the important default. Phase 4 flies a confined
dark space and the map has large unlooked-at regions early on; a planner that
treats "never measured" as "empty" will confidently route straight through a
wall it has not seen yet. `allow_unknown=True` exists because a frontier
explorer needs exactly the opposite behaviour, and it should be an explicit
decision at the call site.
"""

import heapq
import math

# 26-connected: faces, edges and corners. A 6-connected grid can only produce
# axis-aligned staircases, which are ~15% longer in the plane and read as
# jittery flight; the diagonals cost what they actually measure, so the search
# has no incentive to prefer one shape over another.
NEIGHBOURS = [(dx, dy, dz)
              for dx in (-1, 0, 1)
              for dy in (-1, 0, 1)
              for dz in (-1, 0, 1)
              if (dx, dy, dz) != (0, 0, 0)]

# Vertical motion is not free the way horizontal motion is: it costs battery,
# it moves the camera off the horizon the VO tracks, and in a netted arena it
# runs out of room fastest. Weighting it makes the search prefer to go around
# rather than over, without forbidding over.
Z_COST_FACTOR = 1.5


class Occupancy:
    OCCUPIED = "occupied"
    FREE = "free"
    UNKNOWN = "unknown"


def plan(occupancy, start, goal, *, resolution, bounds=None,
         allow_unknown=False, z_cost_factor=Z_COST_FACTOR, max_expansions=200000):
    """Shortest 26-connected path from `start` to `goal`, or None.

    `occupancy(point) -> str` is the map. `resolution` is the grid step, which
    should be the tree's own resolution — coarser steps over a voxel, finer
    asks the same voxel repeatedly for no new information.

    `bounds` is ((min_x, min_y, min_z), (max_x, max_y, max_z)), and it is worth
    passing: without it a search that cannot reach the goal will expand
    outwards through unbounded free space until `max_expansions` stops it.

    Returns a list of (x, y, z) in metres, start first and goal last, or None
    when no path exists. None is the honest answer and the caller must handle
    it — the arena is small enough that "no path" usually means the map has not
    seen enough yet, not that the drone is trapped.

    The goal is snapped to the grid, so the last point is within half a cell of
    what was asked for. A start or goal that is not itself free returns None
    rather than planning out of a wall.
    """
    if resolution <= 0:
        raise ValueError("planner: resolution must be positive")

    origin = tuple(float(v) for v in start)

    def to_cell(p):
        return tuple(int(round((p[i] - origin[i]) / resolution))
                     for i in range(3))

    def to_point(c):
        return tuple(origin[i] + c[i] * resolution for i in range(3))

    def passable(c):
        p = to_point(c)
        if bounds is not None:
            lo, hi = bounds
            if any(p[i] < lo[i] or p[i] > hi[i] for i in range(3)):
                return False
        s = occupancy(p)
        if s == Occupancy.FREE:
            return True
        return allow_unknown and s == Occupancy.UNKNOWN

    start_c = (0, 0, 0)
    goal_c = to_cell(goal)
    if not passable(start_c):
        return None
    if start_c == goal_c:
        return [to_point(start_c)]
    if not passable(goal_c):
        return None

    def step_cost(d):
        dz = d[2] * z_cost_factor
        return math.sqrt(d[0] * d[0] + d[1] * d[1] + dz * dz) * resolution

    def heuristic(c):
        # Euclidean, with the same z weighting the edges use. Weighting one and
        # not the other would make the heuristic inadmissible and A* would stop
        # returning the shortest path.
        dx = (goal_c[0] - c[0])
        dy = (goal_c[1] - c[1])
        dz = (goal_c[2] - c[2]) * z_cost_factor
        return math.sqrt(dx * dx + dy * dy + dz * dz) * resolution

    open_heap = [(heuristic(start_c), 0.0, start_c)]
    came_from = {}
    best = {start_c: 0.0}
    closed = set()
    expansions = 0

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal_c:
            path = [cur]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return [to_point(c) for c in reversed(path)]

        expansions += 1
        if expansions > max_expansions:
            return None

        for d in NEIGHBOURS:
            nxt = (cur[0] + d[0], cur[1] + d[1], cur[2] + d[2])
            if nxt in closed or not passable(nxt):
                continue
            ng = g + step_cost(d)
            if ng < best.get(nxt, math.inf):
                best[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_heap, (ng + heuristic(nxt), ng, nxt))

    return None


def simplify(path, clear):
    """Drop the waypoints a straight line already covers.

    A* returns one point per grid cell — dozens for a 5 m leg, each of which
    would become a setpoint the vehicle decelerates into. `clear(a, b)` is a
    line-of-sight test (hydrone_map.octree.path_is_clear, closed over the
    tree and the inflation radius); this keeps a waypoint only where the path
    actually has to bend.

    The geometry is unchanged — every retained segment was tested clear by the
    same map that produced the path — so this is a reduction in setpoints, not
    a shortcut through anything.
    """
    if len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
