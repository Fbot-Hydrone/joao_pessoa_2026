"""relief — things standing above the floor that might be a landing base.

The blue detector answers "does this look like a pad". It answers it from a
camera, so it inherits the camera's problems: at 7 m across the arena the ring
and the cross are a handful of pixels, and — worse — a pad that is NOT ON THE
FLOOR is placed wrongly no matter how well it is seen. The ground-plane
projection intersects the camera ray with z = floor, and a base sitting 1.5 m
up is not on that plane, so the answer lands somewhere behind or in front of
where the base actually is. Turning on the spot does not fix that: the problem
is not the angle, it is that the projection model is wrong for an elevated pad.

The occupancy map does not share the problem. It measures where matter IS, in
three dimensions, by ray-casting. So it can answer a different question:

    is there something standing here that is not the floor, not a wall, and
    not the house?

Competition bases sit between 0 and 1.5 m up, so anything isolated in that band
with roughly a base's footprint is worth flying over and looking at. It is not
a detection — it is a REASON TO GO LOOK, which is exactly what the mission is
short of.

ROS-free and octomap-free, like planner.py and coverage.py: the map arrives as
a callable from point to 'occupied' / 'free' / 'unknown'.
"""

import math

# The band a base can be in. Below FLOOR_Z is the floor itself; above CEILING_Z
# is the arena's net and whatever hangs from it.
FLOOR_Z = 0.25
CEILING_Z = 1.8

# A base is 1 m across in the arena. A cluster much smaller is noise, much
# larger is the house, a wall or a person.
MIN_FOOTPRINT_CELLS = 2
MAX_FOOTPRINT_M = 2.0

# How far from the arena edge a cluster has to be to not simply be the wall.
WALL_MARGIN_M = 0.6


def _neighbours(cell):
    x, y = cell
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx or dy:
                yield (x + dx, y + dy)


def occupied_cells(occupancy, bounds, resolution, *, floor_z=FLOOR_Z,
                   ceiling_z=CEILING_Z, exclude=()):
    """Grid cells with something standing in the base band.

    `exclude` is a list of (min_x, min_y, max_x, max_y) regions to ignore —
    the house goes here. It is a known structure, it is big, and it is exactly
    the thing that would otherwise dominate every cluster.

    A cell counts if ANY sample up the band is occupied: a base at 1.4 m and a
    base at 0.3 m are both bases, and scanning the column is what makes the
    height irrelevant to the question being asked.
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    zs = []
    z = floor_z
    while z <= ceiling_z + 1e-9:
        zs.append(z)
        z += resolution
    cells = set()
    n_x = int(math.floor((max_x - min_x) / resolution)) + 1
    n_y = int(math.floor((max_y - min_y) / resolution)) + 1
    for i in range(n_x):
        for j in range(n_y):
            x = min_x + i * resolution
            y = min_y + j * resolution
            if any(ex_min_x <= x <= ex_max_x and ex_min_y <= y <= ex_max_y
                   for ex_min_x, ex_min_y, ex_max_x, ex_max_y in exclude):
                continue
            if any(occupancy((x, y, zz)) == "occupied" for zz in zs):
                cells.add((i, j))
    return cells


def cluster(cells):
    """Connected groups of cells, 8-connected. Flood fill, no dependencies."""
    remaining = set(cells)
    out = []
    while remaining:
        seed = remaining.pop()
        group = {seed}
        stack = [seed]
        while stack:
            cur = stack.pop()
            for nb in _neighbours(cur):
                if nb in remaining:
                    remaining.remove(nb)
                    group.add(nb)
                    stack.append(nb)
        out.append(group)
    return out


def relief_candidates(occupancy, bounds, resolution, *, floor_z=FLOOR_Z,
                      ceiling_z=CEILING_Z, exclude=(),
                      min_cells=MIN_FOOTPRINT_CELLS,
                      max_footprint_m=MAX_FOOTPRINT_M,
                      wall_margin_m=WALL_MARGIN_M):
    """Isolated base-sized relief, as a list of (x, y) centres.

    Rejected, in order and for different reasons:

    * too few cells — noise, or the corner of something the map barely saw
    * too wide — the house, a wall, or two things the flood fill joined
    * touching the arena edge — that is the wall itself, and the margin is
      what separates "a base near the wall" from "the wall"

    The output is deliberately a POSITION and nothing else. What it means is
    "something is standing here"; whether it is a base is for the belly camera
    and the rangefinder to say after the mission flies over it.
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    cells = occupied_cells(occupancy, bounds, resolution, floor_z=floor_z,
                           ceiling_z=ceiling_z, exclude=exclude)
    out = []
    for group in cluster(cells):
        if len(group) < min_cells:
            continue
        xs = [min_x + i * resolution for i, _ in group]
        ys = [min_y + j * resolution for _, j in group]
        if (max(xs) - min(xs) > max_footprint_m
                or max(ys) - min(ys) > max_footprint_m):
            continue
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        if (cx - min_x < wall_margin_m or max_x - cx < wall_margin_m
                or cy - min_y < wall_margin_m or max_y - cy < wall_margin_m):
            continue
        out.append((cx, cy))
    return out
