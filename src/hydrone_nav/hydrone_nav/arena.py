"""arena — the competition layout, and how to turn it into the `map` frame.

ROS-free. The rules give base positions in the ARENA frame (Figure 7: origin at
the corner by the takeoff base, x along the Phase 4 block, y away from it), and
say they are known before the attempt. The drone flies in `map`, whose origin
and heading are wherever the EKF put them. The one anchor both share is the
takeoff base: the vehicle is standing on it at arming, so

    map = home_map + Rz(arena_yaw) * (p_arena - takeoff_arena)

`arena_yaw` is the rotation from arena axes to map axes. In this simulator it
is -90 deg (arena +x runs along map -y; checked against Phase 1's home,
(-3.39, 3.00) in map vs (-3.25, 2.97) from Figure 7). On the real drone it is
how the vehicle is turned on the takeoff base relative to the arena — measure
it once on the day.
"""

import math
from dataclasses import dataclass, field

import yaml


@dataclass
class Layout:
    takeoff: tuple
    pickup: list = field(default_factory=list)      # [(x, y, z_top), ...]
    delivery: list = field(default_factory=list)    # [(x, y) or (x, y, z)]
    delivery_z_range: tuple = (0.0, 1.5)


def _point(e, grid):
    """A layout entry -> (x, y[, z[, yaw]]) in metres.

    Either metres, [x, y, ...], or a grid index, {grid: [col, row], z, yaw}:
    the organisers may give a base as a square of the floor grid rather than
    as X, Y. Index 0 is the arena corner by the takeoff; `centre: node` puts
    the base on the crossing of lines `col` and `row` (where Figure 7 draws
    them), `centre: cell` on the middle of square [col, row].
    """
    if not isinstance(e, dict):
        return tuple(e)
    step = float(grid.get("step_m", 0.5))
    ox, oy = grid.get("origin_m", (0.0, 0.0))
    half = 0.5 if grid.get("centre", "node") == "cell" else 0.0
    col, row = e["grid"]
    p = (ox + (col + half) * step, oy + (row + half) * step)
    if "z" in e:
        p += (float(e["z"]),)
        if "yaw" in e:
            p += (float(e["yaw"]),)
    return p


def load_layout(path) -> Layout:
    with open(path) as f:
        d = yaml.safe_load(f)
    grid = d.get("grid") or {}
    return Layout(
        takeoff=_point(d["takeoff"], grid),
        pickup=[_point(p, grid) for p in d.get("pickup", [])],
        delivery=[_point(p, grid) for p in d.get("delivery", [])],
        delivery_z_range=tuple(d.get("delivery_z_range", (0.0, 1.5))))


def arena_to_map(p, home_map, takeoff_arena, arena_yaw_deg=-90.0):
    """(x, y) in the arena frame -> (x, y) in `map`, anchored at the takeoff."""
    dx, dy = p[0] - takeoff_arena[0], p[1] - takeoff_arena[1]
    c = math.cos(math.radians(arena_yaw_deg))
    s = math.sin(math.radians(arena_yaw_deg))
    return (home_map[0] + c * dx - s * dy, home_map[1] + s * dx + c * dy)
