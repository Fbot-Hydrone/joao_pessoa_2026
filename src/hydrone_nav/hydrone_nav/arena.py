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


def load_layout(path) -> Layout:
    with open(path) as f:
        d = yaml.safe_load(f)
    return Layout(
        takeoff=tuple(d["takeoff"]),
        pickup=[tuple(p) for p in d.get("pickup", [])],
        delivery=[tuple(p) for p in d.get("delivery", [])],
        delivery_z_range=tuple(d.get("delivery_z_range", (0.0, 1.5))))


def arena_to_map(p, home_map, takeoff_arena, arena_yaw_deg=-90.0):
    """(x, y) in the arena frame -> (x, y) in `map`, anchored at the takeoff."""
    dx, dy = p[0] - takeoff_arena[0], p[1] - takeoff_arena[1]
    c = math.cos(math.radians(arena_yaw_deg))
    s = math.sin(math.radians(arena_yaw_deg))
    return (home_map[0] + c * dx - s * dy, home_map[1] + s * dx + c * dy)


# ── The arena's boxes, from where its walls are ─────────────────────────────
#
# Phase 1 has no layout, but it still needs to know where the walls are. In
# the sim the map is the arena's own frame (centred, home at (-3.39, 3.00)), so
# boxes of +-5 around the origin fit. On the real drone the ZED world starts at
# the start pose, so the map ORIGIN IS THE TAKEOFF BASE, which sits in a
# corner: the same box puts half of itself outside the net. Instead the
# operator says how far each wall is from the drone on the base, as it faces.
# Rulebook or eyeballed numbers are fine (teams may not measure the arena); the
# sweeps keep `survey_inset_m` off the walls, which absorbs about half a metre.


def walls_box(walls_m, start_yaw_deg=90.0):
    """[front, back, left, right] from the start -> (min_x, min_y, max_x, max_y).

    `start_yaw_deg` is the map heading of the drone's nose at arming. 90 on the
    real drone: the ZED world starts at the start pose with x forward (FLU) and
    vision_odom_bridge hands it to the FCU as ENU (E, N) = (-y, x), so in the
    map (= /mavros/local_position) forward is +y and right is +x.
    """
    f, b, l, r = (float(v) for v in walls_m)
    c = math.cos(math.radians(start_yaw_deg))
    s = math.sin(math.radians(start_yaw_deg))
    pts = [(x * c - y * s, x * s + y * c)
           for x, y in ((f, l), (f, -r), (-b, l), (-b, -r))]
    xs = [round(p[0], 6) + 0.0 for p in pts]
    ys = [round(p[1], 6) + 0.0 for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def arena_boxes(walls_m, plan_z=(0.3, 2.5), start_yaw_deg=90.0,
                reject_slack_m=0.5):
    """(plan_bounds, arena_bounds) for the mission and pad_map, or None if off.

    plan_bounds  [min_x, min_y, min_z, max_x, max_y, max_z]: the walls
                 themselves. The sweeps inset from it and the planner must not
                 route outside the net, so no slack (the sim's +-5 has a metre
                 because its walls are known exactly).
    arena_bounds [min_x, min_y, max_x, max_y]: walls + `reject_slack_m`, the
                 same half metre the sim gives pad_map before a base counts as
                 a ghost behind a wall.

    Off when `walls_m` is empty or all zeros. Anything else must be four
    positive distances: a typo must stop the launch, not fly a default box.
    """
    w = [float(v) for v in (walls_m or [])]
    if not any(w):
        return None
    if len(w) != 4 or min(w) <= 0.0:
        raise ValueError(
            "arena_walls_m must be [front, back, left, right], all > 0 m; "
            f"got {list(walls_m)}")
    x0, y0, x1, y1 = walls_box(w, start_yaw_deg)
    k = float(reject_slack_m)
    plan = [x0, y0, float(plan_z[0]), x1, y1, float(plan_z[1])]
    reject = [x0 - k, y0 - k, x1 + k, y1 + k]
    return plan, reject
