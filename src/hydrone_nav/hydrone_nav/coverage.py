"""coverage — where to go look next, so the arena stops having blind spots.

The Phase 1 search turns on the spot and never moves. MEASURED 2026-08-27 on a
5.5 minute run: ONE travel leg. The vehicle spends the whole mission rotating
at the point it took off from, so it only ever sees what is in line of sight of
that one place — a base behind the house, or outside that cone, does not exist
to it. The detector is not the limit. Nobody ever looked there.

This picks where to look from next. The question it answers is not "where is a
base" — `pad_map` answers that from detections, and it is the only thing that
can, because an occupancy map stores whether something is SOLID and a blue pad
and the white floor under it are the same occupied voxel to it. The question
here is the one only the octomap can answer:

    which parts of the arena has no ray ever reached?

Unknown space is not an obstacle and it is not a target. It is the list of
places a base could still be hiding, and flying to see them is what turns a
blind sweep into coverage.

Free of ROS and of octomap, like planner.py: the map arrives as a CALLABLE from
point to 'occupied' / 'free' / 'unknown'. Tests run against arenas written as
strings.

Scoring
-------
Every candidate viewpoint is scored

    gain(v) / (1 + travel_cost(v) + turn_cost(v))

`gain` is how many unobserved cells would come into view from v. The divisor is
what the trip costs, and it is not just distance — this is where "an A* that
minimises the drone's error" becomes something an algorithm can actually
deliver. A* cannot minimise localisation error directly; the error is not a
function of the path it can evaluate. But drift CORRELATES with two things that
are, and both were measured on this stack:

* **turning**, most of all. The 60 deg yaw jumps all happened during ROTATE
  with `path_len` frozen — a blank wall stops matching mid-turn, the VO holds
  its pose, and the rotation that really happened is never recorded.
* **distance flown**, which accumulates translation error.

So a viewpoint that is slightly worse but much cheaper to reach in turns is the
better one, and `TURN_COST_PER_RAD` is the dial that says by how much.
"""

import math

# What a metre of flight costs, and what a radian of turn costs, in the same
# units. Turning is dearer than travelling BECAUSE OF THE MEASUREMENT above: it
# is where this arena's odometry actually breaks, not merely where time goes.
# 1.0 per metre against 1.5 per radian means a 90 deg turn costs about as much
# as 2.4 m of flight.
TRAVEL_COST_PER_M = 1.0
TURN_COST_PER_RAD = 1.5

# The forward camera's reach for the purpose of "would I see it from there".
# Deliberately SHORTER than the detector's true range: a base at the far edge
# of the frame is a handful of pixels, and counting it as covered would let the
# search tick off arena it only technically looked at.
SENSOR_RANGE_M = 5.0

# Half-angle of the cone a viewpoint is credited with. The vehicle rotates on
# the spot once it arrives, so it eventually sees all around — this models one
# sweep being worth the whole circle, and the value only matters for cells so
# close that geometry stops mattering.
SENSOR_HALF_FOV_RAD = math.pi

# Below this many newly visible cells a trip is not worth the drift it costs.
MIN_GAIN_CELLS = 4


def unobserved_cells(occupancy, bounds, resolution, z):
    """The cells at height `z` no ray has reached, as a list of (x, y).

    This is the whole contribution of the occupancy map to the search, and it
    is a contribution nothing else in the stack can make: an accumulated point
    cloud cannot tell "I looked and it is empty" from "I never looked".
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    out = []
    n_x = int(math.floor((max_x - min_x) / resolution)) + 1
    n_y = int(math.floor((max_y - min_y) / resolution)) + 1
    for i in range(n_x):
        for j in range(n_y):
            x = min_x + i * resolution
            y = min_y + j * resolution
            if occupancy((x, y, z)) == "unknown":
                out.append((x, y))
    return out


def visible_from(occupancy, viewpoint, cells, z, *,
                 sensor_range=SENSOR_RANGE_M, resolution=0.5):
    """Which of `cells` a camera at `viewpoint` would actually see.

    Within range, and with nothing occupied between — a cell behind the house
    is not covered by standing on the other side of it. The ray is walked at
    `resolution` because that is the grain the answer is used at; finer would
    re-ask the same voxel.
    """
    seen = []
    vx, vy = viewpoint
    for (cx, cy) in cells:
        d = math.hypot(cx - vx, cy - vy)
        if d > sensor_range:
            continue
        if _blocked(occupancy, (vx, vy), (cx, cy), z, resolution):
            continue
        seen.append((cx, cy))
    return seen


def _blocked(occupancy, a, b, z, resolution):
    """Is there anything OCCUPIED strictly between a and b?

    Unknown does not block: the cells being counted are themselves unknown, so
    treating unknown as opaque would make every one of them invisible and the
    gain of every viewpoint zero.
    """
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = int(math.ceil(d / resolution))
    for k in range(1, n):          # strictly between: endpoints excluded
        t = k / n
        p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, z)
        if occupancy(p) == "occupied":
            return True
    return False


def trip_cost(frm, to, yaw, *, travel_per_m=TRAVEL_COST_PER_M,
              turn_per_rad=TURN_COST_PER_RAD):
    """What going from `frm` (heading `yaw`) to `to` costs in drift terms."""
    dx, dy = to[0] - frm[0], to[1] - frm[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0
    heading = math.atan2(dy, dx)
    turn = abs((heading - yaw + math.pi) % (2.0 * math.pi) - math.pi)
    return travel_per_m * dist + turn_per_rad * turn


def next_viewpoint(occupancy, *, frm, yaw, bounds, z,
                   cell_m=0.5, viewpoint_m=1.0,
                   sensor_range=SENSOR_RANGE_M,
                   min_gain=MIN_GAIN_CELLS,
                   travel_per_m=TRAVEL_COST_PER_M,
                   turn_per_rad=TURN_COST_PER_RAD,
                   avoid=(), avoid_radius_m=1.0):
    """Where to fly to see the most unseen arena for the least drift.

    Returns ((x, y), gain) or None when nothing is worth the trip — which is
    the honest answer for an arena that has been covered, and is what tells the
    mission the search is finished rather than merely out of turns.

    Two grids on purpose. `cell_m` is how finely coverage is COUNTED and
    `viewpoint_m` is how finely candidate positions are TRIED; making the
    second as fine as the first squares the work for viewpoints that differ by
    less than the vehicle's own position error.

    `avoid` is places already tried and not reached. Without it the search
    loops: the best viewpoint is still the best one after the trip to it
    failed, so it gets chosen again, and the mission alternates between turning
    a full sweep and failing the same flight. The radius exists because the
    vehicle will not stop on exactly the same coordinate twice.
    """
    cells = unobserved_cells(occupancy, bounds, cell_m, z)
    if not cells:
        return None

    (min_x, min_y, _), (max_x, max_y, _) = bounds
    best = None
    n_x = int(math.floor((max_x - min_x) / viewpoint_m)) + 1
    n_y = int(math.floor((max_y - min_y) / viewpoint_m)) + 1
    for i in range(n_x):
        for j in range(n_y):
            v = (min_x + i * viewpoint_m, min_y + j * viewpoint_m)
            # Stand only where the map says the vehicle fits. `unknown` is not
            # good enough: flying to a viewpoint chosen because nothing was
            # known about it is how the search walks into a wall.
            if occupancy((v[0], v[1], z)) != "free":
                continue
            if any(math.hypot(v[0] - a[0], v[1] - a[1]) <= avoid_radius_m
                   for a in avoid):
                continue
            gain = len(visible_from(occupancy, v, cells, z,
                                    sensor_range=sensor_range,
                                    resolution=cell_m))
            if gain < min_gain:
                continue
            cost = trip_cost(frm, v, yaw, travel_per_m=travel_per_m,
                             turn_per_rad=turn_per_rad)
            score = gain / (1.0 + cost)
            if best is None or score > best[0]:
                best = (score, v, gain)
    if best is None:
        return None
    return (best[1], best[2])


def _inward_yaw(edge_a, edge_b, centre):
    """Heading perpendicular to the edge a->b, pointing at the arena.

    Fixed for the whole edge: the camera holds one attitude while the vehicle
    translates, which is the entire reason this shape beats a circuit that
    re-aims at every step.
    """
    ex, ey = edge_b[0] - edge_a[0], edge_b[1] - edge_a[1]
    mx, my = (edge_a[0] + edge_b[0]) / 2.0, (edge_a[1] + edge_b[1]) / 2.0
    # Both perpendiculars; keep the one that points towards the centre.
    for nx, ny in ((-ey, ex), (ey, -ex)):
        if (centre[0] - mx) * nx + (centre[1] - my) * ny > 0:
            return math.atan2(ny, nx)
    return math.atan2(centre[1] - my, centre[0] - mx)


def u_sweep(bounds, *, inset_m=1.2, z=2.0, start_corner=2):
    """Three sides of the arena, as (x, y, z, yaw). FOUR POINTS PER LEG-END.

    LEVEL 1 of the search. The vehicle takes off, translates along one edge
    without rotating at all, turns 90 degrees at the corner, runs the next
    edge, turns 90 degrees again, runs the third. Two turns in the whole sweep,
    both at corners, and the camera faces INTO the arena throughout.

    Why three sides and not four: from the third edge the camera already looks
    back across everything the fourth would cover, so the fourth is a minute of
    battery spent re-photographing what is already in the map.

    Why fixed headings: every earlier shape failed on rotation. Spinning in
    place cannot map an arena at all (no parallax). A rectangle that re-aims at
    the centre each step turns CONTINUOUSLY along every edge. Turning only at
    corners is what lets the detector work a stable scene and keeps the
    odometry out of the manoeuvre this arena breaks it on.

    NO INTERMEDIATE POINTS. A leg is a straight line flown on one heading, so
    a point in the middle of it does nothing except tell the vehicle to stop.
    ArduCopter's GUIDED treats a position target as "go there and halt", and
    the mission waits to arrive before releasing the next one — so every
    intermediate point was a full decelerate-and-accelerate.

    MEASURED with the numbers this stack flies at: WP_SPD 1.5 m/s and WP_ACC
    1.5 m/s/s means reaching cruise takes 1 s and 0.75 m, and stopping the
    same. At the old 1.5 m spacing the vehicle NEVER REACHED CRUISE anywhere
    in the sweep — each hop was pure ramp, ~2.0 s for 1.5 m, an average of
    0.75 m/s against an airframe capable of twice that. One 5.6 m leg flown
    whole is ~4.7 s; the same leg in four hops is ~8 s plus three pauses.

    The intermediate points were inherited from the rectangle that came before,
    where the heading WAS re-aimed at every step and they had a job. Here they
    had none.

    `start_corner` indexes the inset rectangle's corners counter-clockwise from
    (min_x, min_y). It should be the one nearest where the drone took off, so
    the sweep starts without a transit leg.
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    x0, x1 = min_x + inset_m, max_x - inset_m
    y0, y1 = min_y + inset_m, max_y - inset_m
    if x1 <= x0 or y1 <= y0:
        return []
    centre = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    k = start_corner % 4
    order = [corners[(k + i) % 4] for i in range(4)]

    # TRANSLATE, THEN TURN — never both at once. A setpoint that changes
    # position and heading together asks the vehicle to yaw while it is still
    # moving, and yawing under translation is where this arena's odometry loses
    # the most: the camera sweeps, matching fails, and the rotation that really
    # happened is never recorded.
    #
    # So a corner is two setpoints at the SAME position: the leg that arrives
    # there (still on the old heading) and then a pure turn, standing still.
    out = []
    for i, (a, b) in enumerate(zip(order, order[1:])):   # three legs, not four
        yaw = _inward_yaw(a, b, centre)
        out.append((a[0], a[1], z, yaw))    # at the corner, now on the new yaw
        out.append((b[0], b[1], z, yaw))    # fly the whole leg, one setpoint
    return out


def lawnmower(bounds, *, inset_m=1.2, z=2.0, step_m=1.5, lane_m=1.5):
    """Parallel lanes across the arena, alternating direction.

    THE LAST RESORT. It is the most thorough shape there is and the most
    expensive: it covers the floor at `lane_m` spacing instead of relying on
    the camera reaching across the arena, so it finds a base that every other
    level looked past — and it costs several times the flight time.

    Heading follows the lane, flipping 180 degrees at each end, because at this
    point the point is coverage of the ground beneath rather than a long view
    across. That means many more turns than the U, which is exactly why it is
    last and not first.
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    x0, x1 = min_x + inset_m, max_x - inset_m
    y0, y1 = min_y + inset_m, max_y - inset_m
    if x1 <= x0 or y1 <= y0 or lane_m <= 0:
        return []

    out = []
    y = y0
    forward = True
    while y <= y1 + 1e-9:
        a, b = ((x0, y), (x1, y)) if forward else ((x1, y), (x0, y))
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        dist = abs(b[0] - a[0])
        n = max(1, int(math.ceil(dist / step_m)))
        for i in range(n + 1):
            t = i / n
            out.append((a[0] + (b[0] - a[0]) * t, y, z, yaw))
        forward = not forward
        y += lane_m
    return out


def lateral_sweep(bounds, *, inset_m=1.2, z=2.0, step_m=1.5):
    """Two straight passes across the arena, as (x, y, z, yaw) points.

    This replaces a rectangular circuit, and the reason is the same one that
    killed the spin before it: ROTATION. The circuit re-aimed the camera at the
    arena centre at every step, so it turned continuously along every edge —
    which is what "he is still spinning" kept meaning. A pass with a FIXED
    heading does not.

    The shape:

        pass A   along the top, looking across at the arena      (yaw = -90 deg)
        one 180 deg turn, at the end, once
        pass B   along the bottom, looking back the other way    (yaw = +90 deg)

    Two passes and one turn covers the floor from two opposite directions,
    which is what a base needs to be seen from — a pad that is edge-on or
    back-lit on one pass is face-on on the other. And a fixed heading during
    the translation is worth more than the angles it gives up: the detector
    gets a stable scene to work on, the depth camera sweeps a clean band into
    the occupancy map, and the odometry is never asked to do the one thing
    this arena breaks it on.

    `z` is NOT the mission's cruise altitude by default and must not be
    silently taken from it: the top pass runs over the house, whose roof is at
    1.5 m in the competition arena, so a sweep at the usual 1 m would fly into
    it. The caller passes an altitude that clears the house and stays under the
    net.
    """
    (min_x, min_y, _), (max_x, max_y, _) = bounds
    x0, x1 = min_x + inset_m, max_x - inset_m
    y_hi, y_lo = max_y - inset_m, min_y + inset_m
    if x1 <= x0 or y_hi <= y_lo:
        return []

    out = []
    span = x1 - x0
    n = max(1, int(math.ceil(span / step_m)))
    # Pass A: left to right along the top, looking south across the arena.
    for k in range(n + 1):
        out.append((x0 + span * (k / n), y_hi, z, -math.pi / 2.0))
    # Pass B: back along the bottom, looking north. The single 180 deg turn
    # happens on the way between them.
    for k in range(n + 1):
        out.append((x1 - span * (k / n), y_lo, z, math.pi / 2.0))
    return out
