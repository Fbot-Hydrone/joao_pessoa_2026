"""route — which pad to fly to next, as plain functions.

This is the planning half of a mission, kept deliberately free of ROS: it takes
pads and a position and returns a choice. Nothing here subscribes, publishes,
or holds state, so a mission node can ask it a question mid-flight and a test
can ask the same question with a list of fakes.

A "pad" is anything with `id`, `position.x`, `position.y`, `is_takeoff_base`,
`visited` and `observations` — the fields of hydrone_msgs/Pad. Depending on the
message type here would drag ROS back in for no gain.

Phase 1 picks the nearest eligible pad. That is not the shortest tour, and it
is not meant to be: in an 8x8 m arena the legs differ by little, and the
shortest next leg is the least visual-odometry drift accumulated before the
landing that matters. A real tour optimiser belongs here too, when a phase
needs one — that is the point of it being a library.
"""

import math

# Five fused sightings before a pad is worth a leg in the normal search.
#
# This was two, on the argument that the confirmation hover is the real filter.
# MEASURED 2026-10-01, seed 100, YOLO belly: it is not. A second map entry 1.26 m
# from the tall base 4 (top 1.40 m) was picked with TWO looks, and the hover
# "confirmed" it with six belly frames — the camera was seeing base 4 from the
# side. The vehicle landed on the floor beside it, which ends the attempt. The
# six real bases had 27-274 looks by the time they were flown to.
#
# The old worry — real bases seen only twice from far away never qualifying —
# is covered by investigation mode, which drops to one look once a level has
# been flown (phase1_mission_node._is_candidate).
MIN_OBSERVATIONS = 5

# A pad this close to where we armed is the takeoff base under another id.
HOME_RADIUS_M = 1.0

# A pad this close to one we already landed on is THAT base under another id.
# 2026-10-07: two map entries converged onto the same base and the mission
# landed on it twice (-5 per repeat). Real bases are >= 1.5 m apart
# (biguasim_main.bases.sample_bases min_spacing), so 0.8 m never hides one.
VISITED_RADIUS_M = 0.8


def is_candidate(pad, *, blacklist=(), home=None,
                 min_observations=MIN_OBSERVATIONS,
                 home_radius=HOME_RADIUS_M, visited_xy=(),
                 visited_radius=VISITED_RADIUS_M):
    """Is this pad worth flying to?

    Not if it is the base we took off from, not if we already landed on it,
    not if the belly camera refused it, and not until the map has seen it
    enough times to be sure it exists.
    """
    if pad.is_takeoff_base or pad.visited:
        return False
    if int(pad.id) in blacklist:
        return False
    if pad.observations < min_observations:
        return False
    # Belt and braces for the case where registration failed: never treat
    # anything sitting where we armed as a landing site.
    if home is not None:
        if math.hypot(pad.position.x - home[0],
                      pad.position.y - home[1]) < home_radius:
            return False
    for vx, vy in visited_xy:
        if math.hypot(pad.position.x - vx,
                      pad.position.y - vy) < visited_radius:
            return False
    return True


def nearest_candidate(pads, x, y, **kwargs):
    """The closest pad to (x, y) worth flying to, or None.

    `kwargs` are passed straight to is_candidate.
    """
    best, best_d = None, float("inf")
    for pad in pads:
        if not is_candidate(pad, **kwargs):
            continue
        d = math.hypot(pad.position.x - x, pad.position.y - y)
        if d < best_d:
            best, best_d = pad, d
    return best


def takeoff_base_xy(pads, fallback=(0.0, 0.0)):
    """Where home is: the map's registered takeoff base, else `fallback`."""
    for pad in pads:
        if pad.is_takeoff_base:
            return (pad.position.x, pad.position.y)
    return fallback
