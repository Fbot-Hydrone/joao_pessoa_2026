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

# Three fused sightings before a pad is worth a leg. One frame of blue noise
# reaches the map; three from different angles do not.
MIN_OBSERVATIONS = 3

# A pad this close to where we armed is the takeoff base under another id.
HOME_RADIUS_M = 1.0


def is_candidate(pad, *, blacklist=(), home=None,
                 min_observations=MIN_OBSERVATIONS,
                 home_radius=HOME_RADIUS_M):
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
