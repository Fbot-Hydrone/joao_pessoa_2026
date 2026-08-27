"""landmark — turning a re-observed pad into a measurement of the pose error.

This is the loop closure half of SLAM without the half that does not work in
this arena. Scan matching is the wrong tool here: four identical white walls,
a square floor plan with 90 deg symmetry and almost no texture, so an ICP
slides along a wall and mistakes one corner for another. The bases do not have
that problem — they are unique, sparse and individually identifiable, which is
exactly what a landmark needs to be.

The measurement
---------------
A pad in the map has a recorded position. Fly back over it and the detector
projects it somewhere, using the current pose. The physical pad has not moved,
so the whole difference is the pose:

    the map says      base 3 is at (2.02, -3.24)
    the detector says from this pose it is at (2.41, -3.02)
    difference        (0.39, 0.22) -- 0.45 m, and that IS the pose error

Nothing here subscribes, publishes or knows what ROS is, so a node can ask it
mid-flight and a test can ask the same question with a list of fakes. A "pad"
is anything with `id`, `position.x/.y`, `observations`, `is_takeoff_base` and
`visited` — the fields of hydrone_msgs/Pad.

The trap
--------
The pad map was built FROM the pose. Correcting the pose with it feeds the
error straight back in, and a filter that believes its own output walks away
without ever reporting a residual. Three defences, all in this module:

1. **The takeoff base is not from the map.** Its position came from where the
   drone was standing when it armed, which is the one absolute anchor that
   exists — no camera, no projection, no accumulated pose. `ANCHOR_WEIGHT` is
   why it counts for more than everything else put together.
2. **Only settled landmarks.** A pad whose fused position is still moving is
   still being built out of the drifting pose. `LandmarkTracker` watches each
   pad's position across map updates and releases one only once it has stopped
   moving (`settle_tolerance_m`) for `settle_updates` in a row.
3. **Never correct by more than the drift could be.** A correction beyond
   `max_correction_m` is a mis-association, not drift, and applying it is worse
   than applying nothing.

What this deliberately does NOT do
----------------------------------
No yaw. Two landmarks in an 8x8 m arena constrain a heading badly, and heading
is what the VIO now gets from the IMU orientation to within a few degrees.
Estimating it here from a worse source, and then feeding it back, is how a
good yaw becomes a bad one. Translation only, on purpose.
"""

import math

# How many fused sightings before a landmark's recorded position is worth
# trusting as a reference. Matches route.MIN_OBSERVATIONS: one frame of blue
# noise reaches the map, three from different angles do not.
MIN_OBSERVATIONS = 3

# The takeoff base against everything else. Its position is the only one in the
# map that did not come out of the pose being corrected, so it is not merely a
# better landmark — it is a different KIND of evidence, and the weighting says
# so. With one anchor and three ordinary pads the anchor still holds the
# majority (5 against 3).
ANCHOR_WEIGHT = 5.0
PAD_WEIGHT = 1.0

# Beyond this, a "correction" is a mis-association and not drift at all.
# Sized against the arena: 8x8 m, so a 2 m pose error is already severe and a
# 3 m one means the wrong pad was matched.
MAX_CORRECTION_M = 2.0

# A pad's fused position has to stop moving before it is a reference rather
# than a work in progress.
SETTLE_TOLERANCE_M = 0.10
SETTLE_UPDATES = 3


def is_trustworthy(pad, *, min_observations=MIN_OBSERVATIONS):
    """Is this pad's recorded position worth measuring against?

    The takeoff base always is: it was registered from the drone's own armed
    position, so it has an absolute fix regardless of how many times a camera
    has since seen it.
    """
    if getattr(pad, "is_takeoff_base", False):
        return True
    return pad.observations >= min_observations


def weight_of(pad, *, anchor_weight=ANCHOR_WEIGHT, pad_weight=PAD_WEIGHT):
    """How much this landmark's opinion counts. See ANCHOR_WEIGHT."""
    return anchor_weight if getattr(pad, "is_takeoff_base", False) else pad_weight


class LandmarkTracker:
    """Watches pad positions across map updates and releases the settled ones.

    Defence 2 from the module docstring. A pad still being fused moves every
    time a new detection lands; one that has held the same position for several
    updates has stopped being rebuilt out of the drifting pose and can be used
    as a reference.
    """

    def __init__(self, *, settle_tolerance_m=SETTLE_TOLERANCE_M,
                 settle_updates=SETTLE_UPDATES):
        self.settle_tolerance_m = settle_tolerance_m
        self.settle_updates = settle_updates
        self._last = {}        # id -> (x, y)
        self._still = {}       # id -> consecutive updates without moving

    def update(self, pads):
        """Feed one PadMap's worth of pads. Returns the set of settled ids."""
        seen = set()
        for pad in pads:
            pid = int(pad.id)
            seen.add(pid)
            xy = (pad.position.x, pad.position.y)
            prev = self._last.get(pid)
            if prev is None:
                self._still[pid] = 0
            elif math.dist(prev, xy) <= self.settle_tolerance_m:
                self._still[pid] = self._still.get(pid, 0) + 1
            else:
                self._still[pid] = 0
            self._last[pid] = xy
        # A pad that vanished from the map is not settled, it is gone.
        for pid in list(self._last):
            if pid not in seen:
                del self._last[pid]
                self._still.pop(pid, None)
        return self.settled()

    def settled(self):
        """Ids whose position has held still long enough to be a reference."""
        return {pid for pid, n in self._still.items()
                if n >= self.settle_updates}


def drift_from_observations(observations, *, max_correction_m=MAX_CORRECTION_M):
    """Weighted mean pose error from re-observed landmarks.

    `observations` is a sequence of (recorded_xy, observed_xy, weight). The
    return is (dx, dy): what to ADD to the recorded positions to land on the
    observed ones, which is the same quantity as the pose error and the
    negative of the correction to apply to the pose.

    None when there is nothing to say — no observations, no weight, or an
    answer too large to be drift (see MAX_CORRECTION_M). None is the honest
    output: a caller must be able to tell "no correction" apart from "zero
    correction", because the first leaves the estimate alone and the second
    asserts it is already right.
    """
    sx = sy = sw = 0.0
    for recorded, observed, w in observations:
        if w <= 0.0:
            continue
        sx += w * (observed[0] - recorded[0])
        sy += w * (observed[1] - recorded[1])
        sw += w
    if sw <= 0.0:
        return None
    dx, dy = sx / sw, sy / sw
    if math.hypot(dx, dy) > max_correction_m:
        return None
    return (dx, dy)


def anchor_drift(pads, home_xy, *, max_correction_m=MAX_CORRECTION_M):
    """Pose error measured against the takeoff base alone.

    `home_xy` is where the drone actually armed — the absolute truth this whole
    module hangs off. The map's takeoff-base entry starts there and then drifts
    with everything else, so the gap between them is the accumulated error.

    This is step one on purpose: it needs no detection, no settling and no
    association, only the map entry that was registered before any camera
    looked at anything.
    """
    for pad in pads:
        if getattr(pad, "is_takeoff_base", False):
            return drift_from_observations(
                [((home_xy[0], home_xy[1]),
                  (pad.position.x, pad.position.y), ANCHOR_WEIGHT)],
                max_correction_m=max_correction_m)
    return None


# Association gate. A detection further than this from every recorded pad is
# not a re-observation of any of them, and one that is close to TWO of them
# does not say which. Sized above the drift we expect to correct and below the
# spacing the competition rules guarantee between bases.
ASSOCIATION_GATE_M = 1.5


def associate(pads, observed_xy, *, gate_m=ASSOCIATION_GATE_M, eligible=None):
    """Which recorded pad this detection is a re-observation of, or None.

    `eligible` optionally restricts the candidates to a set of ids — that is
    where LandmarkTracker's settled set goes.

    Returns None for a detection that matches nothing, and ALSO for one that
    matches two pads about equally well. A wrong association does not produce a
    small error, it produces a correction the size of the gap between the two
    pads, applied confidently. Refusing an ambiguous match is the cheapest
    protection there is against that.
    """
    ranked = []
    for pad in pads:
        if eligible is not None and int(pad.id) not in eligible:
            continue
        d = math.hypot(pad.position.x - observed_xy[0],
                       pad.position.y - observed_xy[1])
        if d <= gate_m:
            ranked.append((d, pad))
    if not ranked:
        return None
    ranked.sort(key=lambda t: t[0])
    if len(ranked) > 1 and ranked[1][0] < 2.0 * ranked[0][0]:
        return None                    # too close to call
    return ranked[0][1]
