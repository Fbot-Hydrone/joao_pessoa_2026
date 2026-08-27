"""octree — reading the OctoMap from Python, so a planner can ask it things.

octomap_server publishes the tree on /octomap_binary as an opaque byte stream.
ROS's own converters for it (octomap_msgs::binaryMsgToMap) are C++ ONLY: there
is no rclpy equivalent, so a Python node subscribing to that topic gets bytes
it cannot ask a single question of. This module closes that gap.

    from hydrone_map.octree import tree_from_msg, State

    tree = tree_from_msg(msg)               # octomap.OcTree
    state = query(tree, (1.2, -0.4, 0.9))   # OCCUPIED / FREE / UNKNOWN

UNKNOWN is the reason to bother. An accumulated point cloud has two answers:
there is a point here, or there is not — and "no point" covers both "I looked
and it is empty" and "I have never looked". A planner must not treat those the
same: one is a corridor, the other is a guess. The octree separates them, and
carries how sure it is as log-odds.

The temporary file
------------------
octomap-python's OcTree.readBinary() takes a FILENAME, not a buffer, so the
message is written to a temp file with the header the reader expects. That is
ugly and it is also cheap: the whole tree of an 8x8 m arena at 0.15 m is a few
KB. Decode when a plan is needed, not per frame.

The header's `size` field cannot be known before reading, so it is written as 1
and the resulting "Tree size mismatch" line on stderr is EXPECTED — readBinary
returns False and still populates the tree correctly (VERIFIED: 6439 nodes,
4960 leaves from a stream written with size 1). We therefore ignore its return
value and check tree.size() instead, which is the honest test of success.
"""

import os
import tempfile

import numpy as np
import octomap


class State:
    OCCUPIED = "occupied"
    FREE = "free"
    UNKNOWN = "unknown"


def tree_from_msg(msg) -> octomap.OcTree:
    """octomap_msgs/Octomap (binary) -> a queryable OcTree.

    Raises ValueError if the stream yields an empty tree, which is what a
    caller wants to hear rather than silently planning through a blank map.
    """
    if not msg.binary:
        raise ValueError(
            "octree: expected /octomap_binary; /octomap_full is a different "
            "serialization that readBinary cannot parse")

    fd, path = tempfile.mkstemp(suffix=".bt")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(b"# Octomap OcTree binary file\n")
            f.write(f"id {msg.id}\n".encode())
            f.write(b"size 1\n")                    # see module docstring
            f.write(f"res {msg.resolution}\n".encode())
            f.write(b"data\n")
            f.write(bytes(msg.data))

        tree = octomap.OcTree(msg.resolution)
        tree.readBinary(path.encode())              # return value is unreliable
    finally:
        os.unlink(path)

    if tree.size() == 0:
        raise ValueError("octree: decoded an empty tree from a non-empty message")
    return tree


def query(tree, point) -> str:
    """State of the voxel containing `point` (x, y, z) in the map frame."""
    node = tree.search(np.asarray(point, dtype=float))
    try:
        return State.OCCUPIED if tree.isNodeOccupied(node) else State.FREE
    except Exception:
        # search() returns a null node for space no ray has ever reached, and
        # isNodeOccupied on it throws. That throw IS the unknown answer.
        return State.UNKNOWN


def is_free(tree, point) -> bool:
    """True only for space measured as empty. Unknown is NOT free.

    The distinction is the whole point: flying through unknown space because
    nothing said otherwise is how a drone finds a wall it never mapped.
    """
    return query(tree, point) == State.FREE


def path_is_clear(tree, start, end, step=None) -> bool:
    """Is every voxel along the straight segment measured free?

    Samples at half the tree's resolution so no voxel between the endpoints is
    stepped over. Unknown counts as blocked, for the reason in is_free.
    """
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    dist = float(np.linalg.norm(b - a))
    if dist == 0.0:
        return is_free(tree, a)

    step = step or tree.getResolution() / 2.0
    n = max(1, int(np.ceil(dist / step)))
    for i in range(n + 1):
        if not is_free(tree, a + (b - a) * (i / n)):
            return False
    return True


# The drone is 330 mm across. Half of that is the radius a path has to keep
# clear of every occupied voxel, plus a margin for the fact that the pose
# feeding this map is itself an estimate.
DRONE_RADIUS_M = 0.165
SAFETY_MARGIN_M = 0.10
INFLATION_RADIUS_M = DRONE_RADIUS_M + SAFETY_MARGIN_M


def inflated_state(tree, point, radius=INFLATION_RADIUS_M) -> str:
    """State of the space a body of `radius` would occupy at `point`.

    The map answers for a point. A drone is not a point, and the difference is
    the difference between "a path exists" and "I fit through it": an occupancy
    map will happily route a 330 mm airframe through a 150 mm voxel gap,
    because nothing in the map knows the airframe has a size. Every serious
    planner inflates obstacles by the robot's radius before planning, and this
    is where that happens.

    OCCUPIED if any voxel within `radius` is occupied — one wall voxel is
    enough, no matter what surrounds it. Otherwise UNKNOWN if any is unknown,
    and FREE only when the whole ball has been measured empty. The ordering is
    deliberate: occupied is a fact that outranks ignorance, and ignorance
    outranks a partial measurement of free.

    Sampled on the tree's own grid — anything finer asks the same voxel twice
    and anything coarser steps over one. A radius of 0.275 m on a 0.15 m tree
    is 5x5x5 = 125 queries per call, which is why a planner should call this
    once per grid cell and remember the answer, not once per edge.
    """
    p = np.asarray(point, dtype=float)
    res = tree.getResolution()
    n = int(np.ceil(radius / res))
    r2 = radius * radius
    saw_unknown = False
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            for k in range(-n, n + 1):
                off = np.array([i, j, k], dtype=float) * res
                if float(off @ off) > r2:
                    continue            # the ball, not the box
                s = query(tree, p + off)
                if s == State.OCCUPIED:
                    return State.OCCUPIED
                if s == State.UNKNOWN:
                    saw_unknown = True
    return State.UNKNOWN if saw_unknown else State.FREE


def is_free_inflated(tree, point, radius=INFLATION_RADIUS_M) -> bool:
    """True only where a body of `radius` is measured to fit. Unknown is not.

    Same refusal as is_free and for the same reason: flying a 330 mm drone
    into space nothing has looked at, because nothing said otherwise, is how it
    finds a wall it never mapped.
    """
    return inflated_state(tree, point, radius) == State.FREE


def path_is_clear_inflated(tree, start, end, radius=INFLATION_RADIUS_M,
                           step=None) -> bool:
    """path_is_clear, but for a body of `radius` rather than a point.

    This is the one a planner and a mission should call. `path_is_clear` asks
    whether a ray fits; a drone is not a ray, and the difference decides
    whether "clear" means anything.

    Steps at the tree's resolution rather than half of it: each sample already
    covers a ball of `radius`, which is wider than a voxel, so half-resolution
    stepping would re-ask overlapping balls for nothing. Unknown counts as
    blocked, for the reason in is_free.
    """
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    dist = float(np.linalg.norm(b - a))
    if dist == 0.0:
        return is_free_inflated(tree, a, radius)

    step = step or tree.getResolution()
    n = max(1, int(np.ceil(dist / step)))
    for i in range(n + 1):
        if not is_free_inflated(tree, a + (b - a) * (i / n), radius):
            return False
    return True


def path_hits_obstacle(tree, start, end, radius=INFLATION_RADIUS_M,
                       step=None) -> bool:
    """Does this leg run a body of `radius` into something MEASURED occupied?

    The counterpart to path_is_clear_inflated, and the difference between them
    is unknown space. "Clear" demands the whole leg be measured empty, which in
    a half-explored arena is almost never true — early in a flight nearly every
    leg crosses space no ray has reached, and treating that as blocked would
    report an obstruction on every leg and mean nothing.

    This asks the narrower question a mission actually needs: is there
    something IN THE WAY. Unknown is not an obstacle here, it is an absence of
    evidence — which is exactly the distinction the caller has to make, because
    refusing to fly through unknown space would ground the vehicle at takeoff.
    """
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    dist = float(np.linalg.norm(b - a))
    step = step or tree.getResolution()
    n = max(1, int(np.ceil(dist / step))) if dist > 0 else 0
    for i in range(n + 1):
        p = a if n == 0 else a + (b - a) * (i / n)
        if inflated_state(tree, p, radius) == State.OCCUPIED:
            return True
    return False
