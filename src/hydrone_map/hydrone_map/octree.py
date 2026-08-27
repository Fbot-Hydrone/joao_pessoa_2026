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
