"""octree: reading the OctoMap from Python.

Built by feeding a real octomap_server a known scene, so what is pinned is the
actual serialization it publishes — not a fixture someone wrote by hand.

    python3 -m pytest src/hydrone_map/test/test_octree.py -q
"""

import os
import struct
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField
from octomap_msgs.msg import Octomap

from hydrone_map.octree import (INFLATION_RADIUS_M, State, inflated_state,
                                is_free, is_free_inflated, path_is_clear,
                                path_hits_obstacle, path_is_clear_inflated,
                                query, tree_from_msg)

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)

# A wall at x = 6.0 spanning y in [-3, 3], z in [-0.5, 1.0], seen from a sensor
# at the origin. Everything between is free; everything behind is unknown.
WALL_X = 6.0


def scene():
    pts = [(WALL_X, i * 0.05, k * 0.05 - 0.5)
           for i in range(-60, 61) for k in range(0, 30)]
    buf = b"".join(struct.pack("<fff", *p) for p in pts)
    msg = PointCloud2()
    msg.header.frame_id = "sensor"
    msg.height, msg.width = 1, len(pts)
    msg.fields = [PointField(name=a, offset=o, datatype=7, count=1)
                  for a, o in (("x", 0), ("y", 4), ("z", 8))]
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.is_dense = True
    msg.data = buf
    return msg


@pytest.fixture(scope="module")
def tree():
    """A tree decoded from a live octomap_server, or skip.

    The server is an external process; if it is not on this machine the test
    has nothing to say, and saying it loudly is better than a fake pass.
    """
    import shutil
    import subprocess
    if shutil.which("ros2") is None:
        pytest.skip("ros2 not on PATH")

    procs = [
        subprocess.Popen(
            ["ros2", "run", "tf2_ros", "static_transform_publisher",
             "0", "0", "0.5", "0", "0", "0", "odom", "sensor"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(
            ["ros2", "run", "octomap_server", "octomap_server_node",
             "--ros-args", "-p", "resolution:=0.15", "-p", "frame_id:=odom",
             "-p", "base_frame_id:=sensor", "-p", "sensor_model.max_range:=12.0",
             "-p", "latch:=true"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]
    time.sleep(7)

    rclpy.init()
    node = Node("octree_test")
    got = {}
    node.create_subscription(Octomap, "/octomap_binary",
                             lambda m: got.__setitem__("m", m), LATCHED)
    pub = node.create_publisher(
        PointCloud2, "/cloud_in",
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST))
    cloud = scene()
    for _ in range(15):
        cloud.header.stamp = node.get_clock().now().to_msg()
        pub.publish(cloud)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)
    for _ in range(80):
        rclpy.spin_once(node, timeout_sec=0.05)

    msg = got.get("m")
    node.destroy_node()
    rclpy.shutdown()
    for p in procs:
        p.terminate()

    if msg is None:
        pytest.skip("octomap_server published nothing")
    yield tree_from_msg(msg)


def test_the_message_decodes_into_a_populated_tree(tree):
    """The whole point: /octomap_binary is bytes until this works."""
    assert tree.size() > 0
    assert tree.getNumLeafNodes() > 0
    assert tree.getResolution() == pytest.approx(0.15)


def test_the_wall_reads_as_occupied(tree):
    assert query(tree, (WALL_X, 0.0, 0.6)) == State.OCCUPIED


def test_the_air_in_front_of_the_wall_reads_as_free(tree):
    assert query(tree, (4.0, 0.0, 0.6)) == State.FREE


def test_space_no_ray_reached_reads_as_unknown(tree):
    """Not free. This is the distinction a point cloud cannot make."""
    assert query(tree, (0.0, 9.0, 0.6)) == State.UNKNOWN


def test_behind_the_wall_is_unknown_not_free(tree):
    """The wall stopped the rays, so what is behind it was never measured."""
    assert query(tree, (9.0, 0.0, 0.6)) == State.UNKNOWN


def test_is_free_refuses_unknown_space(tree):
    assert is_free(tree, (4.0, 0.0, 0.6))
    assert not is_free(tree, (0.0, 9.0, 0.6))
    assert not is_free(tree, (WALL_X, 0.0, 0.6))


def test_occupancy_carries_a_probability(tree):
    """log-odds, not a boolean — repeated hits make the wall more certain."""
    node = tree.search(np.array([WALL_X, 0.0, 0.6], dtype=float))
    assert node.getLogOdds() > 0
    air = tree.search(np.array([4.0, 0.0, 0.6], dtype=float))
    assert air.getLogOdds() < 0


def test_a_path_through_open_air_is_clear(tree):
    assert path_is_clear(tree, (1.0, 0.0, 0.6), (4.0, 0.0, 0.6))


def test_a_path_into_the_wall_is_blocked(tree):
    assert not path_is_clear(tree, (1.0, 0.0, 0.6), (WALL_X + 0.3, 0.0, 0.6))


def test_a_path_into_unknown_space_is_blocked(tree):
    """Flying into what was never mapped is how a drone finds a wall."""
    assert not path_is_clear(tree, (1.0, 0.0, 0.6), (0.0, 9.0, 0.6))


def test_full_octomap_is_refused_with_a_clear_message():
    msg = Octomap()
    msg.binary = False
    with pytest.raises(ValueError, match="octomap_full"):
        tree_from_msg(msg)


# ── obstacle inflation ──────────────────────────────────────────────────────
#
# The map answers for a POINT. The drone is 330 mm across, and the gap between
# those two facts is the gap between "a path exists" and "I fit through it".

def test_a_point_beside_the_wall_is_free_but_the_drone_does_not_fit(tree):
    """The whole reason inflation exists, on the real tree: a spot the map
    calls free, one voxel from masonry, that a 330 mm airframe cannot occupy."""
    beside = (WALL_X - 0.15, 0.0, 0.6)
    assert query(tree, beside) == State.FREE
    assert inflated_state(tree, beside) == State.OCCUPIED
    assert not is_free_inflated(tree, beside)


def test_open_air_well_clear_of_the_wall_still_fits(tree):
    """Inflation must not swallow the arena. A metre of clearance is a metre."""
    assert is_free_inflated(tree, (4.0, 0.0, 0.6))


def test_a_leg_down_the_measured_corridor_is_clear_for_the_drone_too(tree):
    """Along the sensor's axis, where the cone of measured free space is
    widest. Off to the side the ball reaches into space no ray covered, and
    that is correctly NOT clear — see the unknown tests above."""
    assert path_is_clear_inflated(tree, (2.0, 0.0, 0.6), (4.0, 0.0, 0.6))


def test_the_wall_itself_is_occupied_when_inflated(tree):
    assert inflated_state(tree, (WALL_X, 0.0, 0.6)) == State.OCCUPIED


def test_inflating_near_unmapped_space_reports_unknown_not_free(tree):
    """Occupied outranks unknown outranks free: a ball that is part measured
    and part never looked at is not a place to fly."""
    assert inflated_state(tree, (WALL_X + 1.0, 0.0, 0.6)) == State.UNKNOWN
    assert not is_free_inflated(tree, (WALL_X + 1.0, 0.0, 0.6))


def test_the_radius_covers_the_airframe_plus_a_margin(tree):
    """330 mm across is 165 mm of radius, and the pose feeding this map is
    itself an estimate. Shrinking this is a decision, not a tidy-up."""
    assert INFLATION_RADIUS_M >= 0.165


def test_a_smaller_radius_lets_the_same_point_through(tree):
    """Pins that the radius is what does the work, not a hidden constant."""
    beside = (WALL_X - 0.15, 0.0, 0.6)
    assert inflated_state(tree, beside, radius=0.01) == State.FREE


def test_a_leg_that_grazes_the_wall_is_clear_for_a_ray_and_not_for_the_drone(tree):
    """The gate the mission actually needs. A straight line that a point could
    fly and a 330 mm airframe could not."""
    a = (2.0, -0.9, 0.6)
    b = (WALL_X - 0.15, -0.9, 0.6)
    assert path_is_clear(tree, a, b), "the ray should fit"
    assert not path_is_clear_inflated(tree, a, b), "the drone should not"




def test_unknown_space_is_not_an_obstacle():
    """The distinction that decides whether the mission's gate means anything.
    Early in a flight nearly every leg crosses space no ray has reached; if
    that counted as an obstruction, every leg would report one."""
    class FakeTree:
        def getResolution(self):
            return 0.15
        def search(self, p):
            return None            # octomap returns a null node for unknown
        def isNodeOccupied(self, node):
            raise RuntimeError     # and throws on it — that throw IS "unknown"
    t = FakeTree()
    assert not path_hits_obstacle(t, (0, 0, 1), (3, 0, 1))
    assert not path_is_clear_inflated(t, (0, 0, 1), (3, 0, 1)), \
        "unknown must still not count as CLEAR"


def test_a_leg_into_the_wall_hits_an_obstacle(tree):
    assert path_hits_obstacle(tree, (2.0, 0.0, 0.6), (WALL_X + 0.5, 0.0, 0.6))


def test_a_leg_through_open_air_hits_nothing(tree):
    assert not path_hits_obstacle(tree, (2.0, -1.0, 0.6), (2.0, 1.0, 0.6))


def test_decoding_does_not_print_to_stderr(tree, capfd):
    """readBinary complains "Tree size mismatch" on every call, from C++, at
    the file-descriptor level. A caller that decodes on a timer buries its own
    log in it — MEASURED at 2 Hz for a flight, ~600 copies, hiding the mission
    state lines that explained a hang."""
    import tempfile as tf
    from octomap_msgs.msg import Octomap

    fd, path = tf.mkstemp(suffix=".bt")
    os.close(fd)
    tree.writeBinary(path.encode())
    with open(path, "rb") as f:
        blob = f.read()
    os.unlink(path)

    msg = Octomap()
    msg.binary = True
    msg.id = "OcTree"
    msg.resolution = tree.getResolution()
    # The message's `data` is int8, so the payload has to be signed.
    payload = blob[blob.index(b"data\n") + 5:]
    msg.data = [b - 256 if b > 127 else b for b in payload]

    capfd.readouterr()                       # discard anything already buffered
    assert tree_from_msg(msg).size() > 0     # it still has to WORK
    out, err = capfd.readouterr()
    assert "Tree size mismatch" not in err
    assert "Tree size mismatch" not in out
