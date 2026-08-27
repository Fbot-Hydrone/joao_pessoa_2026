"""octree: reading the OctoMap from Python.

Built by feeding a real octomap_server a known scene, so what is pinned is the
actual serialization it publishes — not a fixture someone wrote by hand.

    python3 -m pytest src/hydrone_map/test/test_octree.py -q
"""

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

from hydrone_map.octree import (State, is_free, path_is_clear, query,
                                tree_from_msg)

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
