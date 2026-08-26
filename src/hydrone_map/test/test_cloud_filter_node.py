"""cloud_filter_node: the cloud that reaches an occupancy map.

Two things are pinned here and neither is about numpy — cloud_filter's own
tests cover the filtering. These are about what the NODE hands octomap_server:

  * the header survives untouched, so the ray origin is looked up from the
    sensor's frame at the sensor's time. Fold the cloud into `odom` here and
    every ray is cast from the world origin instead.
  * the flying pixel does not reach the output.

    python3 -m pytest src/hydrone_map/test/test_cloud_filter_node.py -q
"""

import struct

import numpy as np
import pytest
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from hydrone_map.cloud_filter_node import CloudFilterNode

QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.BEST_EFFORT)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_cloud(pts_hw3, frame_id="zed_left_camera_frame", sec=17, nanosec=42):
    pts = np.asarray(pts_hw3, dtype=np.float32)
    h, w, _ = pts.shape
    buf = np.zeros((h * w, 4), dtype=np.float32)   # x, y, z, rgb — like the ZED
    buf[:, :3] = pts.reshape(h * w, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.height, msg.width = h, w
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * w
    msg.is_dense = True
    msg.data = buf.tobytes()
    return msg


def read_xyz(msg):
    n = msg.width * msg.height
    return np.array(struct.unpack(f"<{n * 3}f", msg.data)).reshape(n, 3)


def run(node_params, cloud, timeout_s=3.0):
    """Publish one cloud into the node, return what it published (or None)."""
    node = CloudFilterNode(parameter_overrides=node_params)
    sink = rclpy.create_node("sink")
    got = []
    sink.create_subscription(PointCloud2, "/hydrone/map/cloud_filtered",
                             got.append, QOS)
    pub = sink.create_publisher(PointCloud2, node.get_parameter("in_cloud").value, QOS)

    import time
    deadline = time.time() + timeout_s
    while time.time() < deadline and not got:
        pub.publish(cloud)
        rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(sink, timeout_sec=0.05)

    node.destroy_node()
    sink.destroy_node()
    return got[0] if got else None


def params(**kw):
    from rclpy.parameter import Parameter
    base = {"stride": 1, "max_depth": 20.0}
    base.update(kw)
    out = []
    for k, v in base.items():
        t = (Parameter.Type.INTEGER if isinstance(v, int) and not isinstance(v, bool)
             else Parameter.Type.BOOL if isinstance(v, bool)
             else Parameter.Type.DOUBLE if isinstance(v, float)
             else Parameter.Type.STRING)
        out.append(Parameter(k, t, v))
    return out


def wall(h, w, distance):
    pts = np.zeros((h, w, 3), dtype=np.float32)
    pts[:, :, 2] = distance
    return pts


def test_the_header_reaches_octomap_untouched(ros):
    """The ray origin depends on it: same frame, same stamp."""
    out = run(params(), make_cloud(wall(8, 8, 4.0), frame_id="zed_left_camera_frame"))
    assert out is not None
    assert out.header.frame_id == "zed_left_camera_frame"
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (17, 42)


def test_a_flying_pixel_never_reaches_the_occupancy_map(ros):
    """The measured case: wall at 4.86 m, edge pixel claiming 18.07 m.

    That one point would carve a free-space tunnel through the wall it belongs
    to, so its absence here is the whole reason this node exists.
    """
    w = wall(9, 9, 4.86)
    w[4, 4, 2] = 18.07
    out = run(params(), make_cloud(w))
    assert out is not None
    assert not (read_xyz(out)[:, 2] > 5.0).any()


def test_the_output_is_a_flat_xyz_cloud(ros):
    out = run(params(), make_cloud(wall(8, 8, 4.0)))
    assert out.height == 1
    assert out.point_step == 12
    assert [f.name for f in out.fields] == ["x", "y", "z"]
    assert out.width == 64


def test_points_past_max_depth_are_dropped_before_the_rays(ros):
    """The arena's diagonal is 11.3 m; a longer ray is the priciest thing
    octomap can be handed, and there is nothing real out there."""
    near = wall(8, 8, 4.0)
    near[:, 4:, 2] = 15.0
    out = run(params(max_depth=12.0, max_edge_step=100.0), make_cloud(near))
    assert out is not None
    assert read_xyz(out)[:, 2].max() < 12.0
