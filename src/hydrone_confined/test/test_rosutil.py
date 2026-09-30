"""Message conversions: the raw Livox CustomMsg reader against the real serializer."""
import numpy as np
import pytest

from hydrone_confined.nodes.rosutil import enu_to_odom, livox_custom_xyz, odom_to_enu, quat_to_rpy, yaw_to_quat


@pytest.mark.parametrize('frame', ['livox_frame', 'x', ''])
@pytest.mark.parametrize('n', [0, 1, 7, 2000])
def test_livox_custom_raw_matches_serializer(frame, n):
    livox = pytest.importorskip('livox_ros_driver2.msg')
    ser = pytest.importorskip('rclpy.serialization')
    rng = np.random.default_rng(n)
    pts = rng.uniform(-20, 20, (n, 3)).astype(np.float32)
    m = livox.CustomMsg()
    m.header.stamp.sec, m.header.stamp.nanosec = 123, 456
    m.header.frame_id = frame
    m.timebase = 987654321
    m.point_num = n
    m.lidar_id = 3
    for k, (x, y, z) in enumerate(pts):
        p = livox.CustomPoint()
        p.offset_time, p.x, p.y, p.z = k * 10, float(x), float(y), float(z)
        p.reflectivity, p.tag, p.line = 7, 1, k % 4
        m.points.append(p)
    out = livox_custom_xyz(ser.serialize_message(m))
    assert out.shape == (n, 3)
    assert np.array_equal(out, pts)


def test_frame_helpers():
    v = np.array([1.0, 2.0, 0.3])
    assert np.allclose(enu_to_odom(odom_to_enu(v)), v)
    r, p, y = quat_to_rpy(*yaw_to_quat(0.7))
    assert abs(r) < 1e-12 and abs(p) < 1e-12 and y == pytest.approx(0.7)
