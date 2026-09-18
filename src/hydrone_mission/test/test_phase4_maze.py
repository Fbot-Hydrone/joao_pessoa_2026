"""The ROS wrapper's own helpers. The mission logic is tested in test_maze_*.py."""
import numpy as np

from hydrone_mission.phase4_maze_node import cloud_xyz, odom_to_enu, yaw_of


class _Field:
    def __init__(self, name, offset):
        self.name, self.offset = name, offset


class _Cloud:
    """Just enough PointCloud2 for cloud_xyz: xyz float32 at 0/4/8."""
    def __init__(self, pts, extra=4):
        self.point_step = 12 + extra
        self.fields = [_Field('x', 0), _Field('y', 4), _Field('z', 8)]
        rows = [np.frombuffer(np.asarray(p, dtype=np.float32).tobytes(), dtype=np.uint8)
                for p in pts]
        self.data = b''.join(r.tobytes() + b'\0' * extra for r in rows)


class _Quat:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z


def test_cloud_xyz_reads_points_and_strides():
    pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0), (10.0, 11.0, 12.0)]
    out = cloud_xyz(_Cloud(pts))
    assert out.shape == (4, 3)
    assert np.allclose(out[2], [7, 8, 9])
    assert np.allclose(cloud_xyz(_Cloud(pts), stride=2), [[1, 2, 3], [7, 8, 9]])


def test_odom_to_enu_is_the_bridge_rotation():
    # vision_odom_bridge publishes the odom pose rotated +90 deg about z
    assert np.allclose(odom_to_enu([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0])
    assert np.allclose(odom_to_enu([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0])
    assert np.allclose(odom_to_enu([0.0, 0.0, 0.3]), [0.0, 0.0, 0.3])


def test_yaw_of_quaternion():
    assert yaw_of(_Quat(1.0, 0.0, 0.0, 0.0)) == 0.0
    assert np.isclose(yaw_of(_Quat(np.cos(0.4), 0.0, 0.0, np.sin(0.4))), 0.8)
