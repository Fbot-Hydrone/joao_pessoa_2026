#!/usr/bin/env python3
"""superlio_bridge: Super-LIO's levelled world re-expressed in FAST-LIO's start frame."""

import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from hydrone_lio.geometry import inv_tf, make_tf, rpy_to_matrix
from hydrone_lio.superlio_bridge import transform_cloud


def _cloud(points):
    """x y z intensity float32, as Super-LIO's PointXYZI cloud (16-byte points)."""
    msg = PointCloud2()
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(['x', 'y', 'z', 'intensity'])]
    msg.point_step, msg.row_step = 16, 16 * len(points)
    msg.data = np.asarray(points, dtype='<f4').tobytes()
    return msg


def _xyz(msg):
    return np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 4)


def test_cloud_moves_by_the_transform_and_keeps_intensity():
    t = make_tf(rpy_to_matrix(0.1, -0.05, math.radians(30)), np.array([1.0, -2.0, 0.5]))
    pts = np.array([[1, 0, 0, 7], [0, 2, 0, 8], [3, -1, 4, 9]], dtype=float)
    out = _xyz(transform_cloud(_cloud(pts), t))
    expect = pts[:, :3] @ t[:3, :3].T + t[:3, 3]
    assert np.allclose(out[:, :3], expect, atol=1e-5)
    assert np.array_equal(out[:, 3], pts[:, 3])


def test_start_pose_becomes_identity_and_motion_is_kept():
    # drone tilted at start: Super-LIO's world -> imu is that tilt, FAST-LIO's is I
    t0 = make_tf(rpy_to_matrix(0.03, -0.02, 0.0), np.zeros(3))
    step = make_tf(rpy_to_matrix(0.0, 0.0, math.radians(90)), np.array([2.0, 0.0, 1.0]))
    t1 = t0 @ step                     # moved by `step` in its own start frame
    assert np.allclose(inv_tf(t0) @ t0, np.eye(4))
    assert np.allclose(inv_tf(t0) @ t1, step)
    # a world point seen at start lands where the start frame sees it
    p_w = np.array([5.0, 1.0, -0.5, 1.0])
    out = _xyz(transform_cloud(_cloud([list(p_w[:3]) + [0]]), inv_tf(t0)))
    assert np.allclose(out[0, :3], (inv_tf(t0) @ p_w)[:3], atol=1e-5)
