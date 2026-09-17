import math

import numpy as np

from hydrone_lio.geometry import matrix_to_quat, quat_to_matrix, rpy_to_matrix
from hydrone_lio.motion_prior_node import G, check, predict_displacement, tilt_only

K, M = 1.9e-6, 2.0
HOVER_SQ = M * G / K     # sum of omega^2 that holds a hover


def samples(sum_sq, r, t0=0.0, t1=0.5, n=26):
    return [(t, sum_sq, r) for t in np.linspace(t0, t1, n)]


def test_hover_predicts_constant_velocity():
    d = predict_displacement(samples(HOVER_SQ, np.eye(3)), [1.0, 0, 0], 0.0, 0.5, K, M)
    assert np.allclose(d, [0.5, 0, 0], atol=1e-6)


def test_motors_off_is_free_fall():
    d = predict_displacement(samples(0.0, np.eye(3)), [0, 0, 0], 0.0, 0.5, K, M)
    assert math.isclose(d[2], -0.5 * G * 0.25, rel_tol=0.05)


def test_pitch_forward_accelerates_forward():
    # nose down (positive pitch in FLU) tilts thrust forward
    r = rpy_to_matrix(0.0, math.radians(10), 0.0)
    d = predict_displacement(samples(HOVER_SQ / math.cos(math.radians(10)), r), [0, 0, 0],
                             0.0, 0.5, K, M)
    assert d[0] > 0.1 and abs(d[2]) < 0.01


def test_tilt_only_drops_yaw():
    r = rpy_to_matrix(0.1, 0.2, 1.3)
    t = tilt_only(r)
    assert np.allclose(t, rpy_to_matrix(0.1, 0.2, 0.0), atol=1e-9)


def test_check_margins():
    assert check([0.52, 0, 0], [0.5, 0, 0], 0.1, 0.5)[0]
    assert not check([-0.5, 0, 0], [0.5, 0, 0], 0.1, 0.5)[0]


def test_quat_roundtrip():
    r = rpy_to_matrix(0.3, -0.7, 2.9)
    assert np.allclose(quat_to_matrix(*matrix_to_quat(r)), r, atol=1e-9)
