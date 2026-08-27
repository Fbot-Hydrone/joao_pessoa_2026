"""The inertial half of VIO.

The ZED SDK's positional tracking is visual-INERTIAL, not visual: the IMU
carries the motion between frames and the images correct it. Vision has two
failure modes the gyro does not share — it goes blind on a blank wall, and it
cannot tell how far the vehicle turned when every feature leaves the frame at
once. Phase 1 searches by rotating on the spot, which is exactly that case.

    python3 -m pytest src/hydrone_localization/test/test_vo_imu.py -q
"""

import cv2
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import Imu

from hydrone_localization.visual_odometry_node import (R_BASE_FROM_OPTICAL,
                                                       VisualOdometryNode)


@pytest.fixture
def node():
    rclpy.init()
    n = VisualOdometryNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def feed_gyro(node, rate_xyz, t0, t1, hz=200.0):
    """Constant angular rate (rad/s, body frame) from t0 to t1."""
    t = t0
    dt = 1.0 / hz
    while t <= t1 + 1e-9:
        m = Imu()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = rate_xyz
        node._cb_imu(m)
        t += dt


def angle_of(R):
    return float(np.linalg.norm(cv2.Rodrigues(R)[0]))


def test_the_fusion_is_on_by_default(node):
    """It was off for part of 2026-08-27 while the gyro looked broken. The
    field was: biguasim's DynamicsIMUEncoder published angular ACCELERATION
    (indices 9-11) as angular_velocity, where the rate lives at 12-14."""
    assert node.get_parameter("in_imu").value.endswith("imu/data")


def test_a_known_yaw_rate_integrates_to_the_right_angle(node):
    """30 deg/s for 0.4 s is 12 degrees, whatever the camera saw."""
    rate = np.radians(30.0)
    feed_gyro(node, (0.0, 0.0, rate), 100.0, 100.4)
    R = node._imu_rotation(100.0, 100.4)
    assert R is not None
    assert np.degrees(angle_of(R)) == pytest.approx(12.0, abs=0.5)


def test_the_rotation_comes_back_in_the_optical_frame(node):
    """The IMU reports in the body frame and the VO accumulates in the
    camera's. Getting this wrong swaps yaw for roll and is invisible until
    the vehicle turns."""
    rate = np.radians(30.0)
    feed_gyro(node, (0.0, 0.0, rate), 200.0, 200.4)
    R_opt = node._imu_rotation(200.0, 200.4)
    C = R_BASE_FROM_OPTICAL
    R_body = C @ R_opt @ C.T                    # map it back
    axis = cv2.Rodrigues(R_body)[0].ravel()
    assert abs(axis[2]) > 0.99 * np.linalg.norm(axis), "yaw did not stay yaw"


def test_no_samples_means_no_prediction_not_zero_rotation(node):
    """Returning identity would report 'it did not turn', which is a lie the
    pose would then carry forever."""
    assert node._imu_rotation(0.0, 1.0) is None


def test_a_gap_the_imu_did_not_cover_is_refused(node):
    """Samples exist, but not for the interval asked about."""
    feed_gyro(node, (0.0, 0.0, 0.1), 300.0, 300.1)
    assert node._imu_rotation(300.0, 300.0 + node.imu_max_gap + 1.0) is None


def test_the_gyro_carries_the_pose_when_vision_fails(node):
    """The point of fusing: a turn on the spot is not invisible just because
    the wall is blank."""
    rate = np.radians(45.0)
    feed_gyro(node, (0.0, 0.0, rate), 400.0, 400.4)
    R = node._imu_rotation(400.0, 400.4)
    before = node.pose_opt.copy()
    node._carry_on_imu(R)
    assert not np.allclose(node.pose_opt, before)
    assert node.n_imu_carried == 1
    moved = np.linalg.norm(node.pose_opt[:3, 3] - before[:3, 3])
    assert moved == pytest.approx(0.0, abs=1e-9), "translation must NOT be dead-reckoned"


def test_carrying_without_imu_changes_nothing(node):
    before = node.pose_opt.copy()
    node._carry_on_imu(None)
    assert np.allclose(node.pose_opt, before)
    assert node.n_imu_carried == 0


def test_the_disagreement_gate_is_tight_enough_to_catch_bad_matches(node):
    """Over ~0.1 s the gyro's error is a slow bias, not a per-frame blunder,
    so a visual answer many degrees away is bad matching."""
    assert 0 < np.degrees(node.imu_rotation_tol) <= 15.0


def test_translation_is_never_dead_reckoned_from_the_accelerometer(node):
    """Double-integrating acceleration needs gravity removed to milli-g and
    drifts to metres in seconds — worse than admitting position is unknown."""
    import inspect
    src = inspect.getsource(node._carry_on_imu)
    assert "linear_acceleration" not in src
    assert "T[:3, :3] = R_imu" in src


# ── the veto's algebra ───────────────────────────────────────────────────────
#
# Got this backwards once and it cost a flight: PnP returns the rotation of the
# POINTS (T_cur_prev) while the gyro gives the rotation of the CAMERA, and they
# are inverses. Comparing them the wrong way round reports DOUBLE the rotation
# whenever they agree, so every real turn tripped the veto and the correct
# visual answer was discarded. Yaw error went 4.2 deg -> 178.6 deg.

def disagreement(R_imu, R_points):
    """What the node computes: identity when the two describe the same turn."""
    return float(np.linalg.norm(cv2.Rodrigues(R_imu @ R_points)[0]))


def yaw(deg):
    return cv2.Rodrigues(np.array([0.0, 0.0, np.radians(deg)]))[0]


def test_agreement_reads_as_zero(node):
    """A camera that turned +20 deg makes the points turn -20 deg."""
    assert np.degrees(disagreement(yaw(20.0), yaw(20.0).T)) == pytest.approx(0.0, abs=1e-6)


def test_agreement_reads_as_zero_at_every_angle(node):
    """The old bug scaled with the turn, so a small-angle test would miss it."""
    for a in (1.0, 5.0, 20.0, 60.0):
        assert np.degrees(disagreement(yaw(a), yaw(a).T)) == pytest.approx(0.0, abs=1e-6)


def test_real_disagreement_is_reported(node):
    d = np.degrees(disagreement(yaw(20.0), yaw(5.0).T))
    assert d == pytest.approx(15.0, abs=0.5)


def test_the_veto_does_not_fire_on_an_honest_turn(node):
    """The regression itself: a 20 deg turn both sensors agree on must pass."""
    assert disagreement(yaw(20.0), yaw(20.0).T) < node.imu_rotation_tol


def test_the_veto_fires_when_vision_misses_a_turn(node):
    """The case it exists for: gyro saw 9.5 deg, PnP saw 0.1."""
    assert disagreement(yaw(9.5), yaw(0.1).T) > node.imu_rotation_tol


# ─────────────────────────────────────────────────────────────────────────────
# The orientation path
#
# MEASURED 2026-08-27 against BiguaSim ground truth: the simulator's gyro reads
# 2.99x the true rate on every axis, while the orientation quaternion on the
# same message matches ground-truth yaw to 0.00 deg. Integrating a rate
# multiplies the sender's scale error by the length of the flight; differencing
# two orientations does not integrate anything.
# ─────────────────────────────────────────────────────────────────────────────

def quat_about_z(angle):
    return (0.0, 0.0, np.sin(angle / 2.0), np.cos(angle / 2.0))


def feed_orientation(node, angles, t0, t1, rate_xyz=(0.0, 0.0, 0.0), hz=200.0):
    """Orientations sampled from t0 to t1, `angles` a yaw(t) callable.

    `rate_xyz` is what the gyro claims at the same time — deliberately allowed
    to disagree, because in the simulator it does.
    """
    t = t0
    dt = 1.0 / hz
    while t <= t1 + 1e-9:
        m = Imu()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        (m.angular_velocity.x, m.angular_velocity.y,
         m.angular_velocity.z) = rate_xyz
        (m.orientation.x, m.orientation.y,
         m.orientation.z, m.orientation.w) = quat_about_z(angles(t - t0))
        m.orientation_covariance[0] = 0.01
        node._cb_imu(m)
        t += dt


def test_orientation_is_preferred_over_a_gyro_that_disagrees(node):
    """The simulator's gyro is 3x fast. The orientation on the same message is
    exact. Whichever this node believes decides the whole VIO, so pin it."""
    true_yaw = np.radians(12.0)
    feed_orientation(node, lambda dt: true_yaw * (dt / 0.4), 300.0, 300.4,
                     rate_xyz=(0.0, 0.0, np.radians(90.0)))   # 3x too fast
    R = node._imu_rotation(300.0, 300.4)
    assert R is not None
    assert np.degrees(angle_of(R)) == pytest.approx(12.0, abs=0.5), \
        "the 3x gyro was believed over the exact orientation"


def test_orientation_needs_no_dt_so_a_sparse_stream_is_still_exact(node):
    """Two samples 0.4 s apart carry the same answer as eighty. That is the
    point of differencing an angle instead of integrating a rate."""
    true_yaw = np.radians(20.0)
    feed_orientation(node, lambda dt: true_yaw * (dt / 0.4), 400.0, 400.4, hz=5.0)
    R = node._imu_rotation(400.0, 400.4)
    assert np.degrees(angle_of(R)) == pytest.approx(20.0, abs=0.5)


def test_orientation_stays_yaw_in_the_optical_frame(node):
    """Same frame contract as the rate path: body yaw must not come back as
    optical roll."""
    feed_orientation(node, lambda dt: np.radians(30.0) * (dt / 0.4),
                     500.0, 500.4)
    R_opt = node._imu_rotation(500.0, 500.4)
    C = R_BASE_FROM_OPTICAL
    axis = cv2.Rodrigues(C @ R_opt @ C.T)[0].ravel()
    assert abs(axis[2]) > 0.99 * np.linalg.norm(axis), "yaw did not stay yaw"


def test_an_unset_orientation_falls_back_to_the_rate(node):
    """A sender that fills in no orientation leaves the identity quaternion and
    an all-zero covariance behind. Believing that pair would report 'nothing
    turned' forever and the rate path would never run again."""
    feed_gyro(node, (0.0, 0.0, np.radians(30.0)), 600.0, 600.4)
    R = node._imu_rotation(600.0, 600.4)
    assert R is not None
    assert np.degrees(angle_of(R)) == pytest.approx(12.0, abs=0.5)


def test_a_covariance_of_minus_one_falls_back_to_the_rate(node):
    """REP 145: orientation_covariance[0] == -1 means there is no orientation.
    A sender that says so must not have its zeros believed."""
    t = 700.0
    while t <= 700.4 + 1e-9:
        m = Imu()
        m.header.stamp.sec = int(t)
        m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        m.angular_velocity.z = np.radians(30.0)
        # a PERFECTLY VALID quaternion, explicitly disclaimed
        (m.orientation.x, m.orientation.y,
         m.orientation.z, m.orientation.w) = quat_about_z(np.radians(90.0))
        m.orientation_covariance[0] = -1.0
        node._cb_imu(m)
        t += 1.0 / 200.0
    R = node._imu_rotation(700.0, 700.4)
    assert np.degrees(angle_of(R)) == pytest.approx(12.0, abs=0.5), \
        "a disclaimed orientation was used anyway"
