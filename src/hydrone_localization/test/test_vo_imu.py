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


def test_it_subscribes_to_the_imu_by_default(node):
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
