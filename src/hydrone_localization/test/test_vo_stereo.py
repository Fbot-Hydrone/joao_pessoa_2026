"""Stereo depth in visual_odometry_node.

The ZED 2i is a stereo camera: its tracker gets the range of the features it
follows by MATCHING its two eyes, not by reading a depth image someone else
computed. Consuming the sim's depth was a shortcut the real drone does not
have, and it hid the errors the real one makes — a blank wall the right eye
cannot disambiguate has NO range, where the sim's depth image handed over a
perfect number.

This is for ODOMETRY ONLY. The mapping stack, the pad detectors and odom_GT
keep using the sim's depth image exactly as before.

    python3 -m pytest src/hydrone_localization/test/test_vo_stereo.py -q
"""

import cv2
import numpy as np
import pytest
import rclpy

from hydrone_localization.visual_odometry_node import VisualOdometryNode

# The sim pair, measured from its own camera_info.
FX = 320.0
BASELINE = 0.12


@pytest.fixture
def node():
    rclpy.init()
    n = VisualOdometryNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_the_sparse_matcher_is_configured(node):
    """Sparse feature matching, NOT a dense disparity map. MEASURED: of 981
    ORB keypoints on a live frame, dense SGBM had disparity for only 267 —
    73% of the features the tracker wanted were discarded, because a corner
    detector puts keypoints exactly where a dense matcher does worst."""
    assert node.matcher is not None
    assert node.stereo_row_tol > 0
    assert node.min_disparity >= 1.0


def test_it_subscribes_to_the_right_eye_by_default(node):
    assert node.get_parameter("in_right").value.endswith("right/image_rect_color")


def test_the_baseline_is_read_from_p3_not_hardcoded(node):
    """REP 104 puts it at P[3] = -fx * B, which is where every stereo consumer
    in ROS looks. Hardcoding it would silently scale every depth."""
    from sensor_msgs.msg import CameraInfo
    info = CameraInfo()
    info.p = [FX, 0.0, 320.0, -FX * BASELINE,
              0.0, FX, 240.0, 0.0,
              0.0, 0.0, 1.0, 0.0]
    node._cb_right_info(info)
    assert node.baseline == pytest.approx(BASELINE)


def test_a_zero_focal_length_does_not_divide_by_zero(node):
    from sensor_msgs.msg import CameraInfo
    info = CameraInfo()
    info.p = [0.0] * 12
    node._cb_right_info(info)          # must not raise
    assert node.baseline is None


# ── the physics, which is what bounds max_depth ──────────────────────────────

def stereo_error_at(z, disparity_error_px=1.0, fx=FX, b=BASELINE):
    """dZ = Z^2 * dd / (fx * B) — triangulation error grows as the SQUARE."""
    return z ** 2 * disparity_error_px / (fx * b)


def test_stereo_error_grows_with_the_square_of_range():
    assert stereo_error_at(10.0) > 3 * stereo_error_at(5.0)


def test_max_depth_is_inside_the_range_stereo_can_actually_measure(node):
    """MEASURED against the sim's depth: 0.305 m error at 1-3 m, 2.535 m at
    3-6 m, 1.864 m at 6-12 m. Accepting features at 20 m let the far half of
    the arena vote on the pose with metres of range error."""
    assert node.max_depth <= 8.0
    # And one pixel of disparity error at the bound stays under a metre.
    assert stereo_error_at(node.max_depth, 0.5) < 1.0


def test_the_old_bound_would_have_been_useless():
    """At 20 m a single pixel of disparity is worth over 10 m."""
    assert stereo_error_at(20.0) > 10.0


# ── matching ─────────────────────────────────────────────────────────────────

def stereo_pair(z=3.0, w=320, h=240, fx=FX, b=BASELINE, seed=0):
    """A textured plane at range z, and the right eye's shifted view of it."""
    rng = np.random.default_rng(seed)
    left = rng.integers(0, 255, (h, w), dtype=np.uint8)
    shift = int(round(fx * b / z))
    right = np.roll(left, -shift, axis=1)
    right[:, -shift:] = 0
    return left, right, shift


def test_a_known_shift_comes_back_as_the_right_depth(node):
    """The whole chain: match the pair, disparity -> Z = fx * B / d."""
    z = 3.0
    left, right, _shift = stereo_pair(z=z)
    node.K = np.array([[FX, 0, 160], [0, FX, 120], [0, 0, 1]], dtype=float)
    node.baseline = BASELINE
    node.last_right = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
    node.clahe = None                      # the pair is already textured
    kp, des = node.orb.detectAndCompute(left, None)
    depths = node._stereo_match(kp, des, left)
    got = depths[np.isfinite(depths)]
    assert len(got) > 20, "almost nothing matched on a textured pair"
    assert np.median(got) == pytest.approx(z, rel=0.15)


def test_unmatchable_scene_yields_no_range_rather_than_a_guess(node):
    """A flat grey pair has no correspondence to find. Returning NaN is the
    honest answer — handing back a confident number is exactly what the sim's
    depth image was doing and what the real drone never gets."""
    flat = np.full((240, 320), 200, np.uint8)
    node.K = np.array([[FX, 0, 160], [0, FX, 120], [0, 0, 1]], dtype=float)
    node.baseline = BASELINE
    node.last_right = cv2.cvtColor(flat, cv2.COLOR_GRAY2BGR)
    node.clahe = None
    kp, des = node.orb.detectAndCompute(flat, None)
    depths = node._stereo_match(kp, des, flat)
    assert depths is None or not np.isfinite(depths).any()


def test_the_range_is_aligned_with_the_keypoints(node):
    """_backproject relies on it: one depth per keypoint, same order."""
    left, right, _ = stereo_pair(z=3.0)
    node.K = np.array([[FX, 0, 160], [0, FX, 120], [0, 0, 1]], dtype=float)
    node.baseline = BASELINE
    node.last_right = cv2.cvtColor(right, cv2.COLOR_GRAY2BGR)
    node.clahe = None
    kp, des = node.orb.detectAndCompute(left, None)
    assert len(node._stereo_match(kp, des, left)) == len(kp)


def test_a_match_on_a_different_row_is_rejected(node):
    """The pair is rectified: a partner off the epipolar line is a wrong
    match, and it is free to reject."""
    assert node.stereo_row_tol < 5.0


def test_tiny_disparities_are_rejected(node):
    """dZ/Z = dd/d — at 1 px of matching error a 4 px disparity is already
    25% range error, and it only gets worse."""
    assert node.min_disparity >= 4.0


def test_both_eyes_get_the_same_equalisation(node):
    """SGBM matches intensities. Equalising one eye and not the other would
    break every correspondence — a silent, total failure."""
    import inspect
    src = inspect.getsource(node._stereo_match)
    assert "self.clahe.apply(right)" in src
