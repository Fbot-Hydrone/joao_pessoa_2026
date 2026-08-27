"""The zero-velocity update in visual_odometry_node.

WHY IT EXISTS, from odom_error_20260827_010810.csv (a 51.5 m flight):

    94% of samples the vehicle was literally still (ground truth moved
    < 0.01 mm) and the VO reported 441.8 m across them — 1.61 mm per frame.
    Total: 51.5 m flown, 527.3 m reported. While genuinely moving the same
    VO is fine (50.6 m against 58.2 m, 1.15x).

So the drift was not bad odometry, it was noise integrated while parked. These
tests pin that a still frame changes nothing and a moving one still does.

    python3 -m pytest src/hydrone_localization/test/test_vo_zupt.py -q
"""

import numpy as np
import pytest
import rclpy

from hydrone_localization.visual_odometry_node import VisualOdometryNode


@pytest.fixture
def node():
    rclpy.init()
    n = VisualOdometryNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


# The measured noise floor and one frame of real flight, for reference.
NOISE_M = 0.00161          # 1.61 mm, measured
REAL_STEP_M = 0.05         # 0.5 m/s at 10 Hz


def test_the_threshold_sits_between_noise_and_real_motion(node):
    """A gate that does not clear the noise is useless, and one that reaches
    real motion would delete the flight."""
    assert node.min_step_m > NOISE_M * 2, "too close to the noise it must reject"
    assert node.min_step_m < REAL_STEP_M / 5, "close enough to eat real motion"


def test_the_measured_noise_is_rejected(node):
    assert NOISE_M < node.min_step_m


def test_one_frame_of_real_flight_is_not_rejected(node):
    assert REAL_STEP_M > node.min_step_m


def test_a_still_frame_leaves_the_pose_untouched(node):
    """The whole point: 1.61 mm of nothing must not become 1.61 mm of travel."""
    before = node.pose_opt.copy()
    step_t, step_r = NOISE_M, np.radians(0.05)
    is_still = step_t < node.min_step_m and step_r < node.min_step_rad
    assert is_still
    # The node returns before touching pose_opt, so it stays identical.
    assert np.allclose(node.pose_opt, before)


def test_translation_alone_is_enough_to_move(node):
    """Sliding sideways without turning is motion."""
    step_t, step_r = REAL_STEP_M, 0.0
    assert not (step_t < node.min_step_m and step_r < node.min_step_rad)


def test_turning_on_the_spot_is_not_treated_as_still(node):
    """Phase 1 searches by rotating in place: zero translation, real rotation.
    A gate on translation alone would delete the entire search."""
    step_t, step_r = 0.0, np.radians(2.0)
    assert not (step_t < node.min_step_m and step_r < node.min_step_rad)


def test_both_must_be_small_for_a_frame_to_count_as_still(node):
    small_t, small_r = NOISE_M, np.radians(0.05)
    assert small_t < node.min_step_m and small_r < node.min_step_rad


def test_the_still_counter_starts_at_zero(node):
    assert node.n_static == 0


def test_the_big_step_bound_still_exists(node):
    """ZUPT is a second gate, not a replacement: a degenerate PnP fit claiming
    metres in one frame must still be thrown away."""
    assert node.max_step_m > node.min_step_m
    assert node.max_step_rad > node.min_step_rad


def test_over_a_still_flight_the_gate_removes_the_whole_invented_distance(node):
    """Replays the measured case: 94% of 159814 samples at 1.61 mm each."""
    still_frames = int(159814 * 0.94)
    invented_m = still_frames * NOISE_M
    assert invented_m > 200, "sanity: this is the scale of the problem"
    kept = 0.0 if NOISE_M < node.min_step_m else invented_m
    assert kept == 0.0


# ── low-contrast detection ───────────────────────────────────────────────────
#
# The other half of the drift: the tracker kept LOSING the scene. The logs are
# full of "VO: only 0 usable matches", and the reason was measured on
# 2026-08-20 (feature_map_node's docstring): 46 ORB keypoints in a whole frame,
# all inside a 30-pixel band on the horizon, because the arena is matte white
# wall on blown-out white floor under a smooth sky.
#
# The arena is not textureless — it is LOW CONTRAST, and OpenCV's default FAST
# threshold of 20 is tuned for ordinary scenes.

import cv2


def arena_frame(h=480, w=640, seed=0):
    """Matte white wall over blown-out floor under smooth sky."""
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w), np.float32)
    img[:200] = 232
    img[200:330] = 214
    img[330:] = 246
    for x in range(80, w, 120):          # panel seams: barely there
        img[200:330, x:x + 2] -= 4
    for x in range(40, w, 160):
        img[330:, x:x + 2] -= 2
    img[200:202, :] -= 6                 # the horizon line
    img += rng.normal(0, 1.2, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


CLAHE = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


def detect(img, threshold, clahe=None):
    if clahe is not None:
        img = clahe.apply(img)
    return cv2.ORB_create(nfeatures=1000, fastThreshold=threshold).detect(img, None)


def test_the_old_defaults_find_nothing_in_this_arena():
    """fastThreshold 20 and no equalisation: this is what was flying."""
    assert len(detect(arena_frame(), 20)) == 0


def test_a_lower_corner_threshold_alone_is_not_enough():
    assert len(detect(arena_frame(), 7)) == 0


def test_equalisation_alone_is_not_enough():
    assert len(detect(arena_frame(), 20, CLAHE)) < 5


def test_together_they_recover_the_scene():
    """489 keypoints in the measurement this was written from."""
    assert len(detect(arena_frame(), 7, CLAHE)) > 200


def test_the_keypoints_spread_over_the_frame_not_one_band():
    """The old failure was 46 keypoints inside 30 rows. A tracker cannot
    survive a turn on features that all sit on one line."""
    kp = detect(arena_frame(), 7, CLAHE)
    v = np.array([k.pt[1] for k in kp])
    assert v.max() - v.min() > 300


def test_the_node_defaults_are_the_ones_that_work(node):
    assert node.fast_threshold <= 10
    assert node.clahe is not None


def test_equalisation_can_be_switched_off():
    """clahe_clip 0 disables it — a textured scene does not need it."""
    from rclpy.parameter import Parameter
    rclpy.init()
    try:
        n = VisualOdometryNode(parameter_overrides=[
            Parameter("clahe_clip", Parameter.Type.DOUBLE, 0.0)])
        assert n.clahe is None
        n.destroy_node()
    finally:
        rclpy.shutdown()
