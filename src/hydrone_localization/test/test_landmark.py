"""Landmark localisation: measuring the pose error from a re-observed pad.

Pure functions over fakes — no ROS, no simulator. The point of keeping
hydrone_localization.landmark free of rclpy is that these run in a second.

    python3 -m pytest src/hydrone_localization/test/test_landmark.py -q
"""

import pytest

from hydrone_localization.landmark import (ANCHOR_WEIGHT, LandmarkTracker,
                                           anchor_drift,
                                           drift_from_observations,
                                           is_trustworthy, weight_of)


class P:
    """The fields of hydrone_msgs/Pad that this module reads."""

    def __init__(self, pid, x, y, observations=9, is_takeoff_base=False):
        self.id = pid
        self.position = type("pt", (), {"x": x, "y": y, "z": 0.0})()
        self.observations = observations
        self.is_takeoff_base = is_takeoff_base


# ── which landmarks may be measured against ─────────────────────────────────

def test_a_pad_seen_once_is_not_a_reference():
    """One frame of blue noise reaches the map. Correcting a pose against it
    would inject that noise into the estimate the vehicle flies on."""
    assert not is_trustworthy(P(1, 0, 0, observations=1))


def test_a_pad_seen_enough_times_is():
    assert is_trustworthy(P(1, 0, 0, observations=3))


def test_the_takeoff_base_is_trusted_with_no_sightings_at_all():
    """Its position did not come from a camera. It came from where the drone
    was standing when it armed, which no amount of drift can have touched."""
    assert is_trustworthy(P(0, 0, 0, observations=0, is_takeoff_base=True))


def test_the_anchor_outweighs_a_crowd_of_ordinary_pads():
    """It is not a better landmark, it is a different kind of evidence: the one
    position in the map that was not built out of the pose being corrected."""
    anchor = weight_of(P(0, 0, 0, is_takeoff_base=True))
    assert anchor > 3 * weight_of(P(1, 0, 0))


# ── the measurement ─────────────────────────────────────────────────────────

def test_the_difference_is_the_pose_error():
    """The example from the module docstring."""
    d = drift_from_observations([((2.02, -3.24), (2.41, -3.02), 1.0)])
    assert d == pytest.approx((0.39, 0.22), abs=1e-9)


def test_landmarks_that_disagree_average_out_by_weight():
    d = drift_from_observations([((0, 0), (1.0, 0.0), 1.0),
                                 ((5, 5), (5.0, 5.0), 3.0)])
    assert d == pytest.approx((0.25, 0.0), abs=1e-9)


def test_nothing_to_say_is_none_and_not_zero():
    """A caller has to tell 'no correction' from 'the estimate is right'. Zero
    asserts the second, and would clamp a drifting pose to a lie."""
    assert drift_from_observations([]) is None
    assert drift_from_observations([((0, 0), (1, 1), 0.0)]) is None


def test_an_absurd_correction_is_refused_rather_than_applied():
    """In an 8x8 m arena a 5 m 'drift' is the wrong pad matched. Applying it
    is worse than applying nothing, so it comes back as None."""
    assert drift_from_observations([((0, 0), (5.0, 0.0), 1.0)]) is None


def test_the_limit_is_a_parameter_not_a_law():
    assert drift_from_observations([((0, 0), (5.0, 0.0), 1.0)],
                                   max_correction_m=6.0) is not None


# ── the anchor ──────────────────────────────────────────────────────────────

def test_the_anchor_measures_drift_against_where_the_drone_armed():
    pads = [P(0, 1.2, -0.4, is_takeoff_base=True), P(1, 3.0, 3.0)]
    assert anchor_drift(pads, (1.0, -0.5)) == pytest.approx((0.2, 0.1), abs=1e-9)


def test_no_takeoff_base_means_no_anchor():
    """Registration can fail. Silently anchoring to an ordinary pad would tie
    the pose to a position that drifted along with it."""
    assert anchor_drift([P(1, 3.0, 3.0)], (0.0, 0.0)) is None


# ── the feedback defence ────────────────────────────────────────────────────

def test_a_pad_still_being_fused_is_not_released():
    """Its position is still being rebuilt out of the drifting pose. Using it
    as a reference is the filter believing its own output."""
    t = LandmarkTracker()
    for x in (1.0, 1.4, 1.9, 2.5):
        settled = t.update([P(1, x, 0.0)])
    assert settled == set()


def test_a_pad_that_stopped_moving_is_released():
    t = LandmarkTracker()
    for _ in range(5):
        settled = t.update([P(1, 2.0, 0.0)])
    assert 1 in settled


def test_settling_restarts_when_the_position_jumps_again():
    """A new detection that moves the fused centre means the map changed its
    mind, and the pad goes back to being a work in progress."""
    t = LandmarkTracker()
    for _ in range(5):
        t.update([P(1, 2.0, 0.0)])
    assert t.update([P(1, 2.8, 0.0)]) == set()


def test_a_pad_that_left_the_map_is_forgotten_not_remembered_as_settled():
    t = LandmarkTracker()
    for _ in range(5):
        t.update([P(1, 2.0, 0.0)])
    assert t.update([P(2, 0.0, 0.0)]) == set()
    assert 1 not in t.settled()


def test_two_pads_settle_independently():
    t = LandmarkTracker()
    for i in range(5):
        settled = t.update([P(1, 2.0, 0.0), P(2, float(i), 0.0)])
    assert settled == {1}


# ── association ─────────────────────────────────────────────────────────────

def test_a_detection_matches_the_pad_it_is_near():
    from hydrone_localization.landmark import associate
    pads = [P(1, 2.0, 0.0), P(2, -3.0, 3.0)]
    assert associate(pads, (2.3, 0.1)).id == 1


def test_a_detection_near_nothing_matches_nothing():
    from hydrone_localization.landmark import associate
    assert associate([P(1, 2.0, 0.0)], (7.0, 7.0)) is None


def test_an_ambiguous_detection_is_refused():
    """Two pads about equally close. A wrong association does not make a small
    error — it makes a correction the size of the gap between them, applied
    with full confidence."""
    from hydrone_localization.landmark import associate
    pads = [P(1, 0.0, 0.0), P(2, 0.6, 0.0)]
    assert associate(pads, (0.3, 0.0)) is None


def test_association_can_be_restricted_to_settled_landmarks():
    from hydrone_localization.landmark import associate
    pads = [P(1, 2.0, 0.0), P(2, -3.0, 3.0)]
    assert associate(pads, (2.1, 0.0), eligible={2}) is None
    assert associate(pads, (2.1, 0.0), eligible={1}).id == 1
