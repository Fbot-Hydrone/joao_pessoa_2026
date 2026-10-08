"""precision_landing: the geometry moved out of phase1_mission_node."""
import math

import pytest

from hydrone_nav import precision_landing as pl


def test_height_is_over_the_measured_top_not_the_altitude():
    assert pl.height_over_pad(2.5, pad_top=1.4) == pytest.approx(1.1)
    assert pl.height_over_pad(2.5, pad_top=None, ground_z=-0.7) == \
        pytest.approx(3.2)
    assert pl.height_over_pad(0.1, pad_top=0.1) == 0.2      # floor


def test_a_camera_on_the_axis_targets_the_optical_centre():
    assert pl.target_uv((320, 240), (0.0, 0.0), 320.0, 1.5) == (320, 240)


def test_a_camera_ahead_must_see_the_pad_behind_centre():
    u, v = pl.target_uv((320, 240), (0.1, 0.0), 320.0, 1.0)
    assert u == 320 and v == pytest.approx(272.0)


def test_pixels_become_centimetres_through_fx_and_height():
    assert pl.offset_cm(100.0, 320.0, 1.6) == pytest.approx(50.0)
    assert pl.offset_cm(100.0, None, 1.6) is None


def test_body_steps_rotate_with_yaw():
    dx, dy = pl.body_to_world((1.0, 0.0), math.pi / 2)
    assert dx == pytest.approx(0.0, abs=1e-9) and dy == pytest.approx(1.0)
