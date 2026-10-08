"""visual_descent: Black Bee style centring + descent, ROS-free."""
import pytest

from hydrone_nav.visual_descent import (CENTRING, DESCENDING, LOST, READY,
                                        VisualDescent)

FX = 320.0


def vd(**k):
    return VisualDescent(target_uv=(320, 240), **k)


def test_it_tracks_the_detection_nearest_the_centre_not_a_neighbour():
    """2026-10-01: with three bases in frame the old servo chased neighbours."""
    v = vd()
    step = v.update([(60, 400), (330, 250), (500, 100)], 2.5, FX, 0.0)
    assert step.picked == (330, 250)


def test_far_off_centre_it_moves_towards_the_pad_and_does_not_descend():
    v = vd()
    step = v.update([(520, 240)], 2.5, FX, 0.0)       # pad to the right
    assert step.phase == CENTRING
    assert step.dy < 0 and step.dz == 0.0             # FLU: right is -y
    assert abs(step.dx) < 1e-9


def test_the_step_is_capped():
    step = vd(max_step_m=0.2).update([(639, 479)], 2.5, FX, 0.0)
    assert (step.dx ** 2 + step.dy ** 2) ** 0.5 == pytest.approx(0.2)


def test_once_roughly_centred_it_descends_while_centring():
    v = vd()
    step = v.update([(360, 260)], 2.5, FX, 0.0)
    assert step.phase == DESCENDING and step.dz < 0


def test_ready_only_when_low_and_well_centred():
    v = vd(descend_to_m=1.2)
    v.update([(330, 245)], 2.5, FX, 0.0)
    assert v.update([(330, 245)], 1.6, FX, 0.1).phase == DESCENDING
    assert v.update([(400, 245)], 1.2, FX, 0.2).phase == DESCENDING   # 80 px
    assert v.update([(330, 245)], 1.2, FX, 0.3).phase == READY


def test_losing_the_pad_for_long_enough_is_lost():
    v = vd(lost_after_s=3.0)
    v.update([(330, 245)], 2.5, FX, 0.0)
    assert v.update([], 2.5, FX, 1.0).phase != LOST
    assert v.update([], 2.5, FX, 4.5).phase == LOST


def test_a_flipped_mount_flips_the_axes():
    s1 = vd().update([(520, 240)], 2.5, FX, 0.0)
    s2 = vd(axes=(1.0, -1.0)).update([(520, 240)], 2.5, FX, 0.0)
    assert s2.dy == pytest.approx(-s1.dy)


def test_a_pad_below_centre_means_move_back():
    """+v is body BACK for this belly camera (down_cam_mimic_node TF)."""
    step = vd().update([(320, 440)], 2.5, FX, 0.0)
    assert step.dx < 0 and abs(step.dy) < 1e-9
