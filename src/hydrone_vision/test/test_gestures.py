"""Phase 3 gesture classifier (hydrone_vision.gestures): synthetic skeletons, no model.

    python3 -m pytest src/hydrone_vision/test/test_gestures.py -q
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from hydrone_vision.gestures import (L_EL, L_SH, L_WR, R_EL, R_SH, R_WR, arm_state,  # noqa: E402
                                     classify, mediapipe_to_coco,
                                     person_center)

UPPER, FORE = 60.0, 55.0


def skeleton(left_deg, right_deg, cx=320.0, sh_y=200.0, half=50.0, bend=0.0, conf=0.9):
    """A person FACING the camera, arms at the given angles (0 hanging, 90 out, 180 up).

    The person's left shoulder is on IMAGE RIGHT (they face us), and each arm
    goes outward from its own side. `bend` folds the forearm by that many degrees.
    """
    k = [(cx, sh_y + 200.0)] * 17
    c = [conf] * 17
    for sh, el, wr, side, ang in ((L_SH, L_EL, L_WR, +1, left_deg), (R_SH, R_EL, R_WR, -1, right_deg)):
        s = (cx + side * half, sh_y)
        a = math.radians(ang)
        e = (s[0] + side * UPPER * math.sin(a), s[1] + UPPER * math.cos(a))
        b = math.radians(ang + bend)
        w = (e[0] + side * FORE * math.sin(b), e[1] + FORE * math.cos(b))
        k[sh], k[el], k[wr] = s, e, w
    k[11], k[12] = (cx + half * 0.8, sh_y + 180), (cx - half * 0.8, sh_y + 180)
    return k, c


@pytest.mark.parametrize("left,right,want", [
    (0, 0, "HOVER"),
    (90, 90, "STOP"),
    (170, 170, "SUBIR"),
    (45, 45, "DESCER"),
    (170, 90, "POUSAR"),
    (90, 170, "POUSAR"),
    (170, 0, "AFASTAR"),
    (0, 170, "AFASTAR"),
    (130, 0, "APROXIMAR"),
    (0, 130, "APROXIMAR"),
])
def test_vocabulary(left, right, want):
    assert classify(*skeleton(left, right))[0] == want


def test_side_arm_points_where_the_drone_goes():
    # operator's LEFT arm out: it is on image right, so the drone goes right
    assert classify(*skeleton(90, 0))[0] == "DIREITA"
    assert classify(*skeleton(0, 90))[0] == "ESQUERDA"


@pytest.mark.parametrize("left,right", [(27, 0), (68, 0), (112, 0), (152, 0), (130, 130), (45, 90)])
def test_dead_zones_and_unlisted_pairs_are_nothing(left, right):
    assert classify(*skeleton(left, right))[0] == "NENHUM"


def test_bent_arm_is_ignored():
    k, c = skeleton(90, 0, bend=100)
    assert arm_state(k[L_SH], k[L_EL], k[L_WR], (0.9, 0.9, 0.9))[0] == "BENT"
    assert classify(k, c)[0] == "NENHUM"


def test_unsure_keypoint_is_ignored():
    k, c = skeleton(170, 170)
    c[L_WR] = 0.2
    assert classify(k, c)[0] == "NENHUM"


def test_mediapipe_landmarks_scale_to_pixels():
    # 33 landmarks; a 45 deg arm in pixels is NOT 45 deg in normalised 4:3 units
    lms = [(0.5, 0.5, 0.9)] * 33
    lms = list(lms)
    lms[11], lms[13], lms[15] = (0.6, 0.4, 0.9), (0.6 + 60 / 640, 0.4 + 60 / 480, 0.9), \
        (0.6 + 120 / 640, 0.4 + 120 / 480, 0.9)
    k, c = mediapipe_to_coco(lms, 640, 480)
    assert len(k) == 17 and len(c) == 17
    assert k[L_SH] == pytest.approx((384.0, 192.0))
    _, _, ang = arm_state(k[L_SH], k[L_EL], k[L_WR], (1, 1, 1))
    assert ang == pytest.approx(45.0)


def test_person_center():
    near = skeleton(0, 0, cx=480.0, half=80.0)
    far = skeleton(0, 0, cx=100.0, half=30.0)
    x, size = person_center(*near, 640, 480)
    assert x == pytest.approx(0.5) and size == pytest.approx(160 / 640)
    k, c = far
    c = list(c)
    c[L_SH] = c[R_SH] = c[11] = c[12] = 0.1
    assert person_center(k, c, 640, 480) is None
