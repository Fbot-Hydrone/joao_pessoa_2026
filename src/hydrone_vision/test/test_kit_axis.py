"""kit_axis on drawn kits: silver bag, black trim, yellow label, on a base.

The real-photo check (kit_dataset_mix, C922) is in docs/FASE2.md.
"""
import math

import cv2
import numpy as np
import pytest

from hydrone_vision.kit_axis import kit_axis


def scene(angle_deg, *, size=(144, 59), ring=True, noise=8, seed=0):
    """A 640x480 belly frame: blue base, yellow ring, a kit at the centre."""
    rng = np.random.default_rng(seed)
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = (160, 70, 20)                                   # blue base (BGR)
    if ring:
        cv2.circle(img, (330, 250), 230, (0, 220, 240), 18)  # yellow ring
    box = ((320, 240), size, angle_deg)
    pts = cv2.boxPoints(box).astype(np.int32)
    cv2.fillPoly(img, [pts], (190, 190, 190))                # silver
    cv2.polylines(img, [pts], True, (20, 20, 20), 6)         # black trim
    lab = cv2.boxPoints(((320, 240), (size[0] * 0.4, size[1] * 0.35),
                         angle_deg)).astype(np.int32)
    cv2.fillPoly(img, [lab], (0, 210, 230))                  # yellow label
    img = img.astype(np.int16) + rng.integers(-noise, noise + 1, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8), box


def radius(box):
    pts = cv2.boxPoints(box)
    w, h = np.ptp(pts[:, 0]), np.ptp(pts[:, 1])
    return math.sqrt(w * h / math.pi)


def err_deg(a, b):
    d = (a - b + math.pi / 2) % math.pi - math.pi / 2
    return abs(math.degrees(d))


@pytest.mark.parametrize("deg", [0, 15, 30, 45, 60, 90, 120, 150, -20])
def test_the_long_axis_comes_back_at_any_angle(deg):
    img, box = scene(deg, seed=deg % 7)
    a, coh = kit_axis(img, 320, 240, radius(box))
    assert err_deg(a, math.radians(deg)) < 3.0
    assert coh > 0.4


def test_a_square_has_no_axis_worth_turning_for():
    img, box = scene(20, size=(90, 90))
    _, coh = kit_axis(img, 320, 240, radius(box))
    assert coh < 0.1


def test_an_off_centre_box_and_a_box_at_the_edge():
    img, box = scene(70)
    shifted = np.roll(img, (40, -60), axis=(0, 1))
    a, _ = kit_axis(shifted, 260, 280, radius(box))
    assert err_deg(a, math.radians(70)) < 3.0
    assert kit_axis(img, 20, 240, radius(box)) is None


@pytest.mark.parametrize("deg", [10, 40, 80])
def test_pale_lines_under_the_kit_do_not_pull_the_axis(deg):
    """The lab base: pale yellow lines through the kit, like the real photos."""
    img, box = scene(deg, ring=False)
    for ang in (0, 60, 120):
        dx, dy = 400 * math.cos(math.radians(ang)), 400 * math.sin(math.radians(ang))
        cv2.line(img, (int(320 - dx), int(240 - dy)), (int(320 + dx), int(240 + dy)),
                 (150, 215, 230), 12)
    p = cv2.boxPoints(box).astype(np.int32)
    cv2.fillPoly(img, [p], (190, 190, 190))
    cv2.polylines(img, [p], True, (20, 20, 20), 6)
    a, coh = kit_axis(img, 320, 240, radius(box))
    assert err_deg(a, math.radians(deg)) < 4.0
    assert coh > 0.4
