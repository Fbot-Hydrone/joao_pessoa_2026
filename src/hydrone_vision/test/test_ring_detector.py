import cv2
import numpy as np

from hydrone_vision.ring_detector import fit_ring

YELLOW = (0, 220, 230)          # BGR, inside the sim yellow band


def frame(cx, cy, r, kit=True):
    img = np.full((480, 640, 3), (170, 160, 150), np.uint8)
    cv2.circle(img, (cx, cy), r, YELLOW, 8)
    if kit:                      # the carried kit: a band over the middle
        cv2.rectangle(img, (245, 0), (420, 400), (200, 170, 146), -1)
    return img


def test_centre_found_with_the_middle_covered():
    u, v, r, conf = fit_ring(frame(335, 212, 98), 1.5)
    assert abs(u - 335) < 4 and abs(v - 212) < 4 and abs(r - 98) < 6
    assert conf > 0.5


def test_wrong_size_for_the_height_is_rejected():
    # radius 98 px is a base ~1.5 m below; at 3 m it must be something else
    assert fit_ring(frame(335, 212, 98), 3.0) is None


def test_no_yellow_no_ring():
    assert fit_ring(frame(335, 212, 98)[:, :, ::-1] * 0, 1.5) is None
    assert fit_ring(frame(335, 212, 98), None) is None
