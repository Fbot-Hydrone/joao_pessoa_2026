"""Centring on the pad without knowing how the camera is mounted.

The camera is simulated as an arbitrary 2x2 mapping from body metres to
pixels. The servo is never told what it is. If it only works for the mapping
it was written against, it does not work.

    python3 -m pytest src/hydrone_nav/test/test_servo.py -q
"""

import math

import pytest

from hydrone_nav.servo import VisualServo

TARGET = (320.0, 240.0)


class Camera:
    """A pad at `pad_xy` in the body frame, seen through an arbitrary mount."""

    def __init__(self, mapping, pad_xy=(0.4, -0.3)):
        self.m = mapping                  # [[a, b], [c, d]]: metres -> pixels
        self.pad = list(pad_xy)

    def move(self, step):
        """The VEHICLE moves, so the pad moves the other way in the body frame."""
        self.pad[0] -= step[0]
        self.pad[1] -= step[1]

    def uv(self):
        return (TARGET[0] + self.m[0][0] * self.pad[0] + self.m[0][1] * self.pad[1],
                TARGET[1] + self.m[1][0] * self.pad[0] + self.m[1][1] * self.pad[1])

    def offset(self):
        return math.hypot(*self.pad)


def converge(mapping, pad_xy=(0.4, -0.3), steps=40):
    cam = Camera(mapping, pad_xy)
    servo = VisualServo(target_uv=TARGET)
    for _ in range(steps):
        step = servo.update(cam.uv(), 1.0)
        if step is None:
            break
        cam.move(step)
    return cam.offset()


IDENTITYish = [[300.0, 0.0], [0.0, 300.0]]


def test_it_centres_with_the_obvious_mounting():
    assert converge(IDENTITYish) < 0.05


def test_it_centres_when_BOTH_signs_are_inverted():
    """A wrong sign does not centre slowly — it walks the vehicle away from the
    pad, accelerating. This is the case that has to work."""
    assert converge([[-300.0, 0.0], [0.0, -300.0]]) < 0.05


def test_it_centres_when_the_camera_is_rotated_ninety_degrees():
    assert converge([[0.0, 300.0], [-300.0, 0.0]]) < 0.05


def test_it_centres_when_the_camera_is_mirrored():
    assert converge([[300.0, 0.0], [0.0, -300.0]]) < 0.05


def test_it_centres_through_a_skewed_mount():
    """Nothing guarantees the real mount is a clean rotation."""
    assert converge([[280.0, 90.0], [-70.0, 310.0]]) < 0.08


def test_it_centres_from_a_long_way_out():
    assert converge(IDENTITYish, pad_xy=(1.2, 0.9)) < 0.08


# ── the guards ───────────────────────────────────────────────────────────────

def test_a_centred_pad_asks_for_no_move():
    """Moving again would only add drift."""
    servo = VisualServo(target_uv=TARGET)
    assert servo.update(TARGET, 1.0) is None


def test_the_deadband_is_respected():
    servo = VisualServo(target_uv=TARGET, deadband_px=20.0)
    assert servo.update((TARGET[0] + 5.0, TARGET[1]), 1.0) is None
    assert servo.update((TARGET[0] + 50.0, TARGET[1]), 1.0) is not None


def test_no_single_step_exceeds_the_limit():
    """The guard that turns a bad estimate into a slow correction rather than
    a flyaway."""
    servo = VisualServo(target_uv=TARGET, max_step_m=0.25)
    for uv in [(TARGET[0] + 5000.0, TARGET[1] - 4000.0), (0.0, 0.0)]:
        step = servo.update(uv, 1.0)
        assert step is None or math.hypot(*step) <= 0.25 + 1e-9


def test_an_off_centre_lens_is_a_parameter_not_a_bug():
    """What the servo cannot learn is where the camera points when the vehicle
    is level. On a misaligned airframe, centring on the IMAGE centre parks the
    vehicle off the pad by that misalignment times the height — so the target
    pixel is a property of the airframe, measured once."""
    off = (360.0, 200.0)
    cam = Camera(IDENTITYish)
    servo = VisualServo(target_uv=off)
    for _ in range(40):
        step = servo.update(cam.uv(), 1.0)
        if step is None:
            break
        cam.move(step)
    # It centred on ITS target, which is 40 px right and 40 px up of the image
    # centre — i.e. deliberately off the geometric middle.
    u, v = cam.uv()
    assert abs(u - off[0]) < 20.0 and abs(v - off[1]) < 20.0


def test_it_identifies_the_mount_in_two_probes():
    """Two orthogonal probes determine a 2x2 Jacobian EXACTLY, for any
    invertible mount. Nothing has to converge, and no rotation, sign or scale
    is assumed — which is the whole reason a real airframe's camera can be
    bolted on any way round."""
    cam = Camera([[0.0, 300.0], [-300.0, 0.0]])
    servo = VisualServo(target_uv=TARGET)
    for _ in range(3):
        step = servo.update(cam.uv(), 1.0)
        if step is None:
            break
        cam.move(step)
    # What it learns is d(pixel)/d(VEHICLE MOTION), which is the NEGATIVE of
    # the mount's metres-to-pixels: moving the vehicle right slides the pad
    # left in frame. That is the quantity it needs — it inverts this to turn a
    # pixel error into a move.
    assert servo._J is not None
    assert servo._J[0][1] == pytest.approx(-300.0, abs=1.0)
    assert servo._J[1][0] == pytest.approx(+300.0, abs=1.0)


def test_a_probe_is_big_enough_to_read_and_small_enough_to_be_wrong_about():
    servo = VisualServo()
    assert servo.min_learn_step_m < servo._probe_size(1.0) <= servo.max_step_m


def test_a_tiny_step_does_not_teach_the_estimate():
    """The pixel change would be mostly noise, and a Broyden update on noise
    corrupts the estimate that the next real step depends on."""
    servo = VisualServo(target_uv=TARGET, min_learn_step_m=0.05)
    servo.update((TARGET[0] + 100.0, TARGET[1]), 1.0)
    servo._last_step = (0.001, 0.0)
    before = servo.updates
    servo.update((TARGET[0] + 90.0, TARGET[1]), 1.0)
    assert servo.updates == before


def test_reset_keeps_what_the_camera_taught_it():
    """A new pad is the same camera, so the probes are not paid for twice."""
    servo = VisualServo(target_uv=TARGET)
    cam = Camera(IDENTITYish)
    for _ in range(4):
        step = servo.update(cam.uv(), 1.0)
        if step is None:
            break
        cam.move(step)
    j = [row[:] for row in servo._J]
    servo.reset()
    assert servo._J == j
    assert servo._last_step is None
