"""frl_core (the camera's classifier) and its bridge to the mission, headless."""
import math

import numpy as np
import pytest

from hydrone_mission.phase3 import core, frl_core

ANG = {"DOWN": 5, "DIAG_DOWN": 45, "SIDE": 90, "DIAG_UP": 130, "UP": 175}


def skeleton(left, right):
    """COCO-17 for a person facing the camera, arms straight at the given states.

    The person's LEFT arm is on the image's RIGHT (+x), so it swings out to +x.
    """
    k = np.zeros((17, 2))
    k[frl_core.L_SH], k[frl_core.R_SH] = (340, 200), (300, 200)
    for sh, el, wr, state, side in ((frl_core.L_SH, frl_core.L_EL, frl_core.L_WR, left, +1),
                                    (frl_core.R_SH, frl_core.R_EL, frl_core.R_WR, right, -1)):
        t = math.radians(ANG[state])
        d = np.array([side * math.sin(t), math.cos(t)])
        k[el], k[wr] = k[sh] + 60 * d, k[sh] + 120 * d
    return k, np.ones(17)


@pytest.mark.parametrize("left,right,gesture", [
    ("DOWN", "DOWN", "HOVER"), ("SIDE", "SIDE", "STOP"), ("UP", "UP", "SUBIR"),
    ("DIAG_DOWN", "DIAG_DOWN", "DESCER"), ("UP", "SIDE", "POUSAR"),
    ("DOWN", "UP", "AFASTAR"), ("DIAG_UP", "DOWN", "APROXIMAR"),
    ("SIDE", "DOWN", "DIREITA"),      # left arm out to the image's right
    ("DOWN", "SIDE", "ESQUERDA"),
])
def test_every_gesture_of_the_vocabulary(left, right, gesture):
    assert frl_core.classify(*skeleton(left, right))[0] == gesture


def test_a_bent_or_uncertain_arm_is_nothing():
    k, c = skeleton("SIDE", "DOWN")
    k[frl_core.L_WR] = k[frl_core.L_SH]           # wrist back at the shoulder: bent
    assert frl_core.classify(k, c)[0] == "NENHUM"
    k, c = skeleton("SIDE", "DOWN")
    c[frl_core.R_WR] = 0.1
    assert frl_core.classify(k, c)[0] == "NENHUM"


def test_debouncer_needs_the_hold_and_brakes_fast():
    d = frl_core.Debouncer()
    assert d.update("APROXIMAR", 0.0) == ("HOVER", False)
    assert d.update("APROXIMAR", 0.4) == ("HOVER", False)
    assert d.update("APROXIMAR", 0.5) == ("APROXIMAR", True)
    assert d.update("NENHUM", 0.6) == ("APROXIMAR", False)
    assert d.update("NENHUM", 0.85) == ("HOVER", True)


def test_every_streamed_name_is_understood_by_the_mission():
    assert set(core.STREAMED) == set(frl_core.COMMANDS) | {"NENHUM"}
    for name in core.STREAMED:
        assert (core.velocity_for(name) is not None) != (core.action_for(name) is not None), name
    assert core.action_for("POUSAR") == core.LAND
    assert core.action_for("STOP") == core.STOP


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, -2.5])
def test_velocity_is_in_the_body_frame(yaw):
    fwd = core.velocity_enu(core.velocity_for("APROXIMAR"), yaw, 0.5, 0.3)
    right = core.velocity_enu(core.velocity_for("DIREITA"), yaw, 0.5, 0.3)
    up = core.velocity_enu(core.velocity_for("SUBIR"), yaw, 0.5, 0.3)
    assert fwd[:2] == pytest.approx((0.5 * math.cos(yaw), 0.5 * math.sin(yaw)))
    assert right[:2] == pytest.approx((0.5 * math.cos(yaw - math.pi / 2),
                                       0.5 * math.sin(yaw - math.pi / 2)))
    assert up == pytest.approx((0, 0, 0.3))
