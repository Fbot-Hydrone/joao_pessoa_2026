"""Phase 3 core: the gesture contract and the step math, headless."""
import math

import pytest

from hydrone_mission.phase3 import core


def test_the_vision_vocabulary_maps_to_actions():
    assert core.action_for("arms_forward") == core.MOVE_FORWARD
    assert core.action_for("right_arm_point") == core.MOVE_RIGHT
    assert core.action_for("both_arms_up") == core.LAND
    assert core.action_for("thumbs_up") == core.TAKEOFF
    assert core.action_for("arms_cross") == core.STOP


def test_action_names_pass_through_and_unknown_is_nothing():
    assert core.action_for("MOVE_UP") == core.MOVE_UP
    assert core.action_for("move_up") == core.MOVE_UP
    assert core.action_for("unknown") is None
    assert core.action_for("") is None


def test_every_terminal_command_publishes_a_mapped_gesture():
    seen = set()
    for words, gesture, _ in core.TERMINAL_COMMANDS:
        assert core.action_for(gesture) is not None, gesture
        for w in words:
            assert w not in seen, f"'{w}' bound twice"
            seen.add(w)
            assert core.parse_terminal(w.upper()) == gesture


def test_raw_gesture_names_work_in_the_terminal():
    assert core.parse_terminal("thumbs_up") == "thumbs_up"
    assert core.parse_terminal("nope") is None


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, -2.0])
def test_steps_are_in_the_body_frame(yaw):
    sp = [1.0, 2.0, 1.5, yaw]
    f = core.step_setpoint(sp, core.MOVE_FORWARD, 0.5, 0.3, 0.1)
    r = core.step_setpoint(sp, core.MOVE_RIGHT, 0.5, 0.3, 0.1)
    assert f[0] - 1.0 == pytest.approx(0.5 * math.cos(yaw))
    assert f[1] - 2.0 == pytest.approx(0.5 * math.sin(yaw))
    # right is forward turned clockwise by 90 deg
    assert r[0] - 1.0 == pytest.approx(0.5 * math.cos(yaw - math.pi / 2))
    assert r[1] - 2.0 == pytest.approx(0.5 * math.sin(yaw - math.pi / 2))
    assert f[2] == r[2] == 1.5 and f[3] == r[3] == yaw


def test_facing_east_right_is_south():
    x, y, _, _ = core.step_setpoint([0, 0, 1, 0.0], core.MOVE_RIGHT, 1.0, 0.3, 0.1)
    assert (x, y) == pytest.approx((0.0, -1.0))


def test_turn_right_is_clockwise_and_wraps():
    sp = core.step_setpoint([0, 0, 1, -3.0], core.TURN_RIGHT, 0.5, 0.3, math.radians(45))
    assert sp[3] == pytest.approx(core.wrap_pi(-3.0 - math.radians(45)))
    assert -math.pi < sp[3] <= math.pi


def test_up_down():
    assert core.step_setpoint([0, 0, 1, 0], core.MOVE_UP, 0.5, 0.3, 0.1)[2] == pytest.approx(1.3)
    assert core.step_setpoint([0, 0, 1, 0], core.MOVE_DOWN, 0.5, 0.3, 0.1)[2] == pytest.approx(0.7)


def test_non_moves_are_refused():
    with pytest.raises(ValueError):
        core.step_setpoint([0, 0, 1, 0], core.LAND, 0.5, 0.3, 0.1)
