#!/usr/bin/env python3
"""
Tests for pad_mission_node's forward run and its landing trigger.

The mission is deliberately tiny — fly +X, land on what the down camera sees,
take off, repeat — so there is not much logic to pin. What IS worth pinning is
the handful of decisions whose failure is silent and expensive:

  * the setpoint must never sit far ahead of the vehicle. The position error is
    what the FCU converts into acceleration, and an aggressive demand is what
    flips the vehicle under BiguaSim's actuation lag (see
    ~/work/biguasim-problems.md, 2026-08-18).
  * a stale or unconfident detection must not trigger a landing.
  * the pad the drone JUST took off from must not trigger a landing, or the
    mission lands on it forever and never advances.

The flight states themselves (arming, takeoff, landing) are not covered here —
they are conversations with ArduPilot, and mocking one proves nothing about the
real vehicle. They are exercised by flying the sim; see docs/LANDING-SITES.md.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_mission/test/test_pad_mission.py -q'
"""

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped

from hydrone_msgs.msg import Pad, PadDetection, PadMap
from hydrone_mission.pad_mission_node import PadMissionNode


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PadMissionNode(parameter_overrides=[
        rclpy.parameter.Parameter("auto_start", value=False),
        rclpy.parameter.Parameter("forward_step", value=1.0),
        rclpy.parameter.Parameter("rearm_distance_m", value=3.0),
        rclpy.parameter.Parameter("min_confidence", value=0.60),
    ])
    n.home = (0.0, 0.0)
    n.leg_start_x = 0.0
    n.target_x = 1.0
    n.state = n.FORWARD
    set_pose(n, 0.0, 0.0, 2.5)
    yield n
    n.destroy_node()


def set_pose(node, x, y=0.0, z=2.5):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    node.pose = pose


def see_pad(node, confidence=0.9, age_s=0.0):
    """Pretend the down camera just reported a pad."""
    det = PadDetection()
    det.camera = "down"
    det.confidence = float(confidence)
    det.position_valid = True
    node._last_down = det
    node._last_down_t = node._now() - age_s


# ── The forward run ──────────────────────────────────────────────────────────

def test_the_setpoint_starts_one_step_ahead(node):
    node._do_forward()
    assert node.setpoint[0] == pytest.approx(1.0)
    assert node.stream_setpoint


def test_the_setpoint_only_advances_once_the_vehicle_arrives(node):
    node._do_forward()
    set_pose(node, 0.6)                 # short of the 0.5 m tolerance band
    node._do_forward()
    assert node.setpoint[0] == pytest.approx(1.0)

    set_pose(node, 0.9)                 # inside tolerance of the 1.0 m target
    node._do_forward()
    assert node.setpoint[0] == pytest.approx(2.0)


def test_the_setpoint_never_runs_far_ahead_of_the_vehicle(node):
    """The whole point of stepping: bounded position error, gentle demand.

    A setpoint metres ahead is a full-throttle acceleration command, which is
    what the sim's lagging actuation turns into a flip.
    """
    for x in [v / 10.0 for v in range(0, 300)]:
        set_pose(node, x)
        node._do_forward()
        assert node.setpoint[0] - x <= node.forward_step + node.wp_tol


def test_the_run_holds_its_lateral_line_and_altitude(node):
    set_pose(node, 5.0, y=1.7, z=2.5)   # blown sideways off the line
    node._do_forward()
    assert node.setpoint[1] == pytest.approx(node.home[1])
    assert node.setpoint[2] == pytest.approx(node.cruise_alt)


def test_forward_limit_of_zero_never_ends_the_run(node):
    node.forward_limit = 0.0
    set_pose(node, 500.0)
    node._do_forward()
    assert node.state == node.FORWARD


def test_forward_limit_ends_the_run(node):
    node.forward_limit = 10.0
    set_pose(node, 10.5)
    node._do_forward()
    assert node.state == node.ABORTED
    assert not node.stream_setpoint


# ── The landing trigger ──────────────────────────────────────────────────────

def test_a_fresh_confident_pad_below_triggers_a_landing(node):
    set_pose(node, 5.0)
    see_pad(node, confidence=0.9)
    node._do_forward()
    assert node.state == node.LAND
    # The FCU owns the descent from here; a position setpoint would fight it.
    assert not node.stream_setpoint


def test_a_stale_detection_does_not_trigger_a_landing(node):
    set_pose(node, 5.0)
    see_pad(node, confidence=0.9, age_s=5.0)
    assert not node._pad_below()


def test_a_low_confidence_detection_does_not_trigger_a_landing(node):
    set_pose(node, 5.0)
    see_pad(node, confidence=0.3)
    assert not node._pad_below()


def test_no_detection_at_all_does_not_trigger_a_landing(node):
    set_pose(node, 5.0)
    assert not node._pad_below()


def test_the_forward_camera_is_ignored(node):
    """This mission has no phase that flies to a distant pad, so a forward-cam
    sighting must not reach the landing trigger at all."""
    set_pose(node, 5.0)
    det = PadDetection()
    det.camera = "forward"
    det.confidence = 0.99
    node._cb_detection(det)
    assert node._last_down is None
    assert not node._pad_below()


# ── Taking off again ─────────────────────────────────────────────────────────

def test_the_pad_just_left_does_not_trigger_another_landing(node):
    """Without the ignore window the drone lands on the pad under it, takes off,
    sees it again, and never advances."""
    node.leg_start_x = 5.0
    set_pose(node, 5.2)
    see_pad(node, confidence=0.9)
    assert not node._pad_below()


def test_a_pad_past_the_ignore_window_does_trigger(node):
    node.leg_start_x = 5.0
    set_pose(node, 8.5)                 # 3.5 m > rearm_distance_m
    see_pad(node, confidence=0.9)
    assert node._pad_below()


def test_the_ignore_window_is_measured_from_the_last_takeoff(node):
    """It resets each leg — it is 'since takeoff', not 'since home'."""
    node.leg_start_x = 0.0
    set_pose(node, 4.0)
    see_pad(node, confidence=0.9)
    assert node._pad_below()            # 4 m into the first leg: armed

    node.leg_start_x = 4.0              # landed here, took off again
    assert not node._pad_below()        # same spot, now inside the window


# ── Recording the landing ────────────────────────────────────────────────────

def make_pad(pad_id, x, y):
    pad = Pad()
    pad.id = pad_id
    pad.position.x = float(x)
    pad.position.y = float(y)
    return pad


def set_map(node, *pads):
    msg = PadMap()
    msg.header.frame_id = "map"
    msg.pads = list(pads)
    node.pad_map = msg


def test_the_pad_underneath_is_the_one_marked_visited(node):
    set_pose(node, 8.0, y=0.1, z=0.12)
    set_map(node, make_pad(0, 0.0, 0.0), make_pad(7, 8.1, 0.0))
    assert node._pad_id_below() == 7


def test_a_landing_nowhere_near_a_mapped_pad_marks_nothing(node):
    """The map is optional to this mission — landing off-map must not crash it,
    it just goes unrecorded."""
    set_pose(node, 40.0, z=0.12)
    set_map(node, make_pad(0, 0.0, 0.0))
    assert node._pad_id_below() is None


def test_an_empty_map_marks_nothing(node):
    set_pose(node, 8.0, z=0.12)
    assert node._pad_id_below() is None


# ── Holding still until the FCU is ready ─────────────────────────────────────

def test_the_mission_holds_until_the_fcu_is_ready(node):
    node.state = node.WAIT_FCU
    node.auto_start = True
    node.mav_state.connected = False
    node._do_wait_fcu()
    assert node.state == node.WAIT_FCU


def test_setpoints_are_not_streamed_before_takeoff(node):
    node.state = node.WAIT_FCU
    node.stream_setpoint = False
    node._tick()
    assert not node.stream_setpoint
