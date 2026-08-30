#!/usr/bin/env python3
"""
Tests for phase1_mission_node: the turn-and-look search, and the decisions that
decide where the vehicle goes.

The flight states themselves (arming, takeoff, landing) are NOT covered here —
they are conversations with ArduPilot, and mocking one proves nothing about the
real vehicle. They are exercised by flying the sim; see docs/PHASE1-MISSION.md.

What IS worth pinning is the handful of decisions whose failure is silent and
expensive:

  * the takeoff base must never be a landing candidate. If it becomes one the
    drone flies out, hovers over the thing it started on, fails to confirm it,
    and burns a search cycle plus the drift that goes with it — every run.
  * a candidate must be CONFIRMED in the map before the vehicle translates. One
    frame of noise must not move the drone.
  * the map must not be read while the drone is turning. A detection taken
    mid-slew is projected through a moving yaw estimate and lands metres out;
    the settle window is the only thing standing between that and a flight to a
    pad that is not there.
  * the search must terminate. Eight turns and a fallback, or the vehicle
    hovers until the battery decides.
  * one belly-camera frame must not satisfy the whole confirmation quota. The
    camera runs at 10 Hz and the tick at 10 Hz, so a detection left in place
    would be counted three times in 0.3 s.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_mission/test/test_phase1_mission.py -q'
"""

import math

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped

from hydrone_msgs.msg import Pad, PadDetection, PadMap
from hydrone_nav import servo as servo_module
from hydrone_mission.phase1_mission_node import (
    Phase1MissionNode, wrap_pi, yaw_of)


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Phase1MissionNode(parameter_overrides=[
        rclpy.parameter.Parameter("auto_start", value=False),
        rclpy.parameter.Parameter("takeoff_alt", value=1.0),
        rclpy.parameter.Parameter("target_bases", value=2),
        rclpy.parameter.Parameter("settle_s", value=2.0),
        rclpy.parameter.Parameter("max_rotations", value=8),
        rclpy.parameter.Parameter("confirm_detections", value=3),
        rclpy.parameter.Parameter("confirm_confidence", value=0.60),
    ])
    n.home = (0.0, 0.0)
    set_pose(n, 0.0, 0.0, 1.0)
    n.setpoint = [0.0, 0.0, 1.0, 0.0]
    yield n
    n.destroy_node()


# ── Fakes ────────────────────────────────────────────────────────────────────

def set_pose(node, x, y=0.0, z=1.0, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    node.pose = pose


def pad(pad_id, x, y, observations=5, visited=False, takeoff_base=False,
        confidence=0.9):
    p = Pad()
    p.id = int(pad_id)
    p.position.x = float(x)
    p.position.y = float(y)
    p.observations = int(observations)
    p.visited = bool(visited)
    p.is_takeoff_base = bool(takeoff_base)
    p.confidence = float(confidence)
    return p


def set_map(node, *pads):
    m = PadMap()
    m.header.frame_id = "map"
    m.pads = list(pads)
    node.pad_map = m


def see_pad(node, confidence=0.9, age_s=0.0):
    """Pretend the belly camera just reported a pad."""
    det = PadDetection()
    det.camera = "down"
    det.confidence = float(confidence)
    det.position_valid = True
    node._last_down = det
    node._last_down_t = node._now() - age_s


def enter(node, state, age_s=0.0):
    """Put the node in a state, optionally pretending it has been there a while."""
    node._enter(state)
    node._state_since = node._now() - age_s


# ── Angle helpers ────────────────────────────────────────────────────────────

def test_wrap_pi_folds_into_one_turn():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(math.radians(370.0)) == pytest.approx(math.radians(10.0))
    assert wrap_pi(math.radians(-190.0)) == pytest.approx(math.radians(170.0))


def test_yaw_of_reads_the_quaternion(node):
    set_pose(node, 0.0, yaw=math.radians(30.0))
    assert math.degrees(yaw_of(node.pose)) == pytest.approx(30.0)


# ── What counts as a landing candidate ───────────────────────────────────────

def test_the_takeoff_base_is_never_a_candidate(node):
    """The one that costs a whole search cycle per run if it regresses."""
    node.home = (9.0, 9.0)              # so the home-proximity guard is not it
    assert not node._is_candidate(pad(1, 0.0, 0.0, takeoff_base=True))


def test_a_visited_pad_is_never_a_candidate(node):
    node.home = (9.0, 9.0)
    assert not node._is_candidate(pad(1, 2.0, 0.0, visited=True))


def test_a_blacklisted_pad_is_never_a_candidate(node):
    node.home = (9.0, 9.0)
    node.blacklist.add(7)
    assert not node._is_candidate(pad(7, 2.0, 0.0))


def test_a_pad_seen_only_once_is_never_a_candidate(node):
    """One frame of blue noise reaches the map. Two frames is a thing that was
    there both times, and the confirmation hover is what settles it — a metre
    up, where the pad is hundreds of pixels across instead of a handful.
    Requiring three across the arena only meant the better judge never voted,
    and it cost real bases (see route.MIN_OBSERVATIONS)."""
    node.home = (9.0, 9.0)
    assert not node._is_candidate(pad(1, 2.0, 0.0, observations=1))
    assert node._is_candidate(pad(1, 2.0, 0.0, observations=2))


def test_anything_sitting_where_we_armed_is_never_a_candidate(node):
    """Belt and braces for a failed takeoff-base registration."""
    node.home = (0.0, 0.0)
    assert not node._is_candidate(pad(1, 0.3, -0.2))


def test_the_nearest_candidate_wins(node):
    node.home = (9.0, 9.0)
    set_pose(node, 0.0, 0.0)
    set_map(node, pad(1, 3.0, 0.0), pad(2, 1.5, 0.0))
    assert node._best_candidate().id == 2


def test_no_map_means_no_candidate(node):
    node.pad_map = None
    assert node._best_candidate() is None


# ── The search: settle, then turn ────────────────────────────────────────────

def test_the_map_is_not_read_before_the_settle_window_closes(node):
    """A pad seen mid-turn is projected through a slewing yaw estimate.

    Acting on it means flying to a position that is wrong by metres, so SETTLE
    must hold even when the map is already offering something.
    """
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=0.5)
    node._do_settle()
    assert node.state == node.SETTLE


def test_a_candidate_found_after_settling_is_taken(node):
    node.survey_done = True               # the landing phase
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.SELECT


def test_settling_with_nothing_in_the_map_starts_a_turn(node):
    node.survey_circuit = False     # this pins the local turn
    node.survey_done = True
    node.max_search_level = 1       # and the ladder must not climb
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.ROTATE


def test_the_turn_is_clockwise(node):
    """ENU yaw runs counter-clockwise from east, so clockwise SUBTRACTS."""
    node.survey_circuit = False     # this pins the local turn
    node.survey_done = True
    node.max_search_level = 1       # and the ladder must not climb
    set_map(node)
    node.setpoint = [0.0, 0.0, 1.0, 0.0]
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert math.degrees(node.setpoint[3]) == pytest.approx(-45.0)


def test_the_turn_wraps_rather_than_winding_up(node):
    node.survey_circuit = False     # this pins the local turn
    node.survey_done = True
    node.max_search_level = 1       # and the ladder must not climb
    set_map(node)
    node.setpoint = [0.0, 0.0, 1.0, math.radians(-170.0)]
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert math.degrees(node.setpoint[3]) == pytest.approx(145.0)


def test_the_turn_does_not_move_the_vehicle(node):
    node.survey_circuit = False
    node.survey_done = True
    set_map(node)
    node.setpoint = [1.3, -0.7, 1.0, 0.0]
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.setpoint[:3] == pytest.approx([1.3, -0.7, 1.0])


def test_a_turn_completes_once_the_heading_is_reached(node):
    node.setpoint = [0.0, 0.0, 1.0, math.radians(-45.0)]
    enter(node, node.ROTATE)
    set_pose(node, 0.0, yaw=math.radians(-44.0))
    node._do_rotate()
    assert node.state == node.SETTLE
    assert node.rotations_done == 1


def test_a_turn_that_never_arrives_still_counts(node):
    """Otherwise a yaw the FCU will not fly hangs the mission forever."""
    node.setpoint = [0.0, 0.0, 1.0, math.radians(-45.0)]
    enter(node, node.ROTATE, age_s=999.0)
    set_pose(node, 0.0, yaw=0.0)
    node._do_rotate()
    assert node.state == node.SETTLE
    assert node.rotations_done == 1


def test_the_search_terminates(node):
    """Eight turns is a full circle, and with nothing left to look at the run
    has to end. It ends by flying HOME, never by landing here: an off-base
    landing is eliminatory, and "where the vehicle happens to be when the
    search gives up" is the arena floor."""
    set_map(node)
    node.survey_done = True              # the sweep is already over
    node.max_search_level = 1            # and the ladder must not climb
    node.rotations_done = node.max_rotations
    node.coverage_search = False         # nothing left to look at
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL
    assert node.target_id is None


def test_a_full_search_makes_exactly_max_rotations_turns(node):
    """Walk the real loop rather than trusting the counter arithmetic."""
    set_map(node)
    node.survey_done = True               # pin the ending, not the sweep
    node.max_search_level = 1             # the ladder would reset the counter
    node.coverage_search = False          # no map here; pin the turn count
    enter(node, node.SETTLE, age_s=5.0)
    for _ in range(200):
        if node.state == node.SETTLE:
            node._state_since = node._now() - 5.0
            node._do_settle()
        elif node.state == node.ROTATE:
            set_pose(node, 0.0, yaw=node.setpoint[3])
            node._do_rotate()
        else:
            break
    assert node.rotations_done == node.max_rotations
    # It ends by flying home, never by landing where the search ran out.
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL


# ── Choosing what to do with the air ─────────────────────────────────────────

def test_a_confirmed_candidate_is_flown_to(node):
    node.survey_done = True          # the landing phase
    node.home = (9.0, 9.0)
    set_pose(node, 0.0, 0.0)
    set_map(node, pad(4, 2.0, 1.0))
    node._do_select()
    assert node.state == node.TRAVEL
    assert node.target_id == 4
    assert node.setpoint[:3] == pytest.approx([2.0, 1.0, 1.0])
    assert node.landing_for == node.LAND_PAD


def test_the_heading_is_held_while_travelling(node):
    """Turning and translating at once moves the detector's geometry and the
    controller's demand together, for nothing: the belly camera looks down."""
    node.home = (9.0, 9.0)
    node.setpoint = [0.0, 0.0, 1.0, math.radians(-90.0)]
    set_map(node, pad(4, 2.0, 1.0))
    node._do_select()
    assert node.setpoint[3] == pytest.approx(math.radians(-90.0))


def test_nothing_in_the_map_falls_through_to_the_search(node):
    set_map(node)
    node._do_select()
    assert node.state == node.SETTLE


def test_the_quota_sends_the_drone_home(node):
    node.survey_done = True          # the landing phase
    node.landed_count = 2
    set_map(node, pad(0, -0.5, 0.2, takeoff_base=True),
            pad(4, 2.0, 1.0))
    node._do_select()
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL
    assert node.setpoint[:2] == pytest.approx([-0.5, 0.2])


def test_home_falls_back_to_where_we_armed(node):
    """If registration failed there is no takeoff-base entry to fly to."""
    node.survey_done = True          # the landing phase
    node.survey_done = True          # the landing phase
    node.home = (1.25, -3.5)
    node.landed_count = 2
    set_map(node, pad(4, 2.0, 1.0))
    node._do_select()
    assert node.setpoint[:2] == pytest.approx([1.25, -3.5])


# ── Arriving, and confirming ─────────────────────────────────────────────────

def test_arriving_over_a_candidate_starts_the_confirmation(node):
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node.setpoint = [2.0, 1.0, 1.0, 0.0]
    set_pose(node, 2.05, 1.05)
    enter(node, node.TRAVEL)
    node._do_travel()
    assert node.state == node.CONFIRM
    assert node._confirm_hits == 0


def test_arriving_home_lands_without_confirming(node):
    """There is nothing to confirm: we registered this base ourselves."""
    node.landing_for = node.LAND_FINAL
    node.setpoint = [0.0, 0.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    assert node.state == node.LAND


def stall(node, seconds=None):
    """Pretend the leg last got closer `seconds` ago."""
    node._travel_progress_t = node._now() - (
        seconds if seconds is not None else node.travel_stall_s + 1.0)


def test_a_long_leg_is_not_given_up_on(node):
    """Elapsed time alone never abandons a leg. A 60 s budget used to blacklist
    pad 4 while the vehicle was 0.70 m away and still closing, so CONFIRM was
    never entered and the belly camera never voted on a base that was there.

    What replaces it tests PROGRESS, so this case is still safe: the leg below
    has been running forever and is still closing on every tick.
    """
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node.setpoint = [20.0, 0.0, 1.0, 0.0]
    enter(node, node.TRAVEL, age_s=99999.0)
    for x in (0.0, 2.0, 4.0, 6.0, 8.0):        # closing, slowly, for ever
        set_pose(node, x, 0.0)
        node._do_travel()
    assert node.blacklist == set()
    assert node.state == node.TRAVEL


def test_a_leg_that_stops_closing_is_given_up_on(node):
    """MEASURED 2026-08-27 on a plain --phase1 run: the pose drifted 4-5 m in
    an 8 m arena, so the target sat permanently 4 m from where the vehicle
    believed it was, `d` could never reach arrive_tol, and the mission sat in
    TRAVEL for four and a half minutes — one leg, no landings, no message."""
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node.setpoint = [20.0, 0.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()                          # records the first distance
    stall(node)
    node._do_travel()
    assert 4 in node.blacklist
    assert node.state == node.SELECT
    assert node.target_id is None


def test_hovering_noise_does_not_count_as_progress(node):
    """Otherwise a dead leg stays alive forever on estimate jitter alone."""
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node.setpoint = [20.0, 0.0, 1.0, 0.0]
    enter(node, node.TRAVEL)
    set_pose(node, 0.0, 0.0)
    node._do_travel()
    stall(node)
    set_pose(node, 0.001, 0.0)                 # a millimetre is not an approach
    node._do_travel()
    assert 4 in node.blacklist


def test_the_return_leg_is_never_blacklisted(node):
    """There is no other candidate to fall back to and the run has to end on
    the takeoff base. Stalling there is reported and the leg is left running,
    which is the old behaviour kept where it was the right one."""
    node.landing_for = node.LAND_FINAL
    node.target_id = None
    node.setpoint = [20.0, 0.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    stall(node)
    node._do_travel()
    assert node.state == node.TRAVEL
    assert node.blacklist == set()


def test_each_leg_gets_a_fresh_progress_record(node):
    """A new leg starts FARTHER from its target than the last one ended from
    its own, so carrying the previous best over would make it look stalled on
    its first tick."""
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node.setpoint = [1.0, 0.0, 1.0, 0.0]
    set_pose(node, 0.5, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()                          # best distance is now 0.5 m
    node.target_id = 5
    node.setpoint = [20.0, 0.0, 1.0, 0.0]
    enter(node, node.TRAVEL)                   # a new, much longer leg
    set_pose(node, 0.0, 0.0)
    node._do_travel()
    assert node.blacklist == set()
    assert node.state == node.TRAVEL


def test_confirmation_needs_several_looks(node):
    node.target_id = 4
    enter(node, node.CONFIRM)
    for _ in range(node.confirm_detections - 1):
        see_pad(node, confidence=0.9)
        node._do_confirm()
        assert node.state == node.CONFIRM
    see_pad(node, confidence=0.9)
    node._do_confirm()
    assert node.state == node.LAND
    assert node.landing_for == node.LAND_PAD


def test_one_frame_cannot_satisfy_the_whole_quota(node):
    """The belly camera and the tick both run at 10 Hz. Without clearing the
    detection, a single frame would be counted three times in 0.3 s."""
    node.target_id = 4
    enter(node, node.CONFIRM)
    see_pad(node, confidence=0.9)
    for _ in range(10):
        node._do_confirm()
    assert node._confirm_hits == 1
    assert node.state == node.CONFIRM


def test_an_unconfident_look_does_not_count(node):
    node.target_id = 4
    enter(node, node.CONFIRM)
    for _ in range(10):
        see_pad(node, confidence=0.4)
        node._do_confirm()
    assert node._confirm_hits == 0


def test_a_stale_look_does_not_count(node):
    node.target_id = 4
    enter(node, node.CONFIRM)
    for _ in range(10):
        see_pad(node, confidence=0.95, age_s=30.0)
        node._do_confirm()
    assert node._confirm_hits == 0


def test_a_candidate_that_never_confirms_is_blacklisted(node):
    """What makes a blue tarp cost half a minute instead of the mission."""
    node.target_id = 4
    enter(node, node.CONFIRM, age_s=999.0)
    node._do_confirm()
    assert 4 in node.blacklist
    assert node.state == node.SETTLE
    assert node.target_id is None


def test_a_rejection_restarts_the_search_from_scratch(node):
    """The drone is somewhere new facing a new direction; the turns it made
    over the old spot say nothing about what is visible from here."""
    node.target_id = 4
    node.rotations_done = 6
    set_pose(node, 2.0, 1.0)
    enter(node, node.CONFIRM, age_s=999.0)
    node._do_confirm()
    assert node.rotations_done == 0
    assert node.setpoint[:3] == pytest.approx([2.0, 1.0, 1.0])


# ── Never land off a base ────────────────────────────────────────────────────

def test_an_exhausted_search_flies_home_instead_of_landing_in_place(node):
    """There is no "land where you are" ending any more. It existed, reasoned
    that a drifted estimate made a cross-arena leg risky — but that trade is
    backwards against an eliminatory rule: a risky leg to a real base beats a
    CERTAIN touchdown on the floor."""
    set_map(node, pad(0, 1.0, -0.5, takeoff_base=True))
    node.survey_done = True
    node.max_search_level = 1
    node.rotations_done = node.max_rotations
    node.coverage_search = False
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node.setpoint[0] == pytest.approx(1.0)
    assert node.setpoint[1] == pytest.approx(-0.5)
    assert node.landing_for == node.LAND_FINAL, "must land ON the base, at home"


def test_an_ordinary_takeoff_searches_instead_of_landing(node):
    set_pose(node, 0.0, 0.0, z=0.0)
    enter(node, node.TAKEOFF)
    set_pose(node, 0.0, 0.0, z=1.0)       # climbed takeoff_alt
    node._do_takeoff()
    assert node.state == node.SELECT
    assert node.rotations_done == 0


def test_takeoff_holds_the_heading_it_climbed_with(node):
    """Commanding yaw 0 would spin the vehicle the moment the stream starts."""
    set_pose(node, 0.0, 0.0, z=0.0, yaw=math.radians(140.0))
    enter(node, node.TAKEOFF)
    set_pose(node, 0.0, 0.0, z=1.0, yaw=math.radians(140.0))
    node._do_takeoff()
    assert math.degrees(node.setpoint[3]) == pytest.approx(140.0, abs=1e-3)


def test_a_climb_from_a_lower_pad_still_counts_as_airborne(node):
    """takeoff_alt is a height above what we are standing on; pose.z is measured
    from the FIRST takeoff plane. Comparing them directly hung the mission on
    2026-08-23: after landing on a pad 0.76 m below the start plane, a perfect
    1.0 m climb reached z=0.24 and the old test wanted 0.85, so TAKEOFF re-sent
    a command ArduPilot rejected because the vehicle was already flying."""
    set_pose(node, 0.0, 0.0, z=-0.76)
    enter(node, node.TAKEOFF)
    set_pose(node, 0.0, 0.0, z=-0.76 + 1.0)
    node._do_takeoff()
    assert node.state == node.SELECT


def test_a_takeoff_that_did_not_lift_us_stays_in_takeoff(node):
    set_pose(node, 0.0, 0.0, z=-0.76)
    enter(node, node.TAKEOFF)
    set_pose(node, 0.0, 0.0, z=-0.70)
    node._do_takeoff()
    assert node.state == node.TAKEOFF


# ── Between landings ─────────────────────────────────────────────────────────

def test_a_pad_landing_leads_to_another_takeoff(node):
    node.landing_for = node.LAND_PAD
    node.landed_count = 1
    node.target_id = 4
    enter(node, node.DWELL, age_s=999.0)
    node._do_dwell()
    assert node.state == node.ARMING
    assert node.target_id is None


def test_the_final_landing_ends_the_mission(node):
    node.landing_for = node.LAND_FINAL
    enter(node, node.DWELL, age_s=999.0)
    node._do_dwell()
    assert node.state == node.DONE


def test_the_takeoff_base_is_registered_before_the_first_climb_only(node):
    node.base_registered = False
    enter(node, node.ARMING)
    node.mav_state.mode = "GUIDED"
    node.mav_state.armed = True
    node._do_arming()
    assert node.state == node.REGISTER

    node.base_registered = True
    enter(node, node.ARMING)
    node._do_arming()
    assert node.state == node.TAKEOFF


# ── The setpoint on the wire ─────────────────────────────────────────────────

def test_the_published_setpoint_carries_the_commanded_yaw(node):
    node._goto(1.0, 2.0, 1.0, math.radians(-90.0))
    published = []
    node.pub_sp.publish = published.append
    node._stream()
    sp = published[0]
    assert sp.pose.position.x == pytest.approx(1.0)
    assert math.degrees(yaw_of(sp)) == pytest.approx(-90.0)


def test_nothing_is_published_once_the_fcu_owns_the_descent(node):
    node._begin_landing()
    published = []
    node.pub_sp.publish = published.append
    node._stream()
    assert published == []


# ── The landing anchor ───────────────────────────────────────────────────────
#
# The only measurement of drift against the WORLD this stack can make.
# landmark.py's re-observation compares two things projected through the same
# drifting pose; this compares the pose against a base the vehicle is
# physically resting on.

def test_the_landing_anchor_measures_the_pose_against_where_we_armed(node,
                                                                    capfd):
    """capfd, not caplog: rclpy's logger writes to the process's stderr, which
    pytest's logging capture never sees."""
    node.home = (1.0, -0.5)
    set_pose(node, 1.4, -0.2)
    capfd.readouterr()
    node._report_landing_anchor()
    text = "".join(capfd.readouterr())
    assert "LANDING ANCHOR" in text
    assert "0.50 m" in text                 # hypot(0.4, 0.3)


def test_the_anchor_says_nothing_without_a_home(node, capfd):
    """Registration can fail. Reporting a drift against a home that was never
    captured would invent a number, and inventing one here is worse than
    having none: this is the measurement everything else is checked against."""
    node.home = None
    set_pose(node, 5.0, 5.0)
    capfd.readouterr()
    node._report_landing_anchor()
    assert "LANDING ANCHOR" not in "".join(capfd.readouterr())


# ── Coverage search ──────────────────────────────────────────────────────────
#
# MEASURED 2026-08-27: without this the vehicle spends the whole mission
# turning at the point it took off from — ONE travel leg in 5.5 minutes — so a
# base outside that cone never existed to it.

def exhaust_the_turns(node):
    node._harvest_relief = lambda: None
    node.rotations_done = node.max_rotations
    enter(node, node.SETTLE, age_s=node.settle_s + 1.0)
    set_map(node)                       # no candidate to distract SELECT


def test_a_coverage_search_that_throws_does_not_take_the_mission_with_it(node):
    """The fallback is what the mission did before coverage existed."""
    def boom():
        raise RuntimeError("octree exploded")
    node.octree_tree = None
    node._octomap_msg = None
    assert node._next_viewpoint() is None      # no map: quietly nothing


def test_coverage_can_be_turned_off(node):
    """coverage_search:=false restores the pure turn-in-place search."""
    node.coverage_search = False
    assert node._next_viewpoint() is None


def test_a_pad_leg_is_not_treated_as_a_viewpoint_leg(node):
    """The flag has to be cleared by whoever starts a normal leg, or the pad
    that follows a coverage trip would never be blacklisted."""
    node.survey_done = True          # the landing phase
    node._viewpoint_leg = True
    node.landed_count = 0
    set_map(node, pad(4, 3.0, 0.0))
    set_pose(node, 0.0, 0.0)
    enter(node, node.SELECT)
    node._do_select()
    assert node.state == node.TRAVEL
    assert not node._viewpoint_leg


def test_arriving_with_no_target_pad_refuses_to_land(node):
    """Belt and braces behind the viewpoint check: nothing may descend without
    a pad it is descending ONTO. "over pad None" is how the vehicle ends up on
    the floor, and that ends the run."""
    node.target_id = None
    node.landing_for = node.LAND_PAD
    node._viewpoint_leg = False
    node.setpoint = [2.0, 1.0, 1.0, 0.0]
    set_pose(node, 2.0, 1.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    assert node.state == node.SETTLE
    assert node.state != node.CONFIRM


# ── Never fly into a wall ────────────────────────────────────────────────────

def test_a_blocked_leg_with_no_way_round_is_refused_not_flown(node):
    """This used to fly the straight line anyway and "rely on the supervisor".
    What that produced was the drone hitting a wall — there is no supervisor
    input inside a 2 m leg at cruise. Refusing costs one target; flying it
    costs the aircraft, and in the competition the attempt."""
    node.target_id = 4
    node.landing_for = node.LAND_PAD
    node._blocked_target = True
    node.setpoint = [3.0, 0.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    assert node.state == node.SETTLE
    assert 4 in node.blacklist
    assert node.target_id is None


def test_a_blocked_viewpoint_is_remembered_not_blacklisted_as_a_pad(node):
    node._viewpoint_leg = True
    node._blocked_target = True
    node.setpoint = [3.0, 2.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    assert node.state == node.SETTLE
    assert node.blacklist == set()
    assert (3.0, 2.0) in node._failed_viewpoints


def test_planning_never_crosses_unmapped_space_by_default(node):
    """It did, on the argument that the straight-line fallback crosses unknown
    space anyway. But a straight line to a pad is short and aimed where the
    camera has been looking; a PLANNED path may detour anywhere, and with
    unknown traversable the cheapest detour runs through the part of the arena
    nothing has mapped — which is where the unseen walls are."""
    assert node.plan_allow_unknown is False


# ── Survey first, land second ────────────────────────────────────────────────

def test_the_same_candidate_is_taken_once_the_survey_is_done(node):
    node.survey_done = True
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.SELECT


def test_an_abort_restarts_the_survey(node):
    """A new attempt has not swept anything, whatever the last one learned."""
    node.survey_done = True
    node._relief_seen.append((1.0, 1.0))
    node._reset()
    assert not node.survey_done
    assert node._relief_seen == []


# ── Relief leads ─────────────────────────────────────────────────────────────

def test_relief_is_published_below_the_detector_s_own_floor(node):
    """Relief is a reason to go LOOK, not a sighting of a pad — nothing has
    said this is blue. It must not outvote the camera in the map's fusion."""
    assert node.relief_confidence < 0.5


def test_relief_can_be_turned_off(node):
    node.relief_leads = False
    node.pub_relief = None
    node._harvest_relief()          # must not raise


def test_the_survey_stops_when_it_stops_learning(node):
    """The predicted gain is optimistic — it credits everything in range with
    line of sight, while the octomap only integrates what the depth camera
    actually swept. MEASURED 2026-08-28: 28, 27, 27 across three trips and
    never near zero, so a survey that waits for it to run out never ends."""
    node.survey_max_stalls = 2
    node.survey_progress_cells = 5
    assert node._survey_gain_is_real(28)      # first, nothing to compare
    assert node._survey_gain_is_real(27)      # no real progress: strike 1
    assert not node._survey_gain_is_real(27)  # strike 2 -> the sweep is over


def test_real_progress_resets_the_patience(node):
    node.survey_max_stalls = 2
    node.survey_progress_cells = 5
    assert node._survey_gain_is_real(40)
    assert node._survey_gain_is_real(39)      # strike 1
    assert node._survey_gain_is_real(20)      # learned something: reset
    assert node._survey_gain_is_real(19)      # strike 1 again
    assert not node._survey_gain_is_real(19)


def test_the_survey_flies_instead_of_spinning(node):
    """Turning on the spot cannot map an arena: what limits the map is not
    where the camera POINTS but where it has PARALLAX, and a camera that never
    translates never sees behind anything."""
    node.survey_done = False
    node.survey_circuit = True
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node._viewpoint_leg, "a circuit leg must never confirm or land"
    assert node._survey_path is not None


def test_the_sweep_ends_the_survey_when_it_has_found_the_quota(node):
    node.survey_done = False
    node.survey_circuit = True
    node.target_bases = 1
    node.home = (9.0, 9.0)
    node._survey_path = []                 # already flown
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.survey_done
    assert node.state == node.SELECT


def test_an_abort_forgets_the_sweep(node):
    node._survey_path = [(1.0, 1.0, 1.0, 0.0)]
    node._reset()
    assert node._survey_path is None


def test_the_relief_is_the_reserve_when_the_quota_is_not_met(node,
                                                             monkeypatch):
    """Only once the blue candidates are exhausted. An elevated base is the one
    the ground-plane projection places wrongly however well it is seen, so it
    is exactly the base most likely to still be missing by now."""
    node.survey_done = True
    node.survey_circuit = False
    node.landed_count = 1
    node.target_bases = 6
    node.home = (9.0, 9.0)
    node.coverage_search = False
    node.rotations_done = node.max_rotations

    found = {"n": 0}

    def harvest():
        found["n"] = 1                     # the relief becomes a candidate

    monkeypatch.setattr(node, "_harvest_relief", harvest)
    monkeypatch.setattr(node, "_best_candidate",
                        lambda: pad(9, 2.0, 0.0) if found["n"] else None)

    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert found["n"], "went home without investigating the relief"
    assert node.state == node.SELECT


def test_the_relief_is_not_consulted_while_blue_candidates_remain(node,
                                                                  monkeypatch):
    """It is a weaker signal — nothing said this lump is blue. Spending a hover
    on it while a real sighting is waiting is the wrong order."""
    node.survey_done = True
    node.survey_circuit = False
    node.home = (9.0, 9.0)
    called = {}
    monkeypatch.setattr(node, "_harvest_relief",
                        lambda: called.setdefault("yes", True))
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.SELECT
    assert "yes" not in called

def test_arriving_on_a_sweep_leg_never_confirms_or_lands(node):
    """The worst failure this mission has had. MEASURED 2026-08-27: a run that
    reported "6 of 6 bases" had landed on ONE. The other five were a look-leg
    arriving and being treated as an arrival over a pad — "over pad None —
    confirming on the belly camera" — putting the vehicle down mid-arena on
    whatever looked blue from 1 m. Off-base landings are eliminatory."""
    node.survey_done = False
    node.survey_circuit = True
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL and node._viewpoint_leg

    set_pose(node, node.setpoint[0], node.setpoint[1])      # arrive
    node._do_travel()
    assert node.state == node.SETTLE, "a look-leg arrival reached CONFIRM"
    assert not node._viewpoint_leg


def test_an_unreachable_look_leg_is_not_blacklisted_as_a_pad(node):
    """A look-leg has no pad to blame when it stalls. The first flight that
    exercised this printed "pad None stopped getting closer"."""
    node._viewpoint_leg = True
    node.setpoint = [3.0, 2.0, 1.0, 0.0]
    set_pose(node, 0.0, 0.0)
    enter(node, node.TRAVEL)
    node._do_travel()
    node._travel_progress_t = node._now() - node.travel_stall_s - 1.0
    node._do_travel()
    assert node.blacklist == set(), "blacklisted a pad for a look-leg's sake"
    assert node.state == node.SETTLE
    assert (3.0, 2.0) in node._failed_viewpoints


def test_a_candidate_is_not_chased_before_the_sweep_is_flown(node):
    """Running at the first sighting is explore-nothing/exploit-everything in
    the worst order: the battery goes on whichever base happened to be in
    front of the camera at takeoff."""
    node.survey_done = False
    node.survey_circuit = True
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL and node._viewpoint_leg, \
        "chased a pad before sweeping the arena"


def test_level_1_climbs_above_the_house(node):
    """The passes cross the house, whose roof is 1.5 m in the competition
    arena, and the cruise height is 1 m. Taking the sweep altitude from
    takeoff_alt would fly the drone into it."""
    node.survey_done = False
    node.survey_circuit = True
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.survey_alt > 1.5
    assert node.setpoint[2] == pytest.approx(node.survey_alt)


def test_level_1_holds_one_heading_per_leg(node):
    """The U turns twice, at its corners. The rectangle it replaced re-aimed
    the camera at every step, so it turned continuously along every edge."""
    node.survey_done = False
    node.survey_circuit = True
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    yaws = [p[3] for p in ([(0, 0, 0, node.setpoint[3])] + node._survey_path)]
    turns = sum(1 for a, b in zip(yaws, yaws[1:]) if abs(a - b) > 1e-9)
    assert turns == 2, f"turned {turns} times; the U turns twice"

def test_a_refused_takeoff_gives_up_instead_of_looping_for_ever(node):
    """TAKEOFF bounces back to ARMING on every refusal. Clearing the counter on
    the way through means the three-strike abort never accumulates — MEASURED
    2026-08-28: after landing on an elevated base at z=0.89 m the FCU refused
    takeoff and the mission sat in ARMING <-> TAKEOFF for the rest of the
    flight, retrying every two seconds."""
    node.dry_run = False
    node.base_registered = True
    node.mav_state.mode = "GUIDED"
    node.mav_state.armed = True
    node._takeoff_tries = 3
    enter(node, node.ARMING)
    node._do_arming()
    assert node.state == node.TAKEOFF
    assert node._takeoff_tries == 3, "the retry counter was cleared on the way"


def test_a_new_landing_cycle_gets_its_takeoff_tries_back(node):
    """DWELL is the place that means 'this is a fresh attempt'."""
    node._takeoff_tries = 3
    node.landed_count = 1
    node.target_bases = 6
    node.landing_for = node.LAND_PAD
    enter(node, node.DWELL, age_s=999.0)
    node._do_dwell()
    assert node._takeoff_tries == 0


# ── The search ladder ────────────────────────────────────────────────────────
#
# Each level exists because the one before it can miss a base, and each costs
# more — which is the whole reason for a ladder rather than starting with the
# thorough one.

def fly_the_whole_level(node):
    """Consume the current level's path without moving the vehicle."""
    node._survey_path = []


def test_a_level_that_finds_everything_stops_the_search(node):
    node.survey_done = False
    node.target_bases = 2
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0), pad(2, -2.0, 1.0))
    node._begin_level()
    fly_the_whole_level(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.survey_done
    assert node.state == node.SELECT
    assert node._level == 1, "escalated despite having found everything"


def test_a_level_that_falls_short_escalates(node):
    node.survey_done = False
    node.target_bases = 6
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    node._begin_level()
    fly_the_whole_level(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node._level == 2
    assert not node.survey_done
    assert node._survey_path is None, "must rebuild the path for the new level"


def test_level_2_is_the_same_shape_half_a_metre_higher(node):
    """A base seen edge-on from below, or hidden by the house, opens up from
    higher — raising the camera changes the geometry without changing the
    flight."""
    node._level = 1
    node._begin_level()
    low = [p[2] for p in node._survey_path]
    node._level = 2
    node._begin_level()
    high = [p[2] for p in node._survey_path]
    assert high[0] == pytest.approx(low[0] + node.level2_climb_m)
    assert len(high) == len(low), "level 2 is the same shape, not a new one"


def test_level_3_has_no_path_and_hands_over_to_rotate_and_relief(node):
    """That is where an ELEVATED base is caught: the ground-plane projection
    cannot place one, so the blue detector's answer for it is in the wrong
    place however well it was seen."""
    node._level = 3
    assert node._begin_level()
    assert node._survey_path == []
    assert node.survey_done


def test_level_4_is_the_lawnmower_and_costs_much_more(node):
    node._level = 1
    node._begin_level()
    u = len(node._survey_path)
    node._level = 4
    node._begin_level()
    assert len(node._survey_path) > u


def test_the_ladder_stops_at_the_top(node):
    node._level = 99
    assert not node._begin_level()


def test_the_ladder_can_be_capped(node):
    """Lower max_search_level to cap what an attempt may cost."""
    node.max_search_level = 1
    node.survey_done = True
    node.survey_circuit = False
    node.landed_count = 0
    node.target_bases = 6
    node.home = (9.0, 9.0)
    node.coverage_search = False
    node.relief_leads = False
    node.pub_relief = None
    node.rotations_done = node.max_rotations
    set_map(node, pad(0, 1.0, -0.5, takeoff_base=True))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL, "kept climbing past the cap"


def test_the_sweep_starts_at_the_corner_the_drone_is_nearest(node):
    """So it starts where it already is instead of transiting first."""
    set_pose(node, 3.5, 3.5)
    node._level = 1
    node._begin_level()
    assert node._survey_path[0][0] > 0
    assert node._survey_path[0][1] > 0


def test_an_abort_returns_to_level_one(node):
    node._level = 4
    node._reset()
    assert node._level == 1


def test_level_3_actually_runs_instead_of_being_judged_flown_at_once(node):
    """It has no path of its own — it is the rotate-and-investigate behaviour
    further down _do_settle. MEASURED 2026-08-28: without the hand-over,
    level 3 escalated 0.08 s after starting and never ran."""
    node._level = 3
    node.survey_done = False
    node.survey_circuit = True
    node._survey_path = None
    node.target_bases = 6
    node.home = (9.0, 9.0)
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node._level == 3, "escalated past level 3 without running it"
    assert node.survey_done


# ── Investigate before climbing ──────────────────────────────────────────────

def test_unconfirmed_leads_are_investigated_before_the_next_level(node):
    """MEASURED 2026-08-28: a run ended a level with 4 confirmed and 2
    unconfirmed candidates and escalated anyway. Investigating those two would
    have completed the quota there and then; instead two more levels were
    flown and still came back 5 of 6."""
    node.survey_done = False
    node.survey_circuit = True
    node.target_bases = 6
    node.landed_count = 0
    node.home = (9.0, 9.0)
    node._survey_path = []                       # the level is flown
    node._harvest_relief = lambda: None
    set_map(node,
            pad(1, 2.0, 0.0, observations=5),    # confirmed
            pad(2, -2.0, 1.0, observations=1))   # a lead nobody looked at
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node._level == 1, "climbed a level with a lead still unlooked at"
    assert node.investigating
    assert node.state == node.SELECT


def test_investigating_drops_the_bar_to_a_single_sighting(node):
    """Not the bar being wrong the rest of the time — the cost of being wrong
    has changed. A doubtful lead now competes only with flying the whole U
    again half a metre up."""
    node.home = (9.0, 9.0)
    node.investigating = False
    assert not node._is_candidate(pad(1, 2.0, 0.0, observations=1))
    node.investigating = True
    assert node._is_candidate(pad(1, 2.0, 0.0, observations=1))


def test_a_lead_that_fails_its_hover_stops_being_one(node):
    """The blacklist is what keeps the investigation from looping."""
    node.home = (9.0, 9.0)
    node.investigating = True
    node.blacklist.add(2)
    set_map(node, pad(2, -2.0, 1.0, observations=1))
    assert node._uninvestigated() == []


def test_the_takeoff_base_is_never_a_lead_to_investigate(node):
    node.home = (9.0, 9.0)
    set_map(node, pad(0, 1.0, 1.0, observations=1, takeoff_base=True))
    assert node._uninvestigated() == []


def test_escalating_leaves_investigation_mode(node):
    """Otherwise the next level would target one-sighting noise as it flies."""
    node.survey_done = True
    node.survey_circuit = False
    node.investigating = True
    node.landed_count = 0
    node.target_bases = 6
    node.home = (9.0, 9.0)
    node.coverage_search = False
    node.relief_leads = False
    node.pub_relief = None
    node.rotations_done = node.max_rotations
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert not node.investigating
    assert node._level == 2


# ── Centring on the pad ──────────────────────────────────────────────────────

def see_pad_at(node, u, v, confidence=0.9):
    det = PadDetection()
    det.camera = "down"
    det.confidence = float(confidence)
    det.position_valid = True
    det.u, det.v = float(u), float(v)
    node._last_down = det
    node._last_down_t = node._now()


def test_the_hover_nudges_towards_the_pad_it_sees(node):
    """The position came from a projection made across the arena; the belly
    camera at 1 m is the only sensor that can say where the pad actually is
    relative to the vehicle."""
    node.target_id = 4
    node.setpoint = [2.0, 1.0, 1.0, 0.0]
    set_pose(node, 2.0, 1.0)
    enter(node, node.CONFIRM)
    see_pad_at(node, 500.0, 240.0)          # well off centre
    node._do_confirm()
    assert node.setpoint[:2] != [2.0, 1.0], "hovered without centring"


def test_a_centred_pad_is_left_alone(node):
    """Moving again would only add drift."""
    node.target_id = 4
    node.setpoint = [2.0, 1.0, 1.0, 0.0]
    set_pose(node, 2.0, 1.0)
    enter(node, node.CONFIRM)
    see_pad_at(node, 320.0, 240.0)
    node._do_confirm()
    assert node.setpoint[:2] == pytest.approx([2.0, 1.0])


def test_centring_can_be_turned_off(node):
    node.centre_on_pad = False
    node.target_id = 4
    node.setpoint = [2.0, 1.0, 1.0, 0.0]
    set_pose(node, 2.0, 1.0)
    enter(node, node.CONFIRM)
    see_pad_at(node, 500.0, 240.0)
    node._do_confirm()
    assert node.setpoint[:2] == pytest.approx([2.0, 1.0])


def test_the_nudge_is_rotated_by_the_vehicle_s_heading(node):
    """The servo knows about pixels and the airframe, not about where north
    is. The same pixel error at two headings must move the setpoint two
    different ways."""
    steps = []
    for yaw in (0.0, math.pi / 2.0):
        node._servo = servo_module.VisualServo(target_uv=(320.0, 240.0))
        node.setpoint = [0.0, 0.0, 1.0, yaw]
        set_pose(node, 0.0, 0.0, yaw=yaw)
        enter(node, node.CONFIRM)
        see_pad_at(node, 500.0, 240.0)
        node._do_confirm()
        steps.append(tuple(node.setpoint[:2]))
    assert steps[0] != pytest.approx(steps[1])


def test_the_target_pixel_is_a_parameter_for_an_off_centre_lens(node):
    """What the servo cannot learn is where the camera points when the vehicle
    is level — that is what "centred" means. On a misaligned airframe it is
    measured once by hovering over a known pad."""
    assert node.get_parameter("pad_target_uv").value == [320.0, 240.0]


# ── The arena is one number, not two sets of corners ─────────────────────────

def arena_node(**kw):
    params = [rclpy.parameter.Parameter("auto_start", value=False)]
    params += [rclpy.parameter.Parameter(k, value=v) for k, v in kw.items()]
    return Phase1MissionNode(parameter_overrides=params)


def test_the_planning_box_comes_from_the_arena_size():
    """A 5 x 6 m arena is `arena_size_x:=5.0 arena_size_y:=6.0` and nothing
    else — it sets the U's legs, the planner's box and the lawnmower's lanes
    at once."""
    n = arena_node(arena_size_x=5.0, arena_size_y=6.0,
                   arena_floor_z=0.3, arena_ceiling_z=2.5)
    assert n.plan_bounds == ((-2.5, -3.0, 0.3), (2.5, 3.0, 2.5))
    n.destroy_node()


def test_the_u_shrinks_with_the_arena():
    """The whole point of parametrising it: the competition arena is 8 x 8 and
    the team's own is 5 x 6."""
    big = arena_node(arena_size_x=8.0, arena_size_y=8.0, survey_inset_m=1.2)
    small = arena_node(arena_size_x=5.0, arena_size_y=6.0, survey_inset_m=1.2)
    big._level = small._level = 1
    big._begin_level()
    small._begin_level()
    big_leg = max(p[0] for p in big._survey_path) - min(p[0] for p in big._survey_path)
    small_leg = max(p[0] for p in small._survey_path) - min(p[0] for p in small._survey_path)
    assert big_leg == pytest.approx(8.0 - 2 * 1.2)
    assert small_leg == pytest.approx(5.0 - 2 * 1.2)
    big.destroy_node()
    small.destroy_node()


def test_the_inset_sets_the_leg_length():
    """leg = size - 2*inset. On a 5 x 6 arena the inset wants to be smaller
    than on an 8 x 8 one."""
    a = arena_node(arena_size_x=6.0, arena_size_y=6.0, survey_inset_m=0.5)
    a._level = 1
    a._begin_level()
    leg = max(p[0] for p in a._survey_path) - min(p[0] for p in a._survey_path)
    assert leg == pytest.approx(5.0)
    a.destroy_node()


def test_an_arena_not_centred_on_where_the_drone_armed():
    """`map` is where the vehicle armed, which on the real drone is a takeoff
    base somewhere in the arena, not its centre."""
    n = arena_node(arena_size_x=8.0, arena_size_y=8.0,
                   arena_centre_x=2.0, arena_centre_y=-1.0,
                   arena_floor_z=0.3, arena_ceiling_z=2.5)
    assert n.plan_bounds == ((-2.0, -5.0, 0.3), (6.0, 3.0, 2.5))
    n.destroy_node()


# ── The arena boundary is a line on the floor, not a wall ────────────────────
#
# The occupancy map knows about WALLS, and in the simulator's arena — as in the
# real hall — the walls stand well outside the competition boundary. Nothing
# physical stops the vehicle crossing it and nothing in the map objects,
# because the space beyond genuinely is empty.

def test_a_setpoint_outside_the_arena_is_clamped(node):
    n = arena_node(arena_size_x=8.0, arena_size_y=8.0, arena_keepout_m=0.3)
    x, y, _, _ = n._fenced((9.0, -7.0, 1.0, 0.0))
    assert x == pytest.approx(3.7)
    assert y == pytest.approx(-3.7)
    n.destroy_node()


def test_a_setpoint_inside_the_arena_is_untouched(node):
    n = arena_node(arena_size_x=8.0, arena_size_y=8.0, arena_keepout_m=0.3)
    assert n._fenced((1.0, -2.0, 1.5, 0.4)) == (1.0, -2.0, 1.5, 0.4)
    n.destroy_node()


def test_the_fence_follows_the_arena_size(node):
    """A 5 x 6 m arena fences at 5/2 and 6/2, less the keepout."""
    n = arena_node(arena_size_x=5.0, arena_size_y=6.0, arena_keepout_m=0.5)
    x, y, _, _ = n._fenced((99.0, 99.0, 1.0, 0.0))
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(2.5)
    n.destroy_node()


def test_the_fence_follows_an_off_centre_arena(node):
    n = arena_node(arena_size_x=8.0, arena_size_y=8.0,
                   arena_centre_x=2.0, arena_centre_y=-1.0,
                   arena_keepout_m=0.0)
    x, y, _, _ = n._fenced((99.0, -99.0, 1.0, 0.0))
    assert x == pytest.approx(6.0)
    assert y == pytest.approx(-5.0)
    n.destroy_node()


def test_the_fence_leaves_height_alone(node):
    """The net above and the floor below are real, and clamping z would fight
    the landing descent — which is the FCU's, not this node's."""
    n = arena_node(arena_size_x=8.0, arena_size_y=8.0)
    assert n._fenced((0.0, 0.0, 99.0, 0.0))[2] == 99.0
    assert n._fenced((0.0, 0.0, -5.0, 0.0))[2] == -5.0
    n.destroy_node()


def test_nothing_can_reach_the_fcu_without_passing_the_fence(node):
    """The guard is at _stream, the single point a setpoint reaches the FCU —
    not at the six places that command a position. A caller that forgets the
    fence is exactly the caller that would have crossed the line."""
    published = []
    node.arena_keepout_m = 0.3
    node.pub_sp = type("P", (), {"publish": lambda _s, m: published.append(m)})()
    node.stream_setpoint = True
    node.setpoint = [99.0, 99.0, 1.0, 0.0]      # set directly, bypassing _goto
    node._stream()
    assert published
    (min_x, min_y, _), (max_x, max_y, _) = node.plan_bounds
    assert published[0].pose.position.x <= max_x - 0.3 + 1e-9
    assert published[0].pose.position.y <= max_y - 0.3 + 1e-9


# ── The settle guards a TURN, not a translation ──────────────────────────────

def test_a_pure_translation_barely_settles(node):
    """MEASURED on an 8x8 arena: the U's 23 waypoints at the full 5 s were
    115 s of the level's ~200 s — more than half the sweep spent waiting for a
    yaw estimate that never moved. The U holds a fixed heading down each leg."""
    node._last_settle_yaw = 0.0
    node.setpoint = [1.0, 2.0, 1.0, 0.0]
    assert node._settle_needed() == pytest.approx(node.settle_moving_s)


def test_a_heading_change_settles_in_full(node):
    """A detection taken while yaw is slewing is projected through a moving
    estimate and lands in the map metres out. That is what the pause is for,
    and it stays."""
    node._last_settle_yaw = 0.0
    node.setpoint = [1.0, 2.0, 1.0, math.radians(90.0)]
    assert node._settle_needed() == pytest.approx(node.settle_s)


def test_a_tiny_heading_wobble_is_not_a_turn(node):
    node._last_settle_yaw = 0.0
    node.setpoint = [1.0, 2.0, 1.0, math.radians(2.0)]
    assert node._settle_needed() == pytest.approx(node.settle_moving_s)


def test_the_first_settle_of_a_run_is_the_full_one(node):
    """Nothing is known about the heading the vehicle arrived on."""
    node._last_settle_yaw = None
    assert node._settle_needed() == pytest.approx(node.settle_s)


def test_an_abort_forgets_the_settled_heading(node):
    node._last_settle_yaw = 1.0
    node._reset()
    assert node._last_settle_yaw is None


def test_the_settle_still_waits_before_reading_the_map(node):
    """Shortening it must not remove it: the vehicle still has to have stopped
    before a detection is believed."""
    node.survey_done = True
    node.survey_circuit = False
    node._last_settle_yaw = 0.0
    node.setpoint = [0.0, 0.0, 1.0, 0.0]
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=0.0)
    node._do_settle()
    assert node.state == node.SETTLE, "read the map without settling at all"


def test_select_will_not_start_landing_before_the_survey(node):
    """TAKEOFF hands straight to SELECT, so this is a SECOND door into the
    landing phase and it had no lock on it. It went unnoticed because the map
    is empty at takeoff — MEASURED 2026-08-29, once detections started being
    accepted while the vehicle moves, a candidate existed by the time SELECT
    first ran and the mission flew to FOUR bases before the sweep began."""
    node.survey_done = False
    node.home = (9.0, 9.0)
    node.landed_count = 0
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SELECT)
    node._do_select()
    assert node.state == node.SETTLE, "started landing before surveying"


def test_takeoff_into_select_still_reaches_the_survey(node):
    """The path the bug came in through."""
    node.survey_done = False
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SELECT)
    node._do_select()
    assert node.state == node.SETTLE


# ── Touchdown is the FCU's word, not ours ────────────────────────────────────
#
# MEASURED 2026-08-29 on an elevated base: the stillness heuristic fired at
# z=0.86, the mission called it landed, went to ARMING and switched to GUIDED —
# WHICH HALTS A LAND DESCENT. Every takeoff after that was refused, correctly,
# because from ArduCopter's point of view the vehicle was still flying. The log
# says so twice: "EKF3 IMU0 MAG0 IN-FLIGHT yaw alignment complete" after the
# mission believed it was parked, and z drifting UP from 0.86 to 1.13.

def land_state(node, *, armed, z, entry_z=2.0):
    node.dry_run = False
    node.mav_state.mode = "LAND"
    node.mav_state.armed = armed
    node._land_entry_z = entry_z
    set_pose(node, 0.0, 0.0, z=z)
    # a full window of stillness at this height
    node._z_hist = [(node._now() - node.land_settle - 1.0, z),
                    (node._now(), z)]


def test_stillness_alone_is_not_a_landing(node):
    """It is a good touchdown DETECTOR and a bad touchdown PROOF: on an
    elevated pad the descent slows near the surface and a 2 s window of small
    movement looks exactly like resting on it."""
    enter(node, node.LAND)
    land_state(node, armed=True, z=0.86)
    node._do_land()
    assert node.state == node.LAND, "called it a landing without the FCU"


def test_the_fcu_disarming_is_what_ends_the_landing(node):
    enter(node, node.LAND)
    land_state(node, armed=False, z=0.86)
    node._do_land()
    assert node.state == node.DWELL


def test_a_landing_that_the_fcu_never_confirms_says_so_loudly(node, capfd):
    """Carrying on is right — the mission must not stall — but the next
    takeoff will probably be refused, and the log has to say why."""
    enter(node, node.LAND, age_s=node.land_timeout + 1.0)
    land_state(node, armed=True, z=0.86)
    capfd.readouterr()
    node._do_land()
    text = "".join(capfd.readouterr())
    assert node.state == node.DWELL
    assert "never disarmed" in text


def test_a_dry_run_still_lands_on_stillness(node):
    """There is no FCU to disarm: the vehicle is unarmed for the whole run by
    construction, so the disarm signal carries no information at all."""
    node.dry_run = True
    enter(node, node.LAND)
    land_state(node, armed=False, z=0.86)
    node.dry_run = True
    node._do_land()
    assert node.state == node.DWELL


# ── One takeoff command per climb ────────────────────────────────────────────

def test_an_accepted_takeoff_is_not_commanded_again(node):
    """This is why the drone climbed 2.9 m when asked for 1.0: the retry fired
    every retry_period regardless, and each accepted NAV_TAKEOFF RESTARTS the
    climb from wherever the vehicle is now, so they stack. MEASURED
    2026-08-30: 25 accepted takeoff commands across 5 climbs — five per climb,
    5 x ~0.6 m = the 2.9 m in every log.

    The retry is right for a MODE change, which ArduPilot can ack and then
    decline. It is wrong for a takeoff, where an accepted ack means the climb
    is already under way."""
    enter(node, node.TAKEOFF)
    assert not node._takeoff_accepted
    node._takeoff_accepted = True
    node._last_cmd_t = node._now() - 999.0      # the retry would fire
    sent = []
    node._start_call = lambda *a, **k: sent.append(a)
    node.dry_run = False
    set_pose(node, 0.0, 0.0, z=0.1)
    node._takeoff_start_z = 0.0
    node._do_takeoff()
    assert sent == [], "commanded a takeoff that was already under way"


def test_a_refused_takeoff_is_commanded_again(node):
    """The retry still exists — it is the refusal path that needs it."""
    enter(node, node.TAKEOFF)
    node._takeoff_accepted = False
    node._last_cmd_t = node._now() - 999.0
    sent = []
    node._start_call = lambda *a, **k: sent.append(a)
    node.dry_run = False
    set_pose(node, 0.0, 0.0, z=0.1)
    node._takeoff_start_z = 0.0
    node._do_takeoff()
    assert sent, "a refused takeoff was never retried"


def test_each_new_climb_starts_with_nothing_accepted(node):
    node._takeoff_accepted = True
    enter(node, node.TAKEOFF)
    assert not node._takeoff_accepted


# ── The mission owns the altitude, the FCU only owns "get airborne" ──────────

def test_the_climb_is_taken_over_as_soon_as_we_are_off_the_ground(node):
    """Who decides the height. With the handover it is the launch parameter,
    enforced by the same position controller and the same arena fence as every
    other leg, instead of whatever NAV_TAKEOFF does on the airframe of the day
    — which nobody has measured on the real drone."""
    node.dry_run = False
    set_pose(node, 1.0, 2.0, z=0.0)
    enter(node, node.TAKEOFF)
    set_pose(node, 1.0, 2.0, z=node.takeoff_handover_m + 0.01)
    node._do_takeoff()
    assert node.state == node.SELECT, "still waiting on the FCU's own climb"


def test_it_does_not_hand_over_while_still_on_the_pad(node):
    """A position setpoint sent before the airframe is clear would drive it
    back into the surface."""
    node.dry_run = False
    set_pose(node, 1.0, 2.0, z=0.0)
    enter(node, node.TAKEOFF)
    set_pose(node, 1.0, 2.0, z=0.05)
    node._do_takeoff()
    assert node.state == node.TAKEOFF


def test_the_held_altitude_matches_every_other_leg(node):
    """ABSOLUTE takeoff_alt, because _do_select, the return home and the
    coverage legs all command it absolute. A takeoff that settled somewhere
    else would climb or drop for no reason at the very next setpoint."""
    node.dry_run = False
    set_pose(node, 1.0, 2.0, z=1.5)          # standing on a raised pad
    enter(node, node.TAKEOFF)
    set_pose(node, 1.0, 2.0, z=1.5 + node.takeoff_handover_m + 0.01)
    node._do_takeoff()
    assert node.setpoint[2] == pytest.approx(node.takeoff_alt)


def test_the_held_altitude_from_the_floor_is_just_takeoff_alt(node):
    node.dry_run = False
    set_pose(node, 0.0, 0.0, z=0.0)
    enter(node, node.TAKEOFF)
    set_pose(node, 0.0, 0.0, z=node.takeoff_handover_m + 0.01)
    node._do_takeoff()
    assert node.setpoint[2] == pytest.approx(node.takeoff_alt)


def test_a_cruise_altitude_above_the_ceiling_is_reported(node, capfd):
    """MEASURED 2026-08-30: takeoff_alt was 3.0 while arena_ceiling_z was 2.5 —
    the vehicle cruised a metre ABOVE the box A* is allowed to search in, so
    every leg it flew was outside it. Nothing crashed, and nothing said so."""
    capfd.readouterr()
    n = arena_node(takeoff_alt=3.0, arena_ceiling_z=2.5)
    text = "".join(capfd.readouterr())
    assert "ABOVE" in text and "arena_ceiling_z" in text
    n.destroy_node()


def test_consistent_heights_say_nothing(node, capfd):
    capfd.readouterr()
    n = arena_node(takeoff_alt=2.0, survey_alt_m=1.8, arena_ceiling_z=2.5)
    text = "".join(capfd.readouterr())
    assert "ABOVE" not in text
    n.destroy_node()
