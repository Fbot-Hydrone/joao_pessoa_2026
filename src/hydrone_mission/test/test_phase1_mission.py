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
    node.home = (9.0, 9.0)
    set_map(node, pad(1, 2.0, 0.0))
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.SELECT


def test_settling_with_nothing_in_the_map_starts_a_turn(node):
    set_map(node)
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert node.state == node.ROTATE


def test_the_turn_is_clockwise(node):
    """ENU yaw runs counter-clockwise from east, so clockwise SUBTRACTS."""
    set_map(node)
    node.setpoint = [0.0, 0.0, 1.0, 0.0]
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert math.degrees(node.setpoint[3]) == pytest.approx(-45.0)


def test_the_turn_wraps_rather_than_winding_up(node):
    set_map(node)
    node.setpoint = [0.0, 0.0, 1.0, math.radians(-170.0)]
    enter(node, node.SETTLE, age_s=5.0)
    node._do_settle()
    assert math.degrees(node.setpoint[3]) == pytest.approx(145.0)


def test_the_turn_does_not_move_the_vehicle(node):
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
    node.landed_count = 2
    set_map(node, pad(0, -0.5, 0.2, takeoff_base=True),
            pad(4, 2.0, 1.0))
    node._do_select()
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL
    assert node.setpoint[:2] == pytest.approx([-0.5, 0.2])


def test_home_falls_back_to_where_we_armed(node):
    """If registration failed there is no takeoff-base entry to fly to."""
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
    node.rotations_done = node.max_rotations
    enter(node, node.SETTLE, age_s=node.settle_s + 1.0)
    set_map(node)                       # no candidate to distract SELECT


def test_a_finished_sweep_repositions_instead_of_giving_up(node, monkeypatch):
    monkeypatch.setattr(node, "_next_viewpoint", lambda: ((3.0, 2.0), 12))
    exhaust_the_turns(node)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node.setpoint[0] == pytest.approx(3.0)
    assert node.setpoint[1] == pytest.approx(2.0)
    assert node.rotations_done == 0, "the new spot gets a full sweep of its own"


def test_repositioning_targets_no_pad(node, monkeypatch):
    """The trip is to LOOK, not to land. Leaving target_id set would make the
    arrival confirm and land on whatever happened to be there."""
    node.target_id = 7
    monkeypatch.setattr(node, "_next_viewpoint", lambda: ((3.0, 2.0), 12))
    exhaust_the_turns(node)
    node._do_settle()
    assert node.target_id is None


def test_nothing_left_to_see_ends_the_run_at_the_takeoff_base(node,
                                                              monkeypatch):
    """None means the search is FINISHED, not merely out of turns — and that
    is the case that justifies going home."""
    monkeypatch.setattr(node, "_next_viewpoint", lambda: None)
    exhaust_the_turns(node)
    node._do_settle()
    assert node.state == node.TRAVEL
    assert node.landing_for == node.LAND_FINAL


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


def test_an_unreachable_viewpoint_is_not_blacklisted_as_a_pad(node,
                                                              monkeypatch):
    """A coverage leg is going somewhere to LOOK. There is no pad to blame,
    and "pad None stopped getting closer" is what this printed on the first
    flight that exercised it."""
    monkeypatch.setattr(node, "_next_viewpoint", lambda: ((3.0, 2.0), 12))
    exhaust_the_turns(node)
    node._do_settle()
    assert node.state == node.TRAVEL and node._viewpoint_leg

    set_pose(node, 0.0, 0.0)
    node._do_travel()
    node._travel_progress_t = node._now() - node.travel_stall_s - 1.0
    node._do_travel()

    assert node.blacklist == set(), "blacklisted a pad for a viewpoint's sake"
    assert node.state == node.SETTLE
    assert (3.0, 2.0) in node._failed_viewpoints


def test_a_pad_leg_is_not_treated_as_a_viewpoint_leg(node):
    """The flag has to be cleared by whoever starts a normal leg, or the pad
    that follows a coverage trip would never be blacklisted."""
    node._viewpoint_leg = True
    node.landed_count = 0
    set_map(node, pad(4, 3.0, 0.0))
    set_pose(node, 0.0, 0.0)
    enter(node, node.SELECT)
    node._do_select()
    assert node.state == node.TRAVEL
    assert not node._viewpoint_leg


def test_arriving_at_a_viewpoint_never_confirms_or_lands(node, monkeypatch):
    """The worst failure this mission has had. MEASURED 2026-08-27: a run that
    reported "6 of 6 bases" had landed on ONE. The other five were a coverage
    leg arriving and being treated as an arrival over a pad — "over pad None —
    confirming on the belly camera" — putting the vehicle down mid-arena on
    whatever looked blue from 1 m. Off-base landings are eliminatory."""
    monkeypatch.setattr(node, "_next_viewpoint", lambda: ((3.0, 2.0), 12))
    exhaust_the_turns(node)
    node._do_settle()
    assert node.state == node.TRAVEL and node._viewpoint_leg

    node.setpoint = [3.0, 2.0, 1.0, 0.0]
    set_pose(node, 3.0, 2.0)             # arrive
    node._do_travel()

    assert node.state == node.SETTLE, "a viewpoint arrival reached CONFIRM"
    assert not node._viewpoint_leg
    assert node.rotations_done == 0, "the new vantage point gets a full sweep"


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
