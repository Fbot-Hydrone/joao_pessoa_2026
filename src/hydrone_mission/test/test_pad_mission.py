#!/usr/bin/env python3
"""
Tests for pad_mission_node's search pattern and target selection.

These are the two pieces of mission logic whose failure modes are expensive and
silent: a search pattern that leaves the arena flies the drone into the empty
plane, and target selection that forgets a `visited` flag lands on the same pad
until the battery runs out. Both are pure functions of state, so both can be
pinned without a simulator.

The flight states themselves (arming, takeoff, landing) are not covered here —
they are conversations with ArduPilot, and mocking one proves nothing about the
real vehicle. They are exercised by flying the sim; see docs/LANDING-SITES.md.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_mission/test/test_pad_mission.py -q'
"""

import math

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped

from hydrone_msgs.msg import Pad, PadDetection, PadMap
from hydrone_mission.pad_mission_node import PadMissionNode, spiral_waypoints


# ── The search pattern ───────────────────────────────────────────────────────

def test_spiral_first_leg_runs_straight_ahead():
    """'Take off and go forward' must literally be the first thing it does."""
    points = spiral_waypoints(step=3.0, radius=12.0, heading=0.0)
    assert points[0] == pytest.approx((3.0, 0.0))


def test_spiral_first_leg_follows_the_takeoff_heading():
    """Facing +Y, forward is +Y — the pattern is body-relative, not world-fixed."""
    points = spiral_waypoints(step=3.0, radius=12.0, heading=math.pi / 2.0)
    assert points[0][0] == pytest.approx(0.0, abs=1e-9)
    assert points[0][1] == pytest.approx(3.0, abs=1e-9)


def test_spiral_stays_inside_the_search_radius():
    """The whole point of bounding the search: the arena is an island in an
    otherwise empty, featureless plane."""
    radius = 12.0
    for point in spiral_waypoints(step=3.0, radius=radius):
        assert math.hypot(*point) <= radius + 1e-9


def test_spiral_terminates_and_covers_the_area():
    """It must be a finite list — an unbounded generator would never let the
    mission conclude 'there is nothing left to find'."""
    points = spiral_waypoints(step=3.0, radius=12.0)
    assert 20 < len(points) < 400
    # It really spirals: later waypoints get further out than the first ones.
    assert math.hypot(*points[-1]) > math.hypot(*points[0])


def test_spiral_never_repeats_a_waypoint():
    points = spiral_waypoints(step=2.0, radius=10.0)
    rounded = {(round(x, 6), round(y, 6)) for x, y in points}
    assert len(rounded) == len(points)


def test_spiral_expands_outward_over_time():
    """Near ground first, far ground later: the pad is most likely close by."""
    points = spiral_waypoints(step=2.0, radius=20.0)
    first_half = max(math.hypot(*p) for p in points[:len(points) // 4])
    last_half = max(math.hypot(*p) for p in points[-len(points) // 4:])
    assert last_half > first_half


def test_degenerate_spiral_parameters_give_nothing():
    assert spiral_waypoints(step=0.0, radius=10.0) == []
    assert spiral_waypoints(step=3.0, radius=0.0) == []
    assert spiral_waypoints(step=-1.0, radius=10.0) == []


# ── Target selection ─────────────────────────────────────────────────────────

@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PadMissionNode(parameter_overrides=[
        rclpy.parameter.Parameter("auto_start", value=False),
        rclpy.parameter.Parameter("min_observations", value=3),
    ])
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    n.pose = pose
    yield n
    n.destroy_node()


def make_pad(pad_id, x, y, observations=5, visited=False, confidence=0.9):
    pad = Pad()
    pad.id = pad_id
    pad.position.x = float(x)
    pad.position.y = float(y)
    pad.observations = observations
    pad.visited = visited
    pad.confidence = confidence
    return pad


def set_map(node, *pads):
    msg = PadMap()
    msg.header.frame_id = "map"
    msg.pads = list(pads)
    node.pad_map = msg


def test_nearest_confirmed_pad_is_chosen(node):
    set_map(node, make_pad(0, 10.0, 0.0), make_pad(1, 2.0, 0.0))
    assert node._next_target().id == 1


def test_visited_pads_are_never_targeted_again(node):
    """The flag that turns 'land' into 'land, then keep going'."""
    set_map(node, make_pad(0, 2.0, 0.0, visited=True), make_pad(1, 9.0, 0.0))
    assert node._next_target().id == 1


def test_unconfirmed_pads_are_not_targeted(node):
    """One sighting is noise. Flying to it would waste the attempt window."""
    set_map(node, make_pad(0, 2.0, 0.0, observations=1))
    assert node._next_target() is None


def test_blacklisted_pads_are_skipped(node):
    """A candidate the down camera refused to confirm must not be retried
    forever — otherwise a blue tarp deadlocks the mission."""
    set_map(node, make_pad(0, 2.0, 0.0), make_pad(1, 9.0, 0.0))
    node.blacklist.add(0)
    assert node._next_target().id == 1


def test_all_pads_done_means_no_target(node):
    set_map(node, make_pad(0, 2.0, 0.0, visited=True),
            make_pad(1, 9.0, 0.0, visited=True))
    assert node._next_target() is None


def test_empty_map_means_no_target(node):
    set_map(node)
    assert node._next_target() is None
    node.pad_map = None
    assert node._next_target() is None


# ── Down-camera confirmation gate ────────────────────────────────────────────

def _down_look(node, x, y, confidence=0.9, age_s=0.0):
    det = PadDetection()
    det.camera = "down"
    det.position_valid = True
    det.confidence = confidence
    det.position.x = float(x)
    det.position.y = float(y)
    node._last_down = det
    node._last_down_t = node._now() - age_s


def test_fresh_confident_look_confirms(node):
    node.target_xy = (4.0, -2.0)
    _down_look(node, 4.05, -1.95)
    assert node._down_confirms()


def test_stale_look_does_not_confirm(node):
    """Descending on a detection from five seconds ago is descending blind."""
    node.target_xy = (4.0, -2.0)
    _down_look(node, 4.0, -2.0, age_s=5.0)
    assert not node._down_confirms()


def test_low_confidence_look_does_not_confirm(node):
    """0.7 is what an unresolved blob scores. commit_confidence exists so that
    'probably a pad, seen from far away' never becomes a landing."""
    node.target_xy = (4.0, -2.0)
    _down_look(node, 4.0, -2.0, confidence=0.70)
    assert not node._down_confirms()


def test_look_at_a_different_pad_does_not_confirm(node):
    """Two bases in the arena: seeing the OTHER one must not green-light a
    descent onto this one's coordinates."""
    node.target_xy = (4.0, -2.0)
    _down_look(node, 9.0, -2.0)
    assert not node._down_confirms()


def test_no_look_at_all_does_not_confirm(node):
    node.target_xy = (4.0, -2.0)
    node._last_down = None
    assert not node._down_confirms()


# ── Travelling to a candidate vs. hovering over it ───────────────────────────

def _enter_inspect(node, pad_x, pad_y, timeout=5.0, travel=60.0):
    node.inspect_timeout = timeout
    node.travel_timeout = travel
    node.target_id = 0
    node.target_xy = (pad_x, pad_y)
    node.target_height = 0.0
    set_map(node, make_pad(0, pad_x, pad_y))
    node._enter(PadMissionNode.INSPECT)


def test_a_distant_candidate_is_not_blacklisted_while_still_flying_to_it(node):
    """The trap these are two separate clocks for.

    inspect_timeout is patience for 'hovering over it and the down camera still
    will not confirm'. These are wall-clock budgets and BiguaSim runs well below
    real time, so a flight across the arena would blow a merged budget and
    blacklist a perfectly good pad before ever looking at it.
    """
    _enter_inspect(node, 30.0, 0.0, timeout=0.0)   # drone is at the origin
    for _ in range(30):
        node._tick()
    assert node.state == PadMissionNode.INSPECT
    assert 0 not in node.blacklist


def test_a_candidate_that_will_not_confirm_from_above_is_blacklisted(node):
    """Arrived, hovering, no down-camera confirmation: this is the blue tarp."""
    _enter_inspect(node, 0.0, 0.0, timeout=0.0)    # drone is already on top
    node._tick()
    assert node.state == PadMissionNode.SEARCH
    assert 0 in node.blacklist


def test_an_unreachable_candidate_eventually_gives_up(node):
    """If the vehicle never gets there at all, the mission must still move on."""
    _enter_inspect(node, 30.0, 0.0, travel=0.0)
    node._tick()
    assert node.state == PadMissionNode.SEARCH
    assert 0 in node.blacklist


def test_arrival_does_not_leak_between_candidates(node):
    """Time spent over pad A must not count against pad B the instant it is
    chosen — that would blacklist the second pad on sight."""
    _enter_inspect(node, 0.0, 0.0, timeout=100.0)
    node._tick()
    assert node._arrived_since is not None
    node._enter(PadMissionNode.SEARCH)
    assert node._arrived_since is None


# ── Relaunch after a landing ─────────────────────────────────────────────────

def _set_fcu(node, mode, armed, z=0.0):
    node.mav_state.connected = True
    node.mav_state.mode = mode
    node.mav_state.armed = armed
    node.pose.pose.position.z = float(z)


def test_relaunch_does_not_try_to_take_off_while_still_in_land(node):
    """The trap on the way back up.

    ArduCopter auto-disarms only after DISARM_DELAY (10 s by default) and the
    dwell on the pad is shorter, so after a landing the vehicle is usually still
    ARMED and still in LAND. Treating 'armed' as ready-to-fly would send a
    takeoff in LAND mode, which ArduPilot refuses — forever.
    """
    node.state = PadMissionNode.ARMING
    _set_fcu(node, mode="LAND", armed=True)
    node._tick()
    assert node.state == PadMissionNode.ARMING, "left ARMING while still in LAND"


def test_relaunch_proceeds_once_guided_is_confirmed(node):
    """Still armed from the landing, now in GUIDED: skip arming, go take off."""
    node.state = PadMissionNode.ARMING
    _set_fcu(node, mode="GUIDED", armed=True)
    node._tick()
    assert node.state == PadMissionNode.TAKEOFF


def test_cold_start_waits_for_guided_before_arming(node):
    node.state = PadMissionNode.ARMING
    _set_fcu(node, mode="STABILIZE", armed=False)
    node._tick()
    assert node.state == PadMissionNode.ARMING


# ── The landing quota ────────────────────────────────────────────────────────

def test_max_pads_stops_the_search_even_with_a_pad_in_view(node):
    """Once the quota is met, a pad still sitting in the map must not pull the
    drone back out — the quota is what the operator asked for."""
    node.max_pads = 1
    node.landed_count = 1
    node.home = (0.0, 0.0)
    node.route = [(3.0, 0.0)]
    node.state = PadMissionNode.SEARCH
    set_map(node, make_pad(0, 5.0, 0.0))
    node._tick()
    assert node.state == PadMissionNode.RETURN_HOME


def test_max_pads_stops_a_divert_on_the_way_home(node):
    # Home is far off so the leg is still in progress; without the quota the
    # pad at 5 m would be a divert.
    node.max_pads = 1
    node.landed_count = 1
    node.home = (20.0, 0.0)
    node.state = PadMissionNode.RETURN_HOME
    set_map(node, make_pad(0, 5.0, 0.0))
    node._tick()
    assert node.state == PadMissionNode.RETURN_HOME


def test_a_late_pad_is_still_taken_on_the_way_home_without_a_quota(node):
    """The other side of it: with no quota, a pad found late is worth the detour."""
    node.max_pads = 0
    node.landed_count = 1
    node.home = (20.0, 0.0)
    node.state = PadMissionNode.RETURN_HOME
    set_map(node, make_pad(0, 5.0, 0.0))
    node._tick()
    assert node.state == PadMissionNode.INSPECT


def test_zero_max_pads_means_no_quota(node):
    """0 is 'keep going until the pattern is exhausted', not 'stop immediately'."""
    node.max_pads = 0
    node.landed_count = 3
    node.home = (0.0, 0.0)
    node.route = [(3.0, 0.0)]
    node.route_idx = 0
    node.state = PadMissionNode.SEARCH
    set_map(node, make_pad(0, 5.0, 0.0))
    node._tick()
    assert node.state == PadMissionNode.INSPECT


# ── Startup behaviour ────────────────────────────────────────────────────────

def test_mission_holds_until_the_fcu_is_ready(node):
    """No MAVROS link means no arming attempt, and certainly no takeoff."""
    node.auto_start = True
    node.pose = None
    for _ in range(20):
        node._tick()
    assert node.state == PadMissionNode.WAIT_FCU
    assert not node.stream_setpoint


def test_setpoints_are_not_streamed_before_takeoff(node):
    """A setpoint stream while landed is how a vehicle gets pushed around on the
    ground. It starts only once the drone is up."""
    assert not node.stream_setpoint
    node._stream()      # must be a no-op, not a crash
