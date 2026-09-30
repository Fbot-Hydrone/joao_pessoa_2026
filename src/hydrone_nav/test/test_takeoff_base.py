#!/usr/bin/env python3
"""
Tests for the two things pad_map_node learned for the Phase 1 mission: it maps
nothing before the vehicle first arms, and the base the drone starts on is
DECLARED rather than detected.

Both exist to keep one thing from happening: the start base becoming an ordinary
map candidate. The drone always begins standing on a base, and on the ground the
cameras are looking at it from the grazing angles that detect it best. If it
reaches the map as a candidate, every run pays a travel leg and a confirmation
hover to rule out something that was known before the propellers turned — and in
this arena every second of flight is visual-odometry drift.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_nav/test/test_takeoff_base.py -q'
"""

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range

from mavros_msgs.msg import State

from hydrone_msgs.msg import PadDetection
from hydrone_msgs.srv import MarkPadVisited, RegisterTakeoffBase
from hydrone_nav.pad_map_node import PadMapNode


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = PadMapNode(parameter_overrides=[
        rclpy.parameter.Parameter("require_armed", value=True),
        rclpy.parameter.Parameter("min_observations", value=3),
        rclpy.parameter.Parameter("provisional_ttl_s", value=20.0),
        rclpy.parameter.Parameter("merge_radius", value=1.2),
        rclpy.parameter.Parameter("takeoff_base_radius", value=1.5),
    ])
    yield n
    n.destroy_node()


# ── Fakes ────────────────────────────────────────────────────────────────────

def detect(node, x, y, z=0.0, confidence=0.9, camera="down", range_m=2.0):
    det = PadDetection()
    det.header.frame_id = "map"
    det.camera = camera
    det.confidence = float(confidence)
    det.position_valid = True
    det.range_m = float(range_m)
    det.position.x = float(x)
    det.position.y = float(y)
    det.position.z = float(z)
    node._cb_detection(det)


def arm(node, armed=True):
    s = State()
    s.armed = bool(armed)
    node._cb_state(s)


def register(node, x, y, z=0.0):
    req = RegisterTakeoffBase.Request()
    req.position.x = float(x)
    req.position.y = float(y)
    req.position.z = float(z)
    return node._svc_register_takeoff_base(req, RegisterTakeoffBase.Response())


def set_pose(node, x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.w = 1.0
    node.pose = pose


def base_of(node):
    return [e for e in node.pads.values() if e.is_takeoff_base]


# ── Nothing is mapped before the first arm ───────────────────────────────────

def test_detections_before_the_first_arm_are_dropped(node):
    for _ in range(10):
        detect(node, 2.0, 0.0)
    assert node.pads == {}


def test_detections_after_the_first_arm_are_mapped(node):
    arm(node)
    detect(node, 2.0, 0.0)
    assert len(node.pads) == 1


def test_the_gate_latches_and_does_not_reclose_on_disarm(node):
    """The drone disarms on every pad it lands on. Detections must keep flowing
    then, or the map goes blind for the rest of the run."""
    arm(node)
    arm(node, armed=False)
    detect(node, 2.0, 0.0)
    assert len(node.pads) == 1


def test_the_gate_can_be_turned_off():
    n = PadMapNode(parameter_overrides=[
        rclpy.parameter.Parameter("require_armed", value=False),
    ])
    try:
        detect(n, 2.0, 0.0)
        assert len(n.pads) == 1
    finally:
        n.destroy_node()


# ── Registering the takeoff base ─────────────────────────────────────────────

def test_registering_creates_a_flagged_entry(node):
    response = register(node, 0.0, 0.0, 0.0)
    assert response.success
    entry = node.pads[response.id]
    assert entry.is_takeoff_base
    assert node.takeoff_base_id == response.id


def test_the_registered_height_is_a_measurement_not_a_projection(node):
    """The drone is standing on it. That beats anything a camera produces, and
    it must not be washed out by a later fly-over's flat-floor guess."""
    response = register(node, 0.0, 0.0, 0.35)
    entry = node.pads[response.id]
    assert entry.height == pytest.approx(0.35)
    assert entry.height_measured


def test_registering_claims_an_entry_that_is_already_there(node):
    """With the arm gate off, or on a second run, the start base may already be
    in the map. Flagging it beats creating a duplicate 20 cm away."""
    arm(node)
    detect(node, 0.2, 0.1)
    assert len(node.pads) == 1
    response = register(node, 0.0, 0.0, 0.0)
    assert len(node.pads) == 1
    assert node.pads[response.id].is_takeoff_base


def test_registering_twice_leaves_exactly_one_takeoff_base(node):
    register(node, 0.0, 0.0, 0.0)
    register(node, 5.0, 5.0, 0.0)
    assert len(base_of(node)) == 1
    assert base_of(node)[0].x == pytest.approx(5.0)


# ── The takeoff base absorbs its own sightings ───────────────────────────────

def test_a_glancing_sighting_of_the_start_base_does_not_spawn_a_phantom(node):
    """Seen from the air at an angle, the start base projects with the largest
    error there is. A second entry 1.3 m away would be indistinguishable from a
    genuine landing site sitting next to it."""
    arm(node)
    register(node, 0.0, 0.0, 0.0)
    detect(node, 1.3, 0.0)
    assert len(node.pads) == 1
    assert base_of(node)[0].observations == 2


def test_a_real_pad_just_outside_the_claim_radius_is_its_own_entry(node):
    arm(node)
    register(node, 0.0, 0.0, 0.0)
    detect(node, 2.0, 0.0)
    assert len(node.pads) == 2


def test_a_pad_closer_to_a_real_entry_is_not_stolen_by_the_wider_claim(node):
    """The takeoff base claims 1.5 m and an ordinary pad 1.2 m. Comparing raw
    distance would let the base win a detection sitting almost on top of the
    other pad."""
    arm(node)
    register(node, 0.0, 0.0, 0.0)
    detect(node, 1.6, 0.0)              # a new entry, outside the base's claim
    detect(node, 1.45, 0.0)             # 1.45 from the base, 0.15 from the pad
    assert len(node.pads) == 2
    assert base_of(node)[0].observations == 1


# ── It survives, and it is not second-guessed ────────────────────────────────

def test_the_takeoff_base_is_never_pruned(node):
    """It has one 'observation' and it is not going to earn more. Ordinary
    pruning would drop it after provisional_ttl_s and the mission would lose
    the only thing it knows for certain."""
    register(node, 0.0, 0.0, 0.0)
    entry = base_of(node)[0]
    entry.last_seen -= 1e6              # ancient
    node._prune()
    assert base_of(node)


def test_a_flyover_does_not_rewrite_the_takeoff_base_height(node):
    """Same reasoning as a visited pad: standing on it measured the surface, and
    a glancing rangefinder pass over the floor beside it must not undo that."""
    register(node, 0.0, 0.0, 0.0)
    entry = base_of(node)[0]

    set_pose(node, 0.0, 0.0, 2.0)
    r = Range()
    r.min_range, r.max_range, r.range = 0.2, 40.0, 1.4    # would imply 0.6 m
    node._cb_range(r)
    node._refine_height_from_rangefinder()

    assert entry.height == pytest.approx(0.0)


def test_the_takeoff_base_reaches_the_published_map(node):
    """The flag has to survive the trip through the Pad message — it is the
    only way the mission ever learns which entry not to fly to."""
    arm(node)
    register(node, 0.0, 0.0, 0.0)
    detect(node, 3.0, 0.0)
    flags = {p.id: p.is_takeoff_base for p in _published_map(node).pads}
    assert sorted(flags.values()) == [False, True]


def _published_map(node):
    from hydrone_msgs.msg import PadMap
    msg = PadMap()
    msg.header.frame_id = node.world_frame
    now = node.get_clock().now().to_msg()
    msg.pads = [node._to_pad_msg(e, now) for e in node.pads.values()]
    return msg


def test_the_takeoff_base_marker_is_its_own_colour(node):
    """Grey/cyan/green all mean 'a landing site in some state'. The takeoff base
    is the one entry that means 'never fly here', and it must not look like the
    thing the mission is hunting for."""
    register(node, 0.0, 0.0, 0.0)
    markers = node._to_markers(_published_map(node))
    discs = [m for m in markers.markers if m.ns == "pads"]
    assert len(discs) == 1
    assert (discs[0].color.r, discs[0].color.g) == pytest.approx((1.0, 0.55))
    labels = [m for m in markers.markers if m.ns == "pad_labels"]
    assert "takeoff-base" in labels[0].text


# ── Landing bookkeeping still works alongside it ─────────────────────────────

def test_marking_visited_still_works(node):
    arm(node)
    detect(node, 2.0, 0.0)
    pad_id = next(iter(node.pads))
    req = MarkPadVisited.Request()
    req.id = pad_id
    req.height = 0.0
    req.height_valid = True
    response = node._svc_visited(req, MarkPadVisited.Response())
    assert response.success
    assert node.pads[pad_id].visited
    assert not node.pads[pad_id].is_takeoff_base


# ── Rejections are never silent ──────────────────────────────────────────────
#
# A detection the detector was confident enough to publish, that the map then
# discards, is the one event where the two halves of the pipeline disagree — and
# from outside it looks exactly like "the detector never saw it". Every gate has
# to say so. Added 2026-08-22 after a confident forward-camera detection failed
# to appear in the map and there was nothing in any log to say which gate closed.

def detection(x=2.0, y=0.0, z=0.0, confidence=0.9, range_m=2.0,
              position_valid=True, camera="forward"):
    det = PadDetection()
    det.header.frame_id = "map"
    det.camera = camera
    det.confidence = float(confidence)
    det.position_valid = bool(position_valid)
    det.range_m = float(range_m)
    det.position.x, det.position.y, det.position.z = float(x), float(y), float(z)
    return det


@pytest.fixture
def rejections(node, monkeypatch):
    """Collect the warnings pad_map emits, so the reasons can be asserted on."""
    said = []
    monkeypatch.setattr(node.get_logger(), "warn",
                        lambda text, **kw: said.append(text))
    arm(node)
    return said


@pytest.fixture
def loud_node(node):
    """Same node with the rejection throttle off, for counting."""
    node.reject_log_period = 0.0
    return node


@pytest.mark.parametrize("det,expected", [
    (detection(position_valid=False), "position_valid is false"),
    (detection(confidence=0.10), "min_confidence"),
    (detection(range_m=999.0), "max_range_m"),
])
def test_every_gate_says_why_it_dropped_a_detection(node, rejections, det,
                                                    expected):
    node._cb_detection(det)
    assert node.pads == {}, "the detection should have been rejected"
    assert any(expected in line for line in rejections), (
        f"no log line mentioning {expected!r}; got {rejections}")


def test_an_accepted_detection_says_nothing(node, rejections):
    node._cb_detection(detection())
    assert len(node.pads) == 1
    assert rejections == []


def test_two_gates_do_not_share_one_throttle_bucket(node, rejections):
    """The trap rclpy's own throttle_duration_sec would have walked into.

    It keys its state on the CALL SITE, and every gate funnels through the same
    logging line — so a belly camera rejecting at 10 Hz would have silenced the
    forward camera's reason for 5 s at a time. Which is precisely the case this
    logging was added to diagnose.
    """
    node._cb_detection(detection(camera="down", confidence=0.10))
    node._cb_detection(detection(camera="forward", position_valid=False))
    assert any("min_confidence" in line for line in rejections)
    assert any("position_valid" in line for line in rejections)


def test_the_same_gate_on_the_same_camera_is_throttled(node, rejections):
    for _ in range(20):
        node._cb_detection(detection(camera="forward", position_valid=False))
    assert len(rejections) == 1


def test_the_throttle_can_be_turned_off(loud_node, rejections):
    for _ in range(5):
        loud_node._cb_detection(detection(camera="forward",
                                          position_valid=False))
    assert len(rejections) == 5
