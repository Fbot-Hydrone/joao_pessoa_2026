#!/usr/bin/env python3
"""
End-to-end test of the landing-pad pipeline, with the real nodes wired together.

    fake BiguaSim camera  ->  down_cam_mimic_node  ->  pad_detector_node
                                                             |
                                    pad_map_node  <----------+
                                          |
                                    /hydrone/pads/map

Everything runs in one process on a MultiThreadedExecutor. What it proves that
the unit tests cannot: the topic names line up, the QoS profiles are compatible
(a RELIABLE subscriber on a BEST_EFFORT sensor topic receives nothing, and
nothing warns you), the static TF the mimic publishes is the one the detector
looks up, and the fused map position matches where the pad actually is.

The only fake here is the image: a synthetic pad rendered into a synthetic
floor, plus a synthetic vehicle pose. Every node in the chain is the real one.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_nav/test/test_pad_pipeline.py -q'
"""

import os
import sys
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Range

from mavros_msgs.msg import State

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..",
                                "hydrone_vision", "test"))

from hydrone_bringup.down_cam_mimic_node import DownCamMimicNode  # noqa: E402
from hydrone_msgs.msg import PadMap  # noqa: E402
from hydrone_msgs.srv import MarkPadVisited  # noqa: E402
from hydrone_nav.pad_map_node import PadMapNode  # noqa: E402
from hydrone_vision.pad_detector_node import PadDetectorNode  # noqa: E402

from test_pad_detector import ground, paste_pad, render_pad, square_quad  # noqa: E402


IN_IMAGE = "/biguasim/test_id0/DownCamera"
IN_INFO = "/biguasim/test_id0/DownCamera/camera_info"

WIDTH, HEIGHT = 640, 480
FX = FY = 320.0          # 90 deg horizontal FOV, as the sim publishes

DRONE_XY = (4.0, -2.0)
DRONE_Z = 3.12           # camera 3.00 m above the floor (mount is -0.12)


class FakeSim(Node):
    """Stands in for BiguaSim's bridge and MAVROS: a camera and a vehicle pose."""

    def __init__(self, pad_offset_px=(0, 0)):
        super().__init__("fake_sim")
        self.pad_offset_px = pad_offset_px

        self.pub_img = self.create_publisher(Image, IN_IMAGE, 10)
        self.pub_info = self.create_publisher(CameraInfo, IN_INFO, 10)
        # MAVROS publishes these BEST_EFFORT; matching it here is part of what
        # the test is checking.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_pose = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", sensor_qos)
        # Where MAVROS publishes the rangefinder on the drone, and where
        # rangefinder_bridge mimics it in sim — pad_map_node's one default.
        self.pub_range = self.create_publisher(
            Range, "/mavros/distance_sensor/rangefinder", sensor_qos)
        # pad_map_node maps nothing until the vehicle has armed (require_armed,
        # see test_takeoff_base.py). A stack that never arms therefore maps
        # nothing at all, so standing in for MAVROS means publishing this too —
        # the drone in this test is notionally airborne over the pad.
        self.pub_state = self.create_publisher(State, "/mavros/state", 10)

        du, dv = pad_offset_px
        scene = paste_pad(ground(WIDTH, HEIGHT), render_pad(),
                          square_quad(WIDTH // 2 + du, HEIGHT // 2 + dv, 70))
        self.frame = scene
        # Overridable so a test can put the surface somewhere other than the
        # floor (that is how the elevated base is simulated).
        self.range_m = DRONE_Z
        self.create_timer(0.05, self._publish)

    def _publish(self):
        now = self.get_clock().now().to_msg()

        info = CameraInfo()
        info.header.stamp = now
        info.width, info.height = WIDTH, HEIGHT
        info.k = [FX, 0.0, WIDTH / 2.0, 0.0, FY, HEIGHT / 2.0, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, WIDTH / 2.0, 0.0,
                  0.0, FY, HEIGHT / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

        img = Image()
        img.header.stamp = now
        img.height, img.width = HEIGHT, WIDTH
        img.encoding = "bgr8"
        img.step = WIDTH * 3
        img.data = self.frame.tobytes()
        self.pub_img.publish(img)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y = DRONE_XY
        pose.pose.position.z = DRONE_Z
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        rng = Range()
        rng.header.stamp = now
        rng.min_range, rng.max_range = 0.05, 40.0
        rng.range = float(self.range_m)
        self.pub_range.publish(rng)

        state = State()
        state.header.stamp = now
        state.connected = True
        state.armed = True
        state.mode = "GUIDED"
        self.pub_state.publish(state)


class MapSink(Node):
    """Keeps the newest /hydrone/pads/map."""

    def __init__(self):
        super().__init__("map_sink")
        self.last: PadMap | None = None
        self.create_subscription(PadMap, "/hydrone/pads/map",
                                 self._cb, 10)
        self.client = self.create_client(MarkPadVisited,
                                         "/hydrone/pads/mark_visited")

    def _cb(self, msg: PadMap):
        self.last = msg


def build_stack(pad_offset_px=(0, 0)):
    param = rclpy.parameter.Parameter
    mimic = DownCamMimicNode(parameter_overrides=[
        param("in_image", value=IN_IMAGE),
        param("in_info", value=IN_INFO),
    ])
    detector = PadDetectorNode(parameter_overrides=[
        param("camera", value="down"),
        param("image_topic", value="/down_cam/image_raw"),
        param("camera_info_topic", value="/down_cam/camera_info"),
        param("depth_topic", value=""),
        param("optical_frame", value="down_cam_optical_frame"),
        param("publish_debug", value=False),
    ])
    pad_map = PadMapNode(parameter_overrides=[
        param("min_observations", value=2),
        param("publish_hz", value=10.0),
    ])
    sim = FakeSim(pad_offset_px)
    sink = MapSink()
    return [sim, mimic, detector, pad_map, sink]


def spin_for(nodes, seconds: float):
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_pad_flows_from_camera_to_map_at_the_right_place():
    """A pad centred in the down camera maps to the point below the drone."""
    nodes = build_stack()
    sink = nodes[-1]
    try:
        spin_for(nodes, 4.0)

        assert sink.last is not None, "pad_map never published"
        assert len(sink.last.pads) == 1, (
            f"expected exactly one pad, got {len(sink.last.pads)}")

        pad = sink.last.pads[0]
        assert pad.observations >= 2
        assert pad.confidence > 0.75, "a pad this close should be verified"
        assert pad.position.x == pytest.approx(DRONE_XY[0], abs=0.15)
        assert pad.position.y == pytest.approx(DRONE_XY[1], abs=0.15)
        assert abs(pad.position.z) < 0.15
        assert sink.last.header.frame_id == "map"
    finally:
        for node in nodes:
            node.destroy_node()


def test_offset_pad_maps_to_the_correct_side_of_the_drone():
    """Shift the pad right and down in the image; it must map right and BEHIND.

    A sign error anywhere in mimic -> TF -> detector -> map shows up here as a
    pad mirrored across the drone, which is exactly the bug that would make the
    approach diverge instead of converge.
    """
    du, dv = 96, 64          # tan = 0.3 right, 0.2 down the image
    nodes = build_stack(pad_offset_px=(du, dv))
    sink = nodes[-1]
    try:
        spin_for(nodes, 4.0)
        assert sink.last and len(sink.last.pads) == 1
        pad = sink.last.pads[0]

        alt = DRONE_Z - 0.12   # camera height above the floor
        # Image right -> drone right -> -Y at yaw 0. Image down -> behind -> -X.
        expect_x = DRONE_XY[0] - alt * (dv / FY)
        expect_y = DRONE_XY[1] - alt * (du / FX)
        assert pad.position.x == pytest.approx(expect_x, abs=0.2)
        assert pad.position.y == pytest.approx(expect_y, abs=0.2)
    finally:
        for node in nodes:
            node.destroy_node()


def _mark_visited(nodes, sink, pad_id, height=None):
    request = MarkPadVisited.Request()
    request.id = int(pad_id)
    request.height_valid = height is not None
    request.height = float(height or 0.0)
    assert sink.client.wait_for_service(timeout_sec=2.0)
    future = sink.client.call_async(request)
    spin_for(nodes, 2.0)
    assert future.done() and future.result().success


def test_marking_a_pad_visited_sticks():
    """The flag the whole take-off-again loop depends on.

    If this ever regressed, the drone would land on the same pad forever: it
    re-detects the thing it is standing on the moment it lifts off.
    """
    nodes = build_stack()
    sink = nodes[-1]
    try:
        spin_for(nodes, 3.0)
        assert sink.last and sink.last.pads
        pad_id = sink.last.pads[0].id
        assert not sink.last.pads[0].visited

        _mark_visited(nodes, sink, pad_id)

        pad = next(p for p in sink.last.pads if p.id == pad_id)
        assert pad.visited
    finally:
        for node in nodes:
            node.destroy_node()


def test_rangefinder_discovers_an_elevated_pad():
    """The arena's second base sits ~0.5 m up.

    Detections project onto the assumed floor, so the map starts it at z=0. The
    downward rangefinder reading taken while hovering over it is what corrects
    that — without this the drone would try to land 0.5 m below the surface.
    """
    nodes = build_stack()
    sim, sink = nodes[0], nodes[-1]
    sim.range_m = DRONE_Z - 0.5      # the surface below is 0.5 m up
    try:
        spin_for(nodes, 4.0)
        assert sink.last and sink.last.pads
        pad = sink.last.pads[0]
        assert pad.height_measured
        assert pad.height == pytest.approx(0.5, abs=0.05)
        assert pad.position.z == pytest.approx(0.5, abs=0.05)
    finally:
        for node in nodes:
            node.destroy_node()


def test_touchdown_height_is_not_overwritten_by_later_flyovers():
    """Standing on a pad measures its top exactly; a fly-over is a guess.

    Once visited, the height recorded at touchdown must survive the rangefinder
    continuing to report the floor beside it.
    """
    nodes = build_stack()
    sim, sink = nodes[0], nodes[-1]
    try:
        spin_for(nodes, 3.0)
        assert sink.last and sink.last.pads
        pad_id = sink.last.pads[0].id

        _mark_visited(nodes, sink, pad_id, height=0.5)
        # The rangefinder keeps insisting the surface is at floor level.
        assert sim.range_m == pytest.approx(DRONE_Z)
        spin_for(nodes, 2.0)

        pad = next(p for p in sink.last.pads if p.id == pad_id)
        assert pad.visited
        assert pad.height_measured
        assert pad.height == pytest.approx(0.5, abs=1e-6)
    finally:
        for node in nodes:
            node.destroy_node()


def test_empty_floor_produces_an_empty_map():
    """No pad in view must mean no entries — not a low-confidence guess."""
    nodes = build_stack()
    sim = nodes[0]
    sim.frame = ground(WIDTH, HEIGHT)
    sink = nodes[-1]
    try:
        spin_for(nodes, 3.0)
        assert sink.last is not None
        assert sink.last.pads == []
    finally:
        for node in nodes:
            node.destroy_node()
