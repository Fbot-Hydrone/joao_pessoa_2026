#!/usr/bin/env python3
"""Unit tests for GripperDynamixelNode dynamic torque ramp, movement analysis, and calibration."""

import pytest
import rclpy
from std_srvs.srv import Trigger

from hydrone_bringup.gripper_dynamixel_node import (
    ADDR_GOAL_POSITION,
    ADDR_MOVING_SPEED,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    ADDR_TORQUE_LIMIT,
    ERR_OVERLOAD,
    GripperDynamixelNode,
)


def make_telemetry_bytes(pos, speed=0, load=0, voltage=121, temp=33, moving=0):
    return [
        pos & 0xFF, (pos >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF,
        load & 0xFF, (load >> 8) & 0xFF,
        voltage & 0xFF,
        temp & 0xFF,
        0, 0,
        1 if moving else 0,
    ]


class FakePort:
    def clearPort(self):
        pass

    def closePort(self):
        pass


class FakeServoPkt:
    """Physics-lite AX-12A emulator with stops, sticky spots, soft/rigid objects, and overload."""

    def __init__(
        self,
        start_pos=430,
        open_stop=110,
        closed_stop=610,
        object_pos=None,
        overload_above_squeeze=None,
        running_load=20,
        sticky_spot=None,          # (pos, break_torque) — stalls until torque >= break_torque
        soft_yield_until=None,     # (pos, yield_torque, solid_pos) — slips until solid_pos
    ):
        self.pos = start_pos
        self.open_stop = open_stop
        self.closed_stop = closed_stop
        self.object_pos = object_pos
        self.overload_above_squeeze = overload_above_squeeze
        self.running_load = running_load
        self.sticky_spot = sticky_spot
        self.soft_yield_until = soft_yield_until
        self.goal = start_pos
        self.torque_limit = 550
        self.torque_enable = 1
        self.speed = 200
        self.max_torque_commanded = 0
        self.writes = []

    def write1ByteTxRx(self, _port, _id, addr, val):
        self.writes.append((addr, val))
        if addr == ADDR_TORQUE_ENABLE:
            self.torque_enable = val
        return (0, 0)

    def write2ByteTxRx(self, _port, _id, addr, val):
        self.writes.append((addr, val))
        if addr == ADDR_GOAL_POSITION:
            self.goal = val
        elif addr == ADDR_TORQUE_LIMIT:
            self.torque_limit = val
            self.max_torque_commanded = max(self.max_torque_commanded, val)
        elif addr == ADDR_MOVING_SPEED:
            self.speed = val
        return (0, 0)

    def readTxRx(self, _port, _id, addr, nbytes):
        assert addr == ADDR_PRESENT_POSITION and nbytes == 11
        step = 120
        err = 0
        load = self.running_load
        moving = 0

        if self.torque_enable and self.torque_limit > 0:
            diff = self.goal - self.pos
            if diff > 0:
                limit = self.closed_stop
                # Optional sticky spot (3D-print drag) that breaks free at higher torque
                if (self.sticky_spot is not None
                        and self.pos <= self.sticky_spot[0]
                        and self.torque_limit < self.sticky_spot[1]):
                    limit = min(limit, self.sticky_spot[0])
                # Optional soft/unseated object that shifts when torque increases
                elif self.soft_yield_until is not None:
                    first_pos, yield_torque, solid_pos = self.soft_yield_until
                    if self.torque_limit < yield_torque and self.pos <= first_pos:
                        limit = min(limit, first_pos)
                    else:
                        limit = min(limit, solid_pos)
                elif self.object_pos is not None and self.pos <= self.object_pos:
                    limit = min(limit, self.object_pos)

                nxt = min(self.pos + min(diff, step), limit)
                if nxt == limit and self.goal > limit:
                    squeeze = self.goal - limit
                    if (self.overload_above_squeeze is not None
                            and squeeze > self.overload_above_squeeze):
                        load = 750
                        err = ERR_OVERLOAD
                    else:
                        # Against a solid obstacle, load scales up with applied torque
                        load = min(self.torque_limit, max(240, int(self.torque_limit * 0.92)))
                    moving = 1 if squeeze > 2 else 0
                else:
                    # Still advancing (e.g. broke free from sticky spot or object shifting)
                    moving = 1 if nxt != self.goal else 0
                self.pos = nxt
            elif diff < 0:
                limit = self.open_stop
                nxt = max(self.pos - min(-diff, step), limit)
                if nxt == limit and self.goal < limit:
                    squeeze = limit - self.goal
                    load = min(self.torque_limit, max(260, squeeze * 25))
                    moving = 1 if squeeze > 2 else 0
                else:
                    moving = 1 if nxt != self.goal else 0
                self.pos = nxt

        return (make_telemetry_bytes(self.pos, speed=0, load=load, moving=moving), 0, err)


class FakeGoalHandle:
    def __init__(self, close: bool):
        class Req:
            pass
        self.request = Req()
        self.request.close = close
        self.feedback = []
        self.succeeded = False
        self.aborted = False

    def publish_feedback(self, fb):
        self.feedback.append(fb.position)

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True


@pytest.fixture(scope="module", autouse=True)
def ros_lifecycle():
    rclpy.init()
    yield
    rclpy.shutdown()


def _build_node(pkt, monkeypatch, overrides):
    monkeypatch.setattr("hydrone_bringup.gripper_dynamixel_node.time.sleep", lambda _s: None)
    clock = {"t": 0.0}

    def fake_mono():
        clock["t"] += 0.15
        return clock["t"]

    monkeypatch.setattr("hydrone_bringup.gripper_dynamixel_node.time.monotonic", fake_mono)

    orig_init = rclpy.node.Node.__init__

    def patched_init(self, name, **kw):
        kw["parameter_overrides"] = [
            rclpy.parameter.Parameter(k, value=v) for k, v in overrides.items()
        ]
        orig_init(self, name, **kw)

    monkeypatch.setattr(rclpy.node.Node, "__init__", patched_init)
    return GripperDynamixelNode(port_h=FakePort(), pkt=pkt)


def test_auto_calibrate_discovers_limits_from_arbitrary_start(monkeypatch):
    pkt = FakeServoPkt(start_pos=430, open_stop=110, closed_stop=610)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": True,
        "open_position": 264,
        "closed_position": 755,
    })
    try:
        assert node.open_pos == 120
        assert node.closed_pos == 600
        assert abs(pkt.pos - 120) <= 20
    finally:
        node.destroy_node()


def test_auto_calibrate_falls_back_when_obstructed(monkeypatch):
    pkt = FakeServoPkt(start_pos=300, open_stop=290, closed_stop=320)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": True,
        "open_position": 264,
        "closed_position": 755,
    })
    try:
        assert node.open_pos == 264
        assert node.closed_pos == 755
    finally:
        node.destroy_node()


def test_calibrate_ignores_mid_travel_linkage_friction(monkeypatch):
    pkt = FakeServoPkt(start_pos=200, open_stop=110, closed_stop=610, running_load=310)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": True,
        "open_position": 264,
        "closed_position": 755,
    })
    try:
        assert node.open_pos == 120
        assert node.closed_pos == 600
    finally:
        node.destroy_node()


def test_sticky_spot_breaks_free_on_torque_ramp_and_reports_empty(monkeypatch):
    # Sticky 3D-print drag stops jaws at pos=320 until torque >= 400.
    # Dynamic ramp increases torque 350 -> 430; jaws advance without torque rise
    # -> recognized as NOT grabbed, continues to closed_pos=600 -> EMPTY!
    pkt = FakeServoPkt(
        start_pos=120, open_stop=110, closed_stop=610,
        object_pos=None, sticky_spot=(320, 400), running_load=180,
    )
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
        "open_position": 120,
        "closed_position": 600,
    })
    try:
        grab_res = node._srv_grab(Trigger.Request(), Trigger.Response())
        assert grab_res.success is False
        assert "EMPTY" in grab_res.message
        assert abs(pkt.pos - 600) <= 20
        assert pkt.max_torque_commanded <= node.hold_torque_limit
    finally:
        node.destroy_node()


def test_soft_object_seats_on_torque_ramp_and_respects_3d_print_cap(monkeypatch):
    # Object lightly touched at 360; raising torque to 430 shifts/seats it to 405
    # (moved=45 > firm_move_tol without much load rise). Next step (430->510) holds
    # steady at 405 with load rising cleanly -> FIRM GRIP confirmed within 3D-print cap!
    pkt = FakeServoPkt(
        start_pos=120, open_stop=110, closed_stop=610,
        soft_yield_until=(360, 400, 405), running_load=150,
    )
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
        "open_position": 120,
        "closed_position": 600,
        "close_max_torque": 680,
        "hold_torque_limit": 700,
    })
    try:
        grab_res = node._srv_grab(Trigger.Request(), Trigger.Response())
        assert grab_res.success is True
        assert "HOLDING" in grab_res.message
        assert pkt.pos == 405
        assert pkt.max_torque_commanded <= 700
    finally:
        node.destroy_node()


def test_close_detects_object_by_torque_and_relieves_overload(monkeypatch):
    pkt = FakeServoPkt(start_pos=120, open_stop=110, closed_stop=610,
                       object_pos=400, overload_above_squeeze=15)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
        "open_position": 120,
        "closed_position": 600,
        "hold_squeeze": 20,
        "relief_step": 8,
        "hold_torque_limit": 700,
    })
    try:
        gh = FakeGoalHandle(close=True)
        res = node._exec(gh)
        assert gh.succeeded and res.success
        assert res.holding is True
        # Started at 400 + 20 = 420 (overloaded), backed off by 8 to 412 (safe!)
        assert pkt.goal == 412
        assert pkt.max_torque_commanded <= 700
        assert "OVERLOAD" not in res.message
    finally:
        node.destroy_node()


def test_grab_and_release_trigger_services(monkeypatch):
    pkt = FakeServoPkt(start_pos=120, open_stop=110, closed_stop=610, object_pos=380)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
        "open_position": 120,
        "closed_position": 600,
    })
    try:
        grab_res = node._srv_grab(Trigger.Request(), Trigger.Response())
        assert grab_res.success is True
        assert "HOLDING" in grab_res.message
        assert pkt.torque_limit <= 780

        rel_res = node._srv_release(Trigger.Request(), Trigger.Response())
        assert rel_res.success is True
        assert abs(pkt.pos - 120) <= 20
    finally:
        node.destroy_node()


def test_gripper_yaml_loads_and_matches_declared_params(monkeypatch):
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory

    yaml_path = os.path.join(
        get_package_share_directory("hydrone_bringup"), "config", "gripper.yaml")
    with open(yaml_path, "r") as f:
        doc = yaml.safe_load(f)
    params = doc["/**"]["ros__parameters"]
    assert params["hold_torque_limit"] == 780
    assert params["max_hold_load"] == 730

    pkt = FakeServoPkt(start_pos=120, open_stop=110, closed_stop=610)
    node = _build_node(pkt, monkeypatch, {
        **params,
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
    })
    try:
        for k in params:
            assert node.has_parameter(k), f"gripper.yaml key {k!r} not declared by node"
    finally:
        node.destroy_node()


def test_open_stops_on_shifted_stop_and_backs_off(monkeypatch):
    pkt = FakeServoPkt(start_pos=500, open_stop=170, closed_stop=610)
    node = _build_node(pkt, monkeypatch, {
        "boot_wait_s": 0.0,
        "auto_calibrate": False,
        "open_position": 120,
        "closed_position": 600,
        "open_backoff": 10,
    })
    try:
        gh = FakeGoalHandle(close=False)
        res = node._exec(gh)
        assert gh.succeeded and res.success
        assert pkt.goal == 180
    finally:
        node.destroy_node()
