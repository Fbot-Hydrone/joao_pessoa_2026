#!/usr/bin/env python3
"""Phase2MissionNode gripper sequencing (from the Orin, 2026-11-24), on the\ncbr/phase2 node: open before arming, one close, re-open before a retry."""

from types import SimpleNamespace

import pytest
import rclpy

from hydrone_mission.phase2_mission_node import Phase2MissionNode


class FakeJob:
    def __init__(self, ok=True, holding=True, message="ok", done=False):
        self.done = done
        self.ok = ok
        self.result = SimpleNamespace(success=ok, holding=holding, z=0.0, message=message)
        self.message = message


class FakeGripper:
    def __init__(self):
        self.calls = []
        self.next_job = None

    def ready(self):
        return True

    def open(self):
        self.calls.append("open")
        j = self.next_job if self.next_job is not None else FakeJob(ok=True, holding=False, done=False)
        self.next_job = None
        return j

    def close(self):
        self.calls.append("close")
        j = self.next_job if self.next_job is not None else FakeJob(ok=True, holding=True, done=False)
        self.next_job = None
        return j


class FakeVehicle:
    def __init__(self):
        self.holds = 0

    def ready(self):
        return True

    def setpoint(self, *_args):
        self.holds += 1


class FakeNav:
    def server_is_ready(self):
        return True


def _make_pose():
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=1.5),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )


@pytest.fixture(scope="module", autouse=True)
def ros_lifecycle():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_mission():
    node = Phase2MissionNode()
    node.gripper = FakeGripper()
    node.vehicle = FakeVehicle()
    node.nav = FakeNav()
    node.pose = _make_pose()
    node.home = (0.0, 0.0)
    node.home_yaw = node.hold_yaw = 0.0
    node.pick_xy = [(1.0, 1.0), (2.0, 1.0)]
    node.deliv_xy = [(1.0, 3.0), (2.0, 3.0)]
    node.tour = [(0, 0), (1, 1)]
    node.leg = 0
    return node


def test_startup_waits_for_gripper_open_before_arming():
    node = make_mission()
    try:
        node.state = node.WAIT
        node.auto_start = False
        node._do_wait()
        assert node.state == node.WAIT           # the arena countdown first
        node.auto_start = True
        node._do_wait()
        assert node.state == node.OPENING
        assert node.gripper.calls == []

        # First OPENING tick fires gripper.open() and stays in OPENING while incomplete
        node._do_opening()
        assert node.gripper.calls == ["open"]
        assert node.state == node.OPENING

        # Still waiting while job is in flight
        node._do_opening()
        assert node.state == node.OPENING

        # Completing open transitions to ARMING
        node._job.done = True
        node._do_opening()
        assert node.state == node.ARMING
    finally:
        node.destroy_node()


def _touch_then_act(node):
    node.target = ("pickup", 1.0, 1.0)
    node._touch_xy = (1.0, 1.0)
    node.range_m = 0.20
    node.state = node.TOUCH
    node._do_touch()


def test_the_gripper_closes_once_on_the_ground_by_default():
    """Closing at touch_range leaves the real fingers over the kit's top."""
    node = make_mission()
    try:
        _touch_then_act(node)
        assert node.gripper.calls == []          # no close in the air
        assert node.state == node.LAND

        node._enter(node.ACT)
        node._do_act()
        assert node.gripper.calls == ["close"]
        node._job.done = True
        node._do_act()
        assert node.gripper.calls == ["close"]
        assert node.carrying is True
        assert node.state == node.ARMING
    finally:
        node.destroy_node()


def test_touch_grab_not_closed_twice_in_act():
    node = make_mission()
    try:
        node.close_in_touch = True
        _touch_then_act(node)
        assert node.gripper.calls == ["close"]
        node._job.done = True
        node._do_touch()
        assert node.state == node.LAND
        assert node._touch_grab == (True, True, "ok")

        # ACT consumes the TOUCH grab without calling gripper.close() a second time
        node._enter(node.ACT)
        node._do_act()
        assert node.gripper.calls == ["close"]   # NOT ["close", "close"]!
        assert node.carrying is True
        assert node.state == node.ARMING
    finally:
        node.destroy_node()


def test_grab_failed_reopens_gripper_before_retry_and_skip():
    node = make_mission()
    try:
        node.pick_retries = 1
        node.retries = 0
        node._grab_failed()
        assert node.state == node.OPENING
        assert node.retries == 1

        # Tick OPENING while airborne -> streams hold setpoint & opens gripper
        node._do_opening()
        assert node.gripper.calls == ["open"]
        assert node.vehicle.holds >= 1
        node._job.done = True
        node._do_opening()
        assert node.state == node.GOTO

        # Exhaust retries -> also opens gripper before skipping to next kit
        node._grab_failed()
        assert node.state == node.OPENING
        assert node.leg == 1
        node._do_opening()
        assert node.gripper.calls == ["open", "open"]
        node._job.done = True
        node._do_opening()
        assert node.state == node.GOTO
    finally:
        node.destroy_node()
