"""phase3_hri_node on a fake FCU: the RC only kills, and takeoffs never loop.

No MAVROS: the State/pose callbacks are fed by hand, the clock is a number,
and every service call is recorded instead of sent.
"""
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from hydrone_mission.phase3 import core
from hydrone_mission.phase3_hri_node import Phase3HriNode


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Phase3HriNode()
    n.t = 0.0
    n.now = lambda: n.t
    n.sent = []
    for c in (n.cli_arm, n.cli_mode, n.cli_takeoff):
        c.service_is_ready = lambda: True

    class Accepted:                           # MAVROS: the FCU acked it
        name = "fake"

        def poll(self, now):
            return "ok"

    def start(client, req):
        n._last_cmd = n.t
        n._call = Accepted()
        n.sent.append(req)
    n._start = start
    fcu(n, mode="STABILIZE", armed=False)
    pose(n, 0.0)
    yield n
    n.destroy_node()


def fcu(n, mode, armed):
    n._cb_state(State(connected=True, mode=mode, armed=armed))


def pose(n, z, x=0.0):
    m = PoseStamped()
    m.pose.position.x, m.pose.position.z = x, z
    m.pose.orientation.w = 1.0
    n._cb_pose(m)


def tick(n, s=0.1, k=1):
    for _ in range(k):
        n.t += s
        n._tick()


def commands(n):
    """What the node sent, as short strings."""
    out = []
    for r in n.sent:
        if isinstance(r, SetMode.Request):
            out.append(r.custom_mode)
        elif isinstance(r, CommandBool.Request):
            out.append("ARM" if r.value else "DISARM")
        elif isinstance(r, CommandTOL.Request):
            out.append("TAKEOFF")
    return out


def to_takeoff(n):
    tick(n)                                   # WAIT_FCU -> ARMING
    tick(n, 3.0)                              # asks GUIDED
    fcu(n, "GUIDED", False)
    tick(n, 3.0)                              # asks to arm
    fcu(n, "GUIDED", True)
    tick(n)                                   # -> TAKEOFF
    assert n.state == n.TAKEOFF


def to_command(n):
    to_takeoff(n)
    pose(n, 1.5)
    tick(n)                                   # airborne -> APPROACH
    n._enter(n.COMMAND)


def test_rc_disarm_in_the_air_aborts_and_never_rearms(node):
    to_command(node)
    node.sent.clear()
    fcu(node, "GUIDED", False)                # the RC kill
    tick(node, 1.0, 120)
    assert node.state == node.ABORTED
    assert commands(node) == []
    assert not node.stream


def test_rc_mode_change_during_takeoff_aborts(node):
    to_takeoff(node)
    node.sent.clear()
    fcu(node, "LAND", True)                   # pilot flips to LAND
    tick(node, 1.0, 60)
    assert node.state == node.ABORTED
    assert "GUIDED" not in commands(node) and "ARM" not in commands(node)


def test_rc_mode_change_while_arming_aborts(node):
    tick(node)
    tick(node, 3.0)
    fcu(node, "GUIDED", False)
    tick(node)                                # GUIDED reached
    fcu(node, "LAND", False)                  # then the RC
    node.sent.clear()
    tick(node, 1.0, 60)
    assert node.state == node.ABORTED
    assert commands(node) == []


def test_guided_refused_by_the_fcu_is_retried(node):
    tick(node)
    tick(node, 3.0, 5)                        # FCU keeps STABILIZE (EKF not ready)
    assert node.state == node.ARMING
    assert commands(node).count("GUIDED") >= 2


def test_disarm_in_land_is_the_landing(node):
    to_command(node)
    node._begin_landing(final=False)
    fcu(node, "LAND", True)
    tick(node)
    pose(node, 0.0)
    fcu(node, "LAND", False)                  # touched down, auto-disarm
    tick(node)
    assert node.state == node.LANDED
    assert node.landings == 1


def test_rc_leaving_land_aborts(node):
    to_command(node)
    node._begin_landing(final=False)
    fcu(node, "LAND", True)
    tick(node)
    fcu(node, "LOITER", True)                 # pilot takes it
    node.sent.clear()
    tick(node, 1.0, 30)
    assert node.state == node.ABORTED
    assert commands(node) == []


def test_takeoff_that_never_lifts_lands_and_aborts(node):
    to_takeoff(node)
    tick(node, 1.0, 400)                      # stays on the ground, armed, GUIDED
    assert node.state == node.ABORTED
    c = commands(node)
    assert c[-1] == "LAND"
    assert c.count("TAKEOFF") == node.p["takeoff_tries"]
    assert c.count("ARM") == 1                # never re-armed


def test_takeoff_short_of_the_height_carries_on(node):
    to_takeoff(node)
    tick(node, 1.0, 5)
    node._takeoff_acked = True
    pose(node, 0.8)                           # up, but below takeoff_alt - 0.15
    node.sent.clear()
    tick(node, 1.0, 70)
    assert node.state in node.AIRBORNE
    assert "TAKEOFF" not in commands(node)


def test_takeoff_gesture_on_a_base_still_rearms(node):
    to_command(node)
    node._begin_landing(final=False)
    fcu(node, "LAND", True)
    tick(node)
    fcu(node, "LAND", False)
    tick(node)
    assert node.state == node.LANDED
    node._execute(core.TAKEOFF)               # the operator asks: commanded
    node.sent.clear()
    tick(node, 3.0)
    assert node.state == node.ARMING
    assert commands(node) == ["GUIDED"]
