"""fcu — everything that touches MAVROS, in one place.

The controller node is the only thing that should import this. Missions talk to
the controller through actions (see vehicle.py), so no phase ever needs to know
about MAVLink, GUIDED, or why a mode change can be acked and still not happen.
"""

import math

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def yaw_of(pose: PoseStamped) -> float:
    """Yaw of a pose, ENU, CCW-positive from east."""
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Call:
    """One in-flight MAVROS service call with its own deadline.

    MAVROS replies use either `success` (CommandBool/CommandTOL) or `mode_sent`
    (SetMode). A reply of success is NOT proof the vehicle did it — callers
    read /mavros/state for that.
    """

    PENDING, OK, FAILED, TIMEOUT = "pending", "ok", "failed", "timeout"

    def __init__(self, node, tag, client, request, timeout_s):
        self.tag = tag
        self.name = client.srv_name
        self.future = client.call_async(request)
        self._node = node
        self.deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)

    def poll(self) -> str:
        if self.future.done():
            r = self.future.result()
            if r is None:
                return self.FAILED
            ok = getattr(r, "success", None)
            if ok is None:
                ok = getattr(r, "mode_sent", False)
            return self.OK if ok else self.FAILED
        if self._node.get_clock().now().nanoseconds > self.deadline:
            self.future.cancel()
            return self.TIMEOUT
        return self.PENDING


class FcuLink:
    """MAVROS state, commands and the position-setpoint stream."""

    def __init__(self, node, *, setpoint_hz=10.0, service_timeout_s=30.0,
                 frame_id="map"):
        self.node = node
        self.svc_timeout = service_timeout_s
        self.frame_id = frame_id
        self.cbg = ReentrantCallbackGroup()

        self.state = State()
        self.pose = None
        self._gripe, self._gripe_t = "", -1e9

        node.create_subscription(State, "/mavros/state", self._cb_state, 10,
                                 callback_group=self.cbg)
        node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, qos_profile_sensor_data,
                                 callback_group=self.cbg)
        node.create_subscription(StatusText, "/mavros/statustext/recv",
                                 self._cb_statustext, 10,
                                 callback_group=self.cbg)

        self.cli_mode = node.create_client(SetMode, "/mavros/set_mode",
                                           callback_group=self.cbg)
        self.cli_arm = node.create_client(CommandBool, "/mavros/cmd/arming",
                                          callback_group=self.cbg)
        self.cli_takeoff = node.create_client(CommandTOL, "/mavros/cmd/takeoff",
                                              callback_group=self.cbg)

        # The stream. Silent whenever the FCU owns the vehicle (LAND): a
        # position setpoint mid-descent is at best ignored, at worst fights
        # the flare.
        self.setpoint = None          # (x, y, z, yaw)
        self.streaming = False
        self.pub_sp = node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        node.create_timer(1.0 / max(setpoint_hz, 1.0), self._stream,
                          callback_group=self.cbg)

    # ── state ────────────────────────────────────────────────────────────────

    def _cb_state(self, msg):
        self.state = msg

    def _cb_pose(self, msg):
        self.pose = msg

    def _cb_statustext(self, msg):
        """Remember ArduPilot's most recent arm/pre-arm complaint."""
        text = msg.text.strip()
        if text.startswith(("Arm:", "PreArm:")):
            if text != self._gripe:
                self.node.get_logger().warn(f"FCU refuses: {text}")
            self._gripe, self._gripe_t = text, self.now()

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def fcu_reason(self) -> str:
        """The FCU's refusal, if recent enough to be about this attempt."""
        if self._gripe and self.now() - self._gripe_t < 10.0:
            return self._gripe
        return "no reason given by the FCU"

    @property
    def armed(self) -> bool:
        return bool(self.state.armed)

    @property
    def mode(self) -> str:
        return self.state.mode

    def ready(self) -> bool:
        return (self.state.connected and self.pose is not None
                and self.cli_arm.service_is_ready()
                and self.cli_mode.service_is_ready())

    def xyz(self):
        p = self.pose.pose.position
        return p.x, p.y, p.z

    def yaw(self) -> float:
        return yaw_of(self.pose)

    # ── commands (non-blocking: each returns a Call or None) ─────────────────

    def _call(self, tag, client, req):
        if not client.service_is_ready():
            self.node.get_logger().info(
                f"service {client.srv_name} not up yet",
                throttle_duration_sec=5.0)
            return None
        return Call(self.node, tag, client, req, self.svc_timeout)

    def set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        return self._call("mode", self.cli_mode, req)

    def arm(self, value: bool = True):
        return self._call("arm" if value else "disarm", self.cli_arm,
                          CommandBool.Request(value=value))

    def takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        return self._call("takeoff", self.cli_takeoff, req)

    # ── setpoint stream ──────────────────────────────────────────────────────

    def hold_here(self, z=None, yaw=None):
        x, y, z0 = self.xyz()
        self.go(x, y, z0 if z is None else z,
                self.yaw() if yaw is None else yaw)

    def go(self, x, y, z, yaw):
        self.setpoint = (float(x), float(y), float(z), float(yaw))
        self.streaming = True

    def stop_stream(self):
        self.streaming = False

    def _stream(self):
        if not self.streaming or self.setpoint is None:
            return
        x, y, z, yaw = self.setpoint
        sp = PoseStamped()
        sp.header.stamp = self.node.get_clock().now().to_msg()
        sp.header.frame_id = self.frame_id
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_sp.publish(sp)
