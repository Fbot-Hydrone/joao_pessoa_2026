#!/usr/bin/env python3
"""
pad_mission_node — the simplest mission that flies: go forward, land on what
the down camera sees, take off, repeat.

    take off  ->  fly straight ahead (+X)
              ->  down camera sees a pad  ->  stop and LAND on the spot
              ->  mark it visited in the map
              ->  take off again and carry on forward
              ->  ... forever, or until forward_limit_m is reached.

State machine
-------------
    WAIT_FCU -> ARMING -> TAKEOFF -> FORWARD -> LAND -> DWELL -+-> DONE
                  ^                                            |
                  +--------------------------------------------+

There is deliberately no search pattern, no forward-camera lead, no align or
descend phase, and no return-home leg. This is the skeleton: get one full
takeoff/detect/land/takeoff cycle working in the sim first, then add robustness
back one piece at a time. Anything more here is another thing that can break
while you are trying to find out why the drone will not fly.

Everything is driven by a 10 Hz tick and every service call is asynchronous with
its own deadline — a blocking call inside a timer callback would stall the
setpoint stream and hand the vehicle back to the FCU's failsafe.

Forward is world +X, not the vehicle's heading. The setpoints published here
carry yaw = 0, so the FCU holds the nose along +X anyway; making "forward" mean
anything else would put the flight path and the nose in disagreement. Spawn the
vehicle at x = 0 and it flies out along the arena's +X axis.

Interfaces
----------
in:   /hydrone/pads/detections    hydrone_msgs/PadDetection  (down camera only)
      /hydrone/pads/map           hydrone_msgs/PadMap        (only to name the
                                                              pad we landed on)
      /mavros/state, /mavros/local_position/pose
out:  /mavros/setpoint_position/local
      /hydrone/mission/status     std_msgs/String
srv:  /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff
      /hydrone/pads/mark_visited
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from hydrone_msgs.msg import PadDetection, PadMap
from hydrone_msgs.srv import MarkPadVisited


# ── Async service helper ─────────────────────────────────────────────────────

class _Call:
    """One in-flight service call with its own deadline.

    rclpy's spin_until_future_complete cannot be used here: this node's tick
    runs inside the executor, and blocking it would stop the setpoint stream.
    """

    PENDING, OK, FAILED, TIMEOUT = "pending", "ok", "failed", "timeout"

    def __init__(self, node: Node, client, request, timeout_s: float):
        self.name = client.srv_name
        self.future = client.call_async(request)
        self.deadline_ns = (node.get_clock().now().nanoseconds
                            + int(timeout_s * 1e9))

    def poll(self, now_ns: int) -> str:
        if self.future.done():
            result = self.future.result()
            if result is None:
                return self.FAILED
            # MAVROS replies use either `success` (CommandBool/CommandTOL) or
            # `mode_sent` (SetMode).
            ok = getattr(result, "success", None)
            if ok is None:
                ok = getattr(result, "mode_sent", False)
            return self.OK if ok else self.FAILED
        if now_ns > self.deadline_ns:
            self.future.cancel()
            return self.TIMEOUT
        return self.PENDING


class PadMissionNode(Node):

    # ── States ──────────────────────────────────────────────────────────────
    WAIT_FCU = "WAIT_FCU"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    FORWARD = "FORWARD"
    LAND = "LAND"
    DWELL = "DWELL"
    DONE = "DONE"
    ABORTED = "ABORTED"

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy's Node so tests can pass
        # parameter_overrides; the parameters here are read once, in __init__.
        super().__init__("pad_mission", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        # Metres above the takeoff plane, which is what the FCU's local frame
        # measures. Must clear the tallest thing in the arena — there is no
        # obstacle avoidance here.
        self.declare_parameter("cruise_alt", 2.5)

        # How far ahead the position setpoint is placed. Each step is a jump the
        # FCU's position controller answers with acceleration, so a big step is
        # an aggressive demand; keep it small.
        self.declare_parameter("forward_step", 1.0)
        self.declare_parameter("waypoint_tol", 0.5)
        # 0 = fly forward forever (until aborted). Anything else ends the run
        # with a landing once that many metres have been covered.
        self.declare_parameter("forward_limit_m", 0.0)

        # A detection counts if it is this confident and this recent.
        self.declare_parameter("min_confidence", 0.60)
        self.declare_parameter("fresh_detection_s", 1.0)
        # After taking off from a pad, that same pad is still directly below.
        # Ignore detections until this much ground has been covered, or the
        # mission lands on the pad it just left, forever.
        self.declare_parameter("rearm_distance_m", 3.0)

        self.declare_parameter("dwell_s", 4.0)
        self.declare_parameter("land_timeout_s", 60.0)
        self.declare_parameter("land_settle_s", 2.0)
        self.declare_parameter("takeoff_timeout_s", 45.0)
        self.declare_parameter("service_timeout_s", 30.0)
        self.declare_parameter("setpoint_hz", 10.0)
        self.declare_parameter("auto_start", True)
        # Gap between repeats of a mode/arm/takeoff command. MAVROS acking a
        # command is not the same as ArduPilot accepting it, so every command is
        # re-sent on this period until /mavros/state shows the effect.
        self.declare_parameter("retry_period_s", 2.0)

        p = lambda n: self.get_parameter(n).value
        self.cruise_alt = float(p("cruise_alt"))
        self.forward_step = float(p("forward_step"))
        self.wp_tol = float(p("waypoint_tol"))
        self.forward_limit = float(p("forward_limit_m"))
        self.min_conf = float(p("min_confidence"))
        self.fresh_s = float(p("fresh_detection_s"))
        self.rearm_distance = float(p("rearm_distance_m"))
        self.dwell_s = float(p("dwell_s"))
        self.land_timeout = float(p("land_timeout_s"))
        self.land_settle = float(p("land_settle_s"))
        self.takeoff_timeout = float(p("takeoff_timeout_s"))
        self.svc_timeout = float(p("service_timeout_s"))
        self.auto_start = bool(p("auto_start"))
        self.retry_period = float(p("retry_period_s"))

        # ── State ───────────────────────────────────────────────────────────
        self.state = self.WAIT_FCU
        self._state_since = self._now()
        self.mav_state = State()
        self.pose: PoseStamped | None = None
        self.pad_map: PadMap | None = None

        self.home: tuple[float, float] | None = None
        # Where the current forward leg started, so rearm_distance is measured
        # from the pad we just left rather than from home.
        self.leg_start_x = 0.0
        self.target_x = 0.0
        self.landed_count = 0

        self.setpoint: list[float] = [0.0, 0.0, 0.0]
        self.stream_setpoint = False
        self._settle_since: float | None = None
        self._call: _Call | None = None
        self._pending: str | None = None    # what _call is for
        self._last_cmd_t = 0.0              # when the last command went out
        self._takeoff_tries = 0
        self._last_down: PadDetection | None = None
        self._last_down_t = 0.0
        # Last refusal ArduPilot gave us, and when. See _cb_statustext.
        self._fcu_gripe = ""
        self._fcu_gripe_t = 0.0

        # ── I/O ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_sp = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_status = self.create_publisher(
            String, "/hydrone/mission/status", 10)

        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, sensor_qos)
        self.create_subscription(PadMap, "/hydrone/pads/map", self._cb_map, 10)
        self.create_subscription(PadDetection, "/hydrone/pads/detections",
                                 self._cb_detection, 20)
        # ArduPilot explains its refusals in STATUSTEXT ("Arm: Throttle too
        # high", "PreArm: VisOdom: not healthy", ...). MAVROS logs those on its
        # own logger, which means the reason and the failure end up in two
        # different places and correlating them means diffing timestamps across
        # log streams. Mirror it here so the mission's own log says why.
        # MAVROS publishes statustext/recv BEST_EFFORT; a plain (RELIABLE)
        # subscription is QoS-incompatible and receives NOTHING — measured
        # 2026-08-18, every arm failure said "no reason given by the FCU" while
        # MAVROS was logging the real reason ("Arm: Accels inconsistent").
        self.create_subscription(StatusText, "/mavros/statustext/recv",
                                 self._cb_statustext, sensor_qos)

        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_visited = self.create_client(MarkPadVisited,
                                              "/hydrone/pads/mark_visited")

        self.create_service(Trigger, "/hydrone/mission/start", self._svc_start)
        self.create_service(Trigger, "/hydrone/mission/abort", self._svc_abort)

        self.create_timer(1.0 / max(float(p("setpoint_hz")), 1.0),
                          self._stream)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_status)

        limit = (f"{self.forward_limit:.0f} m of it"
                 if self.forward_limit > 0.0 else "no distance limit")
        self.get_logger().info(
            f"pad_mission ready — forward along +X at {self.cruise_alt:.1f} m "
            f"in {self.forward_step:.1f} m steps, {limit}, landing on whatever "
            "the down camera sees. "
            f"{'Auto-starting.' if self.auto_start else 'Call /hydrone/mission/start.'}")

    # ────────────────────────────────────────────────────────────────────────
    # Inputs
    # ────────────────────────────────────────────────────────────────────────

    def _cb_state(self, msg: State):
        self.mav_state = msg

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_map(self, msg: PadMap):
        self.pad_map = msg

    def _cb_detection(self, msg: PadDetection):
        """Keep the freshest look from the DOWN camera. The forward camera is
        ignored entirely: it sees pads at a range this mission has no phase to
        fly to, so acting on it would only complicate the path."""
        if msg.camera == "down":
            self._last_down = msg
            self._last_down_t = self._now()

    def _cb_statustext(self, msg: StatusText):
        """Remember ArduPilot's most recent arm/pre-arm complaint."""
        text = msg.text.strip()
        if text.startswith(("Arm:", "PreArm:")):
            if text != self._fcu_gripe:
                self.get_logger().warn(f"FCU refuses: {text}")
            self._fcu_gripe = text
            self._fcu_gripe_t = self._now()

    def _fcu_reason(self) -> str:
        """The FCU's refusal, if it is recent enough to be about this attempt."""
        if self._fcu_gripe and self._now() - self._fcu_gripe_t < 10.0:
            return self._fcu_gripe
        return "no reason given by the FCU"

    # ────────────────────────────────────────────────────────────────────────
    # Services
    # ────────────────────────────────────────────────────────────────────────

    def _svc_start(self, request, response):
        self.auto_start = True
        if self.state in (self.DONE, self.ABORTED):
            self._reset()
        response.success = True
        response.message = "mission armed to start"
        return response

    def _svc_abort(self, request, response):
        self.get_logger().warn("ABORT requested — landing where we are.")
        self.stream_setpoint = False
        self._enter(self.ABORTED)
        self._set_mode("LAND")
        response.success = True
        response.message = "aborting: landing in place"
        return response

    def _reset(self):
        self.state = self.WAIT_FCU
        self.home = None
        self.landed_count = 0

    # ────────────────────────────────────────────────────────────────────────
    # Setpoint stream
    # ────────────────────────────────────────────────────────────────────────

    def _stream(self):
        """Publish the current position target.

        Deliberately silent whenever the FCU owns the descent (LAND, DWELL): a
        position setpoint arriving mid-landing is at best ignored and at worst
        fights the flare.
        """
        if not self.stream_setpoint:
            return
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(self.setpoint[0])
        sp.pose.position.y = float(self.setpoint[1])
        sp.pose.position.z = float(self.setpoint[2])
        sp.pose.orientation.w = 1.0
        self.pub_sp.publish(sp)

    def _goto(self, x: float, y: float, z: float):
        self.setpoint = [x, y, z]
        self.stream_setpoint = True

    # ────────────────────────────────────────────────────────────────────────
    # State machine
    # ────────────────────────────────────────────────────────────────────────

    def _tick(self):
        if self.state in (self.DONE, self.ABORTED):
            return
        handler = {
            self.WAIT_FCU: self._do_wait_fcu,
            self.ARMING: self._do_arming,
            self.TAKEOFF: self._do_takeoff,
            self.FORWARD: self._do_forward,
            self.LAND: self._do_land,
            self.DWELL: self._do_dwell,
        }[self.state]
        handler()

    # ── WAIT_FCU ─────────────────────────────────────────────────────────────

    def _do_wait_fcu(self):
        if not self.auto_start:
            return
        if not (self.mav_state.connected and self.pose is not None):
            self._throttle("waiting for MAVROS link and a local position...")
            return
        if not (self.cli_arm.service_is_ready()
                and self.cli_mode.service_is_ready()):
            self._throttle("waiting for MAVROS command services...")
            return

        # Home is wherever we are standing; the forward run is measured from it.
        self.home = (self.pose.pose.position.x, self.pose.pose.position.y)
        self.get_logger().info(
            f"home = ({self.home[0]:.1f}, {self.home[1]:.1f}) — flying +X "
            "from here.")
        self._enter(self.ARMING)

    # ── ARMING ───────────────────────────────────────────────────────────────

    def _do_arming(self):
        """GUIDED first, then arm.

        Retries are driven by elapsed time rather than by the previous call's
        result: MAVROS acks a mode change that ArduPilot then declines (EKF not
        ready, pre-arm check pending), so "the call succeeded" is not the same
        as "the vehicle is in GUIDED". Only /mavros/state settles that.

        GUIDED is checked BEFORE armed, which matters on the relaunch after a
        landing: ArduCopter auto-disarms only after DISARM_DELAY (10 s by
        default) and dwell is shorter than that, so the vehicle is usually still
        armed — and still in LAND. Taking "armed" as done would send a takeoff
        while in LAND, which ArduPilot refuses, forever.
        """
        if self.mav_state.mode == "GUIDED" and self.mav_state.armed:
            self._takeoff_tries = 0
            self._enter(self.TAKEOFF)
            return
        if self._poll_call() == "pending":
            return
        if self._now() - self._last_cmd_t < self.retry_period:
            return

        if self.mav_state.mode != "GUIDED":
            self._set_mode("GUIDED")
            return

        self._start_call("arm", self.cli_arm, CommandBool.Request(value=True))

        if self._since_entered() > self.takeoff_timeout:
            self.get_logger().warn(
                f"still not armed after {self.takeoff_timeout:.0f} s. "
                f"ArduPilot says: {self._fcu_reason()}. Still retrying. "
                "(Common causes: no EKF origin yet; no vision pose reaching the "
                "FCU — check /mavros/vision_pose/pose; a GCS virtual joystick "
                "holding the throttle stick off minimum.)")
            self._state_since = self._now()

    # ── TAKEOFF ──────────────────────────────────────────────────────────────

    def _do_takeoff(self):
        """Ask the FCU to climb to cruise altitude.

        ArduCopter will not climb from a bare position setpoint in GUIDED — it
        needs an explicit takeoff — so this is a command, not a setpoint, and the
        setpoint stream only starts once we are up.
        """
        if (self.pose is not None
                and self.pose.pose.position.z >= self.cruise_alt - 0.25):
            x = self.pose.pose.position.x
            self.get_logger().info(
                f"airborne at {self.pose.pose.position.z:.2f} m — "
                "heading forward.")
            # The pad we just left is at this x. Both the ignore window and the
            # first waypoint are measured from here.
            self.leg_start_x = x
            self.target_x = x + self.forward_step
            self._goto(self.target_x, self.home[1], self.cruise_alt)
            self._enter(self.FORWARD)
            return

        if self._poll_call() == "pending":
            return

        if self._since_entered() > self.takeoff_timeout:
            self._takeoff_tries += 1
            if self._takeoff_tries > 3:
                self.get_logger().error(
                    "takeoff refused three times — check EKF origin/home "
                    "(see docs/DEVELOP-PIPELINES.md: no origin -> no home -> "
                    "NAV_TAKEOFF fails). Aborting.")
                self._enter(self.ABORTED)
                return
            self.get_logger().warn("takeoff did not lift us; retrying.")
            self._enter(self.ARMING)
            return

        if self._now() - self._last_cmd_t >= self.retry_period:
            req = CommandTOL.Request()
            req.altitude = float(self.cruise_alt)
            self._start_call("takeoff", self.cli_takeoff, req)

    # ── FORWARD ──────────────────────────────────────────────────────────────

    def _do_forward(self):
        """Walk the setpoint forward along +X, landing the moment a pad shows up.

        The setpoint is placed one `forward_step` ahead and only advanced once
        the vehicle has actually got there — so the position error the FCU sees
        never exceeds one step, and the acceleration it demands stays small.
        """
        if self.pose is None:
            return

        if self._pad_below():
            self.get_logger().info(
                f"pad below (conf {self._last_down.confidence:.2f}) after "
                f"{self._flown():.1f} m — stopping and landing here.")
            self.stream_setpoint = False
            self._settle_since = None
            self._enter(self.LAND)
            self._set_mode("LAND")
            return

        if self.forward_limit > 0.0:
            travelled = self.pose.pose.position.x - self.home[0]
            if travelled >= self.forward_limit:
                self.get_logger().info(
                    f"{travelled:.1f} m covered (forward_limit_m) and "
                    f"{self.landed_count} landing(s) made — landing to finish.")
                self.stream_setpoint = False
                self._enter(self.ABORTED)
                self._set_mode("LAND")
                return

        self._goto(self.target_x, self.home[1], self.cruise_alt)
        if self.pose.pose.position.x >= self.target_x - self.wp_tol:
            self.target_x += self.forward_step
            self._throttle(f"forward {self._flown():.1f} m on this leg.")

    def _flown(self) -> float:
        """Distance covered since the last takeoff."""
        if self.pose is None:
            return 0.0
        return self.pose.pose.position.x - self.leg_start_x

    def _pad_below(self) -> bool:
        """Is the down camera looking at a pad we have not just taken off from?"""
        if self._flown() < self.rearm_distance:
            return False
        if self._last_down is None:
            return False
        if self._now() - self._last_down_t > self.fresh_s:
            return False
        return self._last_down.confidence >= self.min_conf

    # ── LAND ─────────────────────────────────────────────────────────────────

    def _do_land(self):
        """Wait for touchdown. Two independent signals, whichever comes first."""
        # Keep asking until /mavros/state agrees we are in LAND. An acked mode
        # command that ArduPilot then declined would otherwise leave us hovering
        # here until the timeout.
        if (self.mav_state.mode != "LAND"
                and self._poll_call() != "pending"
                and self._now() - self._last_cmd_t >= self.retry_period):
            self.get_logger().warn("not in LAND yet; re-sending the mode.")
            self._set_mode("LAND")
            return

        # Landed = the FCU disarmed us, or we are sitting near the takeoff plane.
        # A pad is low enough that its top surface is still well under 0.5 m.
        disarmed = not self.mav_state.armed
        low = self.pose is not None and self.pose.pose.position.z <= 0.5

        if disarmed or low:
            if self._settle_since is None:
                self._settle_since = self._now()
            elif self._now() - self._settle_since >= self.land_settle:
                z = self.pose.pose.position.z if self.pose else 0.0
                self.landed_count += 1
                self.get_logger().info(
                    f"LANDED (#{self.landed_count}) — resting at z={z:.2f} m.")
                self._mark_visited(z)
                self._enter(self.DWELL)
        else:
            self._settle_since = None

        if self._since_entered() > self.land_timeout:
            self.get_logger().warn(
                f"no touchdown within {self.land_timeout:.0f} s — carrying on "
                "anyway so the mission does not stall here.")
            self._enter(self.DWELL)

    def _mark_visited(self, height: float):
        """Tell the pad map we landed, so it stops offering this pad.

        The map is only read here, to put a name to the thing under us: whatever
        pad entry is nearest is the one we are standing on. If the map has
        nothing (it is optional to this mission), the landing simply goes
        unrecorded — the forward run does not depend on it.
        """
        pad_id = self._pad_id_below()
        if pad_id is None:
            self.get_logger().warn(
                "landed, but no pad in /hydrone/pads/map is near enough to mark "
                "visited — the map will not know about this landing.")
            return
        if not self.cli_visited.service_is_ready():
            self.get_logger().warn(
                "/hydrone/pads/mark_visited unavailable — the map will not know "
                "about this landing.")
            return
        req = MarkPadVisited.Request()
        req.id = int(pad_id)
        req.height_valid = True
        # The altitude we came to rest at IS the pad's top surface.
        req.height = float(height)
        self.cli_visited.call_async(req)
        self.get_logger().info(f"pad {pad_id} marked visited.")

    def _pad_id_below(self) -> int | None:
        if self.pad_map is None or self.pose is None:
            return None
        best, best_d = None, 2.0     # nothing further than 2 m is "under us"
        for pad in self.pad_map.pads:
            d = math.hypot(self.pose.pose.position.x - pad.position.x,
                           self.pose.pose.position.y - pad.position.y)
            if d < best_d:
                best, best_d = pad.id, d
        return best

    # ── DWELL ────────────────────────────────────────────────────────────────

    def _do_dwell(self):
        """Sit on the pad, then go again."""
        if self._since_entered() < self.dwell_s:
            return
        self._takeoff_tries = 0
        self.get_logger().info("taking off again to carry on forward.")
        self._enter(self.ARMING)

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self._start_call("mode", self.cli_mode, req)

    def _start_call(self, tag: str, client, request):
        if not client.service_is_ready():
            self._throttle(f"service {client.srv_name} not up yet")
            return
        self._pending = tag
        self._last_cmd_t = self._now()
        self._call = _Call(self, client, request, self.svc_timeout)

    def _poll_call(self) -> str:
        """Advance the in-flight call. Returns its status, 'idle' if none."""
        if self._call is None:
            return "idle"
        status = self._call.poll(self.get_clock().now().nanoseconds)
        if status == _Call.PENDING:
            return "pending"
        if status != _Call.OK:
            self.get_logger().warn(
                f"{self._call.name} ({self._pending}) -> {status}"
                + (f": {self._fcu_reason()}" if self._pending in ("arm", "takeoff")
                   else ""))
        self._call = None
        return status

    def _enter(self, state: str):
        if state != self.state:
            self.get_logger().info(f"[{self.state} -> {state}]")
        self.state = state
        self._state_since = self._now()
        self._call = None
        self._pending = None
        # Let the new state issue its first command immediately rather than
        # waiting out the retry period of whatever the old state was doing.
        self._last_cmd_t = 0.0

    def _since_entered(self) -> float:
        return self._now() - self._state_since

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _throttle(self, text: str):
        self.get_logger().info(text, throttle_duration_sec=5.0)

    def _publish_status(self):
        z = self.pose.pose.position.z if self.pose else float("nan")
        x = self.pose.pose.position.x if self.pose else float("nan")
        self.pub_status.publish(String(data=(
            f"state={self.state} mode={self.mav_state.mode} "
            f"armed={self.mav_state.armed} x={x:.2f} z={z:.2f} "
            f"leg={self._flown():.1f}m landed={self.landed_count}")))


def main(args=None):
    rclpy.init(args=args)
    node = PadMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
