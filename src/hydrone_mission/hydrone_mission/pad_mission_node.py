#!/usr/bin/env python3
"""
pad_mission_node — search the arena, land on every landing pad, take off again.

The behaviour, end to end:

    take off  ->  fly a bounded search pattern  ->  spot a pad ahead on the ZED
              ->  fly over it and confirm it on the down camera
              ->  align, descend, LAND
              ->  sit on it, mark it visited
              ->  take off again and resume the search where it left off
              ->  ... until nothing unvisited is left, then come home and land.

State machine
-------------
    WAIT_FCU -> ARMING -> TAKEOFF -> SEARCH -+-> INSPECT -> ALIGN -> DESCEND
                                    ^        |                        |
                                    |        |                     LAND -> DWELL
                                    +--------+------------------------+
                                    |
                                    +-> RETURN_HOME -> FINAL_LAND -> DONE

Every transition is driven by a 10 Hz tick, every service call is asynchronous
with its own deadline, and nothing ever blocks the executor — a blocking call
inside a timer callback would stall the setpoint stream and hand the vehicle
back to the FCU's failsafe.

Why the search is bounded
-------------------------
The world outside the arena is an empty plane. A drone told to "fly forward
until you see something" flies forward forever. The search is therefore an
expanding square spiral, centred on the takeoff point and clipped to
`search_radius`, whose FIRST leg runs straight ahead — so the simple case ("take
off, go forward, the pad is there") is the first thing that happens, and the
failure case ("there is nothing out there") terminates instead of running until
the battery does.

Why a pad is confirmed twice
----------------------------
The forward camera finds pads at range, where the detector cannot resolve the
ring and the cross and so caps its confidence at 0.75 (see pad_detector.py).
That is enough to be worth flying to, not enough to land on. The drone flies
over the candidate and re-checks it with the down camera from a few metres, where
the structure IS resolvable and confidence clears `commit_confidence`. A
candidate that fails to confirm is blacklisted and the search resumes — that is
how a blue tarp costs twenty seconds instead of the mission.

Interfaces
----------
in:   /hydrone/pads/map           hydrone_msgs/PadMap        (what to fly to)
      /hydrone/pads/detections    hydrone_msgs/PadDetection  (fresh down-cam look)
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


# ── Search pattern ───────────────────────────────────────────────────────────

def spiral_waypoints(step: float, radius: float,
                     heading: float = 0.0) -> list[tuple[float, float]]:
    """Expanding square spiral about the origin, clipped to a square of `radius`.

    Leg lengths go 1,1,2,2,3,3,... times `step`, turning left each time — the
    classic lawn-search pattern, which covers the area near the start first
    (where the pad probably is) without ever revisiting a leg.

    The first leg runs along `heading` (radians, 0 = +X), so with the drone's
    takeoff heading passed in, "take off and go forward" is literally step one.
    """
    if step <= 0.0 or radius <= 0.0:
        return []

    ch, sh = math.cos(heading), math.sin(heading)
    # Unit legs in the rotated frame: forward, left, back, right.
    dirs = [(ch, sh), (-sh, ch), (-ch, -sh), (sh, -ch)]

    points: list[tuple[float, float]] = []
    x = y = 0.0
    leg = 1
    d = 0
    # Once a leg is longer than the diameter, every further waypoint is outside.
    while leg * step <= 2.0 * radius + step:
        for _ in range(2):
            dx, dy = dirs[d % 4]
            for _ in range(leg):
                x += dx * step
                y += dy * step
                if math.hypot(x, y) <= radius:
                    points.append((x, y))
            d += 1
        leg += 1
    return points


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
    SEARCH = "SEARCH"
    INSPECT = "INSPECT"
    ALIGN = "ALIGN"
    DESCEND = "DESCEND"
    LAND = "LAND"
    DWELL = "DWELL"
    RETURN_HOME = "RETURN_HOME"
    FINAL_LAND = "FINAL_LAND"
    DONE = "DONE"
    ABORTED = "ABORTED"

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy's Node so tests can pass
        # parameter_overrides; the parameters here are read once, in __init__.
        super().__init__("pad_mission", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        # Altitudes are metres above the takeoff plane, which is what the FCU's
        # local frame measures and what the pad map stores.
        # cruise_alt MUST clear the tallest thing in the arena (the 1.5 m
        # structure) — there is no forward obstacle avoidance in this node.
        self.declare_parameter("cruise_alt", 2.5)
        self.declare_parameter("align_alt", 2.0)
        # Height above the PAD's own top surface at which LAND is handed to the
        # FCU. Its rangefinder flare takes it from there.
        self.declare_parameter("land_trigger_agl", 0.8)
        self.declare_parameter("descend_step", 0.25)

        self.declare_parameter("search_radius", 12.0)
        self.declare_parameter("search_step", 3.0)
        self.declare_parameter("waypoint_tol", 0.6)
        self.declare_parameter("align_tol", 0.25)
        self.declare_parameter("align_hold_s", 2.0)

        # Below this a detection is a lead, not a landing site. 0.75 is the
        # ceiling for an unresolved pad, so >0.75 means the ring AND the cross
        # were actually verified — see pad_detector.py.
        self.declare_parameter("commit_confidence", 0.80)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("fresh_detection_s", 2.0)
        self.declare_parameter("inspect_timeout_s", 25.0)
        # Separate budget for GETTING to a candidate, which under a
        # slow-running sim dominates. See _do_inspect.
        self.declare_parameter("travel_timeout_s", 120.0)

        self.declare_parameter("dwell_s", 4.0)
        self.declare_parameter("land_timeout_s", 60.0)
        self.declare_parameter("land_settle_s", 2.0)
        self.declare_parameter("takeoff_timeout_s", 45.0)
        self.declare_parameter("service_timeout_s", 30.0)
        self.declare_parameter("mission_timeout_s", 900.0)
        self.declare_parameter("max_pads", 0)     # 0 = no limit
        self.declare_parameter("setpoint_hz", 10.0)
        self.declare_parameter("auto_start", True)
        # Gap between repeats of a mode/arm/takeoff command. MAVROS acking a
        # command is not the same as ArduPilot accepting it, so every command is
        # re-sent on this period until /mavros/state shows the effect.
        self.declare_parameter("retry_period_s", 2.0)

        p = lambda n: self.get_parameter(n).value
        self.cruise_alt = float(p("cruise_alt"))
        self.align_alt = float(p("align_alt"))
        self.land_trigger_agl = float(p("land_trigger_agl"))
        self.descend_step = float(p("descend_step"))
        self.search_radius = float(p("search_radius"))
        self.search_step = float(p("search_step"))
        self.wp_tol = float(p("waypoint_tol"))
        self.align_tol = float(p("align_tol"))
        self.align_hold = float(p("align_hold_s"))
        self.commit_conf = float(p("commit_confidence"))
        self.min_obs = int(p("min_observations"))
        self.fresh_s = float(p("fresh_detection_s"))
        self.inspect_timeout = float(p("inspect_timeout_s"))
        self.travel_timeout = float(p("travel_timeout_s"))
        self.dwell_s = float(p("dwell_s"))
        self.land_timeout = float(p("land_timeout_s"))
        self.land_settle = float(p("land_settle_s"))
        self.takeoff_timeout = float(p("takeoff_timeout_s"))
        self.svc_timeout = float(p("service_timeout_s"))
        self.mission_timeout = float(p("mission_timeout_s"))
        self.max_pads = int(p("max_pads"))
        self.auto_start = bool(p("auto_start"))
        self.retry_period = float(p("retry_period_s"))

        # ── State ───────────────────────────────────────────────────────────
        self.state = self.WAIT_FCU
        self._state_since = self._now()
        self.mav_state = State()
        self.pose: PoseStamped | None = None
        self.pad_map: PadMap | None = None

        self.home: tuple[float, float] | None = None
        self.home_yaw = 0.0
        self.route: list[tuple[float, float]] = []
        self.route_idx = 0

        self.target_id: int | None = None
        self.target_xy: tuple[float, float] = (0.0, 0.0)
        self.target_height = 0.0
        self.blacklist: set[int] = set()
        self.landed_count = 0

        self.setpoint: list[float] = [0.0, 0.0, 0.0]
        self.stream_setpoint = False
        self._descend_target_z = 0.0
        self._align_ok_since: float | None = None
        self._arrived_since: float | None = None
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
        self._mission_start: float | None = None

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
        self.create_subscription(StatusText, "/mavros/statustext/recv",
                                 self._cb_statustext, 10)

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

        self.get_logger().info(
            f"pad_mission ready — search {self.search_radius:.0f} m radius in "
            f"{self.search_step:.0f} m legs at {self.cruise_alt:.1f} m. "
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
        """Keep the freshest look from the DOWN camera — that is the one that
        proves we are actually over the pad, not merely near its map entry."""
        if msg.camera == "down" and msg.position_valid:
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
        self.route, self.route_idx = [], 0
        self.home = None
        self.target_id = None
        self.blacklist.clear()
        self.landed_count = 0
        self._mission_start = None

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
        if self._mission_start is not None:
            elapsed = self._now() - self._mission_start
            if elapsed > self.mission_timeout:
                self.get_logger().warn(
                    f"mission timeout after {elapsed:.0f} s — landing in place.")
                self.stream_setpoint = False
                self._enter(self.ABORTED)
                self._set_mode("LAND")
                return

        handler = {
            self.WAIT_FCU: self._do_wait_fcu,
            self.ARMING: self._do_arming,
            self.TAKEOFF: self._do_takeoff,
            self.SEARCH: self._do_search,
            self.INSPECT: self._do_inspect,
            self.ALIGN: self._do_align,
            self.DESCEND: self._do_descend,
            self.LAND: self._do_land,
            self.DWELL: self._do_dwell,
            self.RETURN_HOME: self._do_return_home,
            self.FINAL_LAND: self._do_final_land,
        }[self.state]
        handler()

    # ── WAIT_FCU ─────────────────────────────────────────────────────────────

    def _do_wait_fcu(self):
        if not self.auto_start:
            return
        if not (self.mav_state.connected and self.pose is not None):
            self._throttle("waiting for MAVROS link and a local position...")
            return
        if not self.cli_arm.service_is_ready() or not self.cli_mode.service_is_ready():
            self._throttle("waiting for MAVROS command services...")
            return

        # Home is wherever we are standing. Everything downstream — the search
        # spiral, the return leg — is relative to it.
        self.home = (self.pose.pose.position.x, self.pose.pose.position.y)
        self.home_yaw = self._yaw()
        self.route = spiral_waypoints(self.search_step, self.search_radius,
                                      self.home_yaw)
        self.route_idx = 0
        self._mission_start = self._now()
        self.get_logger().info(
            f"home = ({self.home[0]:.1f}, {self.home[1]:.1f}), heading "
            f"{math.degrees(self.home_yaw):.0f} deg. "
            f"{len(self.route)} search waypoints planned.")
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
        if self.pose is not None and self.pose.pose.position.z >= self.cruise_alt - 0.25:
            self.get_logger().info(
                f"airborne at {self.pose.pose.position.z:.2f} m — searching.")
            self._goto(self.pose.pose.position.x, self.pose.pose.position.y,
                       self.cruise_alt)
            self._enter(self.SEARCH)
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

    # ── SEARCH ───────────────────────────────────────────────────────────────

    def _do_search(self):
        """Fly the spiral, breaking off the moment the map offers a target."""
        # Checked before target selection: once the landing quota is met, a pad
        # appearing in the map must not pull the drone back out again.
        if self.max_pads and self.landed_count >= self.max_pads:
            self.get_logger().info(
                f"{self.landed_count} pads visited (max_pads) — heading home.")
            self._enter(self.RETURN_HOME)
            return

        target = self._next_target()
        if target is not None:
            self.target_id = target.id
            self.target_xy = (target.position.x, target.position.y)
            self.target_height = target.height
            self.get_logger().info(
                f"pad {target.id} at ({target.position.x:.2f}, "
                f"{target.position.y:.2f}) looks like a landing site "
                f"(conf {target.confidence:.2f}, {target.observations} looks) "
                "— going to check it.")
            self._enter(self.INSPECT)
            return

        if self.route_idx >= len(self.route):
            self.get_logger().info(
                "search pattern exhausted and no unvisited pad left — "
                f"{self.landed_count} landing(s) made. Heading home.")
            self._enter(self.RETURN_HOME)
            return

        wx, wy = self.route[self.route_idx]
        gx, gy = self.home[0] + wx, self.home[1] + wy
        self._goto(gx, gy, self.cruise_alt)

        if self._dist_xy(gx, gy) < self.wp_tol:
            self.route_idx += 1
            self.get_logger().info(
                f"search waypoint {self.route_idx}/{len(self.route)} reached.")

    def _next_target(self):
        """The nearest confirmed, unvisited, un-blacklisted pad — or None."""
        if self.pad_map is None or self.pose is None:
            return None
        best, best_d = None, float("inf")
        for pad in self.pad_map.pads:
            if pad.visited or pad.id in self.blacklist:
                continue
            if pad.observations < self.min_obs:
                continue
            d = self._dist_xy(pad.position.x, pad.position.y)
            if d < best_d:
                best, best_d = pad, d
        return best

    # ── INSPECT ──────────────────────────────────────────────────────────────

    def _do_inspect(self):
        """Fly over the candidate and wait for the down camera to vouch for it.

        The two clocks here are deliberately separate. `inspect_timeout` is the
        patience for "I am hovering over it and the down camera still will not
        confirm" — the actual verdict on the candidate. Flying there is a
        different thing entirely, and gets `travel_timeout`.

        Merging them would blacklist good pads: these are wall-clock budgets and
        BiguaSim runs well below real time, so the flight across the arena eats
        far more of the budget than it appears to (same trap as
        config/timeouts.yaml documents for the MAVROS timeouts).
        """
        self._refresh_target()
        self._goto(self.target_xy[0], self.target_xy[1], self.cruise_alt)

        if self._down_confirms():
            self.get_logger().info(
                f"pad {self.target_id} confirmed from below "
                f"(conf {self._last_down.confidence:.2f}) — aligning.")
            self._align_ok_since = None
            self._enter(self.ALIGN)
            return

        if self._dist_xy(*self.target_xy) > 2.0 * self.wp_tol:
            self._arrived_since = None          # still on the way
        elif self._arrived_since is None:
            self._arrived_since = self._now()

        stalled = (self._arrived_since is not None
                   and self._now() - self._arrived_since > self.inspect_timeout)
        unreachable = self._since_entered() > self.travel_timeout

        if stalled or unreachable:
            self.get_logger().warn(
                f"pad {self.target_id} "
                + ("never confirmed from below while hovering over it"
                   if stalled else "could not be reached")
                + " — blacklisting it and resuming the search. "
                  "(A blue object that is not a pad ends up here.)")
            self.blacklist.add(self.target_id)
            self.target_id = None
            self._enter(self.SEARCH)

    def _down_confirms(self) -> bool:
        """Is there a fresh, confident down-camera look at THIS pad?"""
        if self._last_down is None:
            return False
        if self._now() - self._last_down_t > self.fresh_s:
            return False
        if self._last_down.confidence < self.commit_conf:
            return False
        return math.hypot(self._last_down.position.x - self.target_xy[0],
                          self._last_down.position.y - self.target_xy[1]) < 1.5

    # ── ALIGN ────────────────────────────────────────────────────────────────

    def _do_align(self):
        """Hold station over the pad until the error is small AND stays small.

        The hold matters: a single frame inside tolerance can be a swing through
        the target. Descending on that would drift off the pad on the way down.
        """
        self._refresh_target()
        self._goto(self.target_xy[0], self.target_xy[1], self.align_alt)

        err = self._dist_xy(*self.target_xy)
        near_alt = (self.pose is not None
                    and abs(self.pose.pose.position.z - self.align_alt) < 0.4)

        if err < self.align_tol and near_alt:
            if self._align_ok_since is None:
                self._align_ok_since = self._now()
            elif self._now() - self._align_ok_since >= self.align_hold:
                self._descend_target_z = self.align_alt
                self.get_logger().info(
                    f"centred over pad {self.target_id} "
                    f"({err * 100:.0f} cm) — descending.")
                self._enter(self.DESCEND)
        else:
            self._align_ok_since = None

        if self._since_entered() > self.inspect_timeout:
            self.get_logger().warn(
                f"could not settle over pad {self.target_id} "
                f"(error {err:.2f} m) — resuming the search.")
            self.blacklist.add(self.target_id)
            self.target_id = None
            self._enter(self.SEARCH)

    # ── DESCEND ──────────────────────────────────────────────────────────────

    def _do_descend(self):
        """Walk the setpoint down, re-centring on every fresh down-cam look.

        A ramped setpoint rather than one big step: the FCU tracks a moving
        target smoothly, and if the pad drifts out from under us the descent
        stops with plenty of height in hand.
        """
        self._refresh_target()
        floor = self.target_height + self.land_trigger_agl

        if self.pose is not None:
            # Only advance the ramp once the vehicle has caught up with it.
            if self.pose.pose.position.z < self._descend_target_z + 0.2:
                self._descend_target_z = max(
                    floor, self._descend_target_z - self.descend_step)

        self._goto(self.target_xy[0], self.target_xy[1], self._descend_target_z)

        err = self._dist_xy(*self.target_xy)
        if err > 3.0 * self.align_tol:
            self.get_logger().warn(
                f"drifted {err:.2f} m off pad {self.target_id} — climbing back "
                "to re-align.")
            self._align_ok_since = None
            self._enter(self.ALIGN)
            return

        if (self.pose is not None
                and self.pose.pose.position.z <= floor + 0.15
                and self._descend_target_z <= floor + 1e-3):
            self.get_logger().info(
                f"handing the last {self.land_trigger_agl:.1f} m to the FCU: "
                "LAND.")
            self.stream_setpoint = False
            self._settle_since = None
            self._enter(self.LAND)
            self._set_mode("LAND")

    # ── LAND ─────────────────────────────────────────────────────────────────

    def _do_land(self):
        """Wait for touchdown. Two independent signals, whichever comes first."""
        self._refresh_target()

        # Keep asking until /mavros/state agrees we are in LAND. An acked mode
        # command that ArduPilot then declined would otherwise leave us hovering
        # here until the timeout.
        if (self.mav_state.mode != "LAND"
                and self._poll_call() != "pending"
                and self._now() - self._last_cmd_t >= self.retry_period):
            self.get_logger().warn("not in LAND yet; re-sending the mode.")
            self._set_mode("LAND")
            return

        disarmed = not self.mav_state.armed
        on_pad = False
        if self.pose is not None:
            on_pad = (self.pose.pose.position.z
                      <= self.target_height + 0.25)

        if disarmed or on_pad:
            if self._settle_since is None:
                self._settle_since = self._now()
            elif self._now() - self._settle_since >= self.land_settle:
                z = self.pose.pose.position.z if self.pose else self.target_height
                self.landed_count += 1
                self.get_logger().info(
                    f"LANDED on pad {self.target_id} "
                    f"(#{self.landed_count}) — resting at z={z:.2f} m.")
                self._mark_visited(z)
                self._enter(self.DWELL)
        else:
            self._settle_since = None

        if self._since_entered() > self.land_timeout:
            self.get_logger().warn(
                f"no touchdown on pad {self.target_id} within "
                f"{self.land_timeout:.0f} s — marking it visited anyway so the "
                "mission moves on.")
            self._mark_visited(None)
            self._enter(self.DWELL)

    def _mark_visited(self, height: float | None):
        """Record the landing. The resting altitude IS the pad's top surface."""
        req = MarkPadVisited.Request()
        req.id = int(self.target_id if self.target_id is not None else 0)
        req.height_valid = height is not None
        req.height = float(height if height is not None else 0.0)
        if self.cli_visited.service_is_ready():
            self.cli_visited.call_async(req)
        else:
            self.get_logger().warn(
                "/hydrone/pads/mark_visited unavailable — the pad map will not "
                "know we landed, and the pad may be targeted again.")
        # Local guard: even if the service call is lost, this run will not
        # re-target the pad it is standing on.
        if self.target_id is not None:
            self.blacklist.add(self.target_id)

    # ── DWELL ────────────────────────────────────────────────────────────────

    def _do_dwell(self):
        """Sit on the pad, then go again."""
        if self._since_entered() < self.dwell_s:
            return
        self.target_id = None
        self._takeoff_tries = 0
        self.get_logger().info("taking off again to continue the search.")
        self._enter(self.ARMING)

    # ── RETURN_HOME / FINAL_LAND ─────────────────────────────────────────────

    def _do_return_home(self):
        # A pad discovered late is still worth taking, so keep looking on the
        # way back — unless the landing quota is already met.
        quota_met = bool(self.max_pads) and self.landed_count >= self.max_pads
        target = None if quota_met else self._next_target()
        if target is not None:
            self.target_id = target.id
            self.target_xy = (target.position.x, target.position.y)
            self.target_height = target.height
            self.get_logger().info(
                f"pad {target.id} appeared on the way home — diverting.")
            self._enter(self.INSPECT)
            return

        self._goto(self.home[0], self.home[1], self.cruise_alt)
        if self._dist_xy(*self.home) < self.wp_tol:
            self.get_logger().info("home reached — landing.")
            self.stream_setpoint = False
            self._enter(self.FINAL_LAND)
            self._set_mode("LAND")

    def _do_final_land(self):
        landed = (not self.mav_state.armed
                  or (self.pose is not None
                      and self.pose.pose.position.z < 0.25))
        if landed or self._since_entered() > self.land_timeout:
            self.get_logger().info(
                f"\n{'=' * 52}\n"
                f"  MISSION COMPLETE\n"
                f"  Pads landed on : {self.landed_count}\n"
                f"  Pads in the map: "
                f"{len(self.pad_map.pads) if self.pad_map else 0}\n"
                f"  Rejected leads : {len(self.blacklist) - self.landed_count}\n"
                f"  Elapsed        : "
                f"{self._now() - (self._mission_start or self._now()):.0f} s\n"
                f"{'=' * 52}")
            self._enter(self.DONE)

    # ────────────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────────────

    def _refresh_target(self):
        """Re-read the target's fused position from the map.

        The map keeps improving the estimate while we approach — especially once
        the down camera starts contributing — so the setpoint should follow it
        rather than freeze at whatever the first sighting said.
        """
        if self.pad_map is None or self.target_id is None:
            return
        for pad in self.pad_map.pads:
            if pad.id == self.target_id:
                self.target_xy = (pad.position.x, pad.position.y)
                self.target_height = pad.height
                return

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
        # Arrival is per-target; leaking it forward would time out the next
        # candidate the moment it is chosen.
        self._arrived_since = None

    def _since_entered(self) -> float:
        return self._now() - self._state_since

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _dist_xy(self, x: float, y: float) -> float:
        if self.pose is None:
            return float("inf")
        return math.hypot(self.pose.pose.position.x - x,
                          self.pose.pose.position.y - y)

    def _yaw(self) -> float:
        q = self.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _throttle(self, text: str):
        self.get_logger().info(text, throttle_duration_sec=5.0)

    def _publish_status(self):
        z = self.pose.pose.position.z if self.pose else float("nan")
        self.pub_status.publish(String(data=(
            f"state={self.state} mode={self.mav_state.mode} "
            f"armed={self.mav_state.armed} z={z:.2f} "
            f"wp={self.route_idx}/{len(self.route)} "
            f"target={self.target_id} landed={self.landed_count}")))


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
