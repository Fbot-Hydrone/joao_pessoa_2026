#!/usr/bin/env python3
"""
phase1_mission_node — spin, look, land, repeat.

The Phase 1 flight for the 5x5 m arena. The drone starts on a base in a corner,
and the sites it must land on are somewhere in the square with it. There is
nowhere to fly TO before looking, so this mission does not fly a pattern at all:
it takes off, turns on the spot, and only ever translates once it has something
to translate towards.

    arm  ->  register the base under us as the takeoff base
         ->  take off to takeoff_alt
         ->  is a confirmed, unvisited, non-takeoff base in the map?
                 yes -> fly over it -> confirm on the belly camera
                          confirmed -> LAND, mark visited, take off, repeat
                          not confirmed -> blacklist it, resume searching
                 no  -> turn 45 deg CW, settle, look again
                          8 turns with nothing -> the fallback
         ->  once `target_bases` landings are done: fly to the takeoff base,
             land on it, DONE.

State machine
-------------
    WAIT_FCU -> ARMING -> REGISTER -> TAKEOFF -> SELECT -+-> TRAVEL -> CONFIRM
                  ^                                      |              |
                  |                                      +-> ROTATE     |
                  |                                      |    ^   |     |
                  |                                      |    +-SETTLE  |
                  |                                      |              v
                  +---------------- DWELL <---------------------------- LAND
                                      |
                                      +-> DONE

Why turning instead of flying a pattern
---------------------------------------
Every metre flown is visual-odometry drift, and the arena gives the VO very
little to work with (docs/LANDING-SITES.md §10, and the ORB survey that found 46
keypoints in a whole frame). A 5x5 m square is small enough that a camera at
1 m sees all of it from one spot given enough headings, so the cheapest search
is the one that does not move the vehicle: turn, look, turn. The only
translation in the whole mission is the leg to a base the drone has already
decided to land on.

Why the pause matters
---------------------
Detection runs continuously — the detectors know nothing about this node — but
the mission only ACTS on detections that were taken while the vehicle was
stationary. A pad seen mid-turn is projected through a yaw estimate that is
still slewing, and lands in the map metres from the real thing. Yaw has to be
settled and held for `settle_s` before the map is read. That is also why the
pause is short: it is long enough for the estimate to stop moving, not long
enough to accumulate meaningful drift standing still.

Why the takeoff base is declared, not detected
----------------------------------------------
The base the drone starts on is a rectangle with a circular hole rather than the
disc-with-ring of a real landing site, but it carries the same colours and the
same cross, and from an oblique angle the detector does sometimes call it. Left
alone it would become a map candidate, and ruling it out would cost a travel leg
and a confirmation hover — drift spent on a question already answered. So the
instant the vehicle arms, its own position is registered as the takeoff base
(`RegisterTakeoffBase`), and pad_map_node refuses to map anything at all before
that arm. See docs/PHASE1-MISSION.md.

Speed
-----
There is no setpoint stepping here. One setpoint per leg, and the speed is the
FCU's business: WPNAV_SPEED / WPNAV_ACCEL bound the translation and ATC_SLEW_YAW
bounds the turn, all set in config/params/holybro_sitl.parm. Chopping a 2 m leg
into pieces would not make the vehicle gentler — the position controller already
accelerates against its own limits — it would only add arrival tests, each with
its own tolerance, on an estimate that is the least trustworthy thing in the
stack. Takeoff speed is deliberately NOT touched; the existing climb is fine.

Interfaces
----------
in:   /hydrone/pads/map           hydrone_msgs/PadMap        (what to fly to)
      /hydrone/pads/detections    hydrone_msgs/PadDetection  (down cam confirms)
      /mavros/state, /mavros/local_position/pose
out:  /mavros/setpoint_position/local
      /hydrone/mission/status     std_msgs/String
srv:  /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff
      /hydrone/pads/mark_visited, /hydrone/pads/register_takeoff_base
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
from hydrone_msgs.srv import MarkPadVisited, RegisterTakeoffBase


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


def wrap_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose: PoseStamped) -> float:
    """Yaw of a pose, ENU, CCW-positive from east."""
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Phase1MissionNode(Node):

    # ── States ──────────────────────────────────────────────────────────────
    WAIT_FCU = "WAIT_FCU"
    ARMING = "ARMING"
    REGISTER = "REGISTER"
    TAKEOFF = "TAKEOFF"
    SELECT = "SELECT"
    SETTLE = "SETTLE"
    ROTATE = "ROTATE"
    TRAVEL = "TRAVEL"
    CONFIRM = "CONFIRM"
    LAND = "LAND"
    DWELL = "DWELL"
    DONE = "DONE"
    ABORTED = "ABORTED"

    # ── Why we are landing. Decides what happens after the dwell. ───────────
    #   PAD      a confirmed landing site: count it, then go find the next one
    #   FALLBACK the search came up empty: touch down, take off, land, stop
    #   FINAL    the last landing of the run: stay down
    LAND_PAD = "pad"
    LAND_FALLBACK = "fallback"
    LAND_FINAL = "final"

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy's Node so tests can pass parameter_overrides;
        # the parameters here are read once, in __init__.
        super().__init__("phase1_mission", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        # Metres above the takeoff plane — which is the TOP of the base the
        # drone starts on, not the arena floor. 1 m keeps a test flight cheap to
        # crash; at that height the 320x240 / 90 deg belly camera covers a ~2 m
        # square of floor, so a 1 m base is in frame whenever the position error
        # is under about half a metre.
        self.declare_parameter("takeoff_alt", 1.0)
        # How many landing sites to visit before going home. The takeoff base is
        # NOT one of them.
        #
        # ONE, not the competition's two, while this mission has never been
        # flown: the first question is whether a single
        # find-confirm-land-take-off-return cycle closes at all, and a second
        # base only adds a leg on an estimate that has already been through a
        # landing and a takeoff. Raise it once one cycle has been watched end to
        # end. Kept in step with phase1.launch.py's default so `ros2 run` and
        # `ros2 launch` do not quietly disagree.
        self.declare_parameter("target_bases", 1)

        # ── The search ──────────────────────────────────────────────────────
        self.declare_parameter("rotation_step_deg", 45.0)
        # 8 x 45 deg = one full turn. Past that the drone is looking at scenery
        # it has already rejected.
        self.declare_parameter("max_rotations", 8)
        # Held stationary before the map is believed. Short on purpose: long
        # enough for the yaw estimate to stop moving, short enough not to spend
        # the flight hovering. Your ceiling was 2 s.
        self.declare_parameter("settle_s", 2.0)
        self.declare_parameter("yaw_tol_deg", 8.0)
        self.declare_parameter("rotate_timeout_s", 20.0)

        # ── Flying to a candidate ───────────────────────────────────────────
        self.declare_parameter("arrive_tol_m", 0.35)
        self.declare_parameter("travel_timeout_s", 60.0)

        # ── Confirming it on the belly camera ───────────────────────────────
        # The forward camera IDENTIFIES (it sees across the arena, at ranges
        # where the ring and cross are not resolvable and confidence is capped);
        # the down camera VALIDATES from directly above, where they are. A
        # candidate has to earn `confirm_detections` fresh looks above
        # `confirm_confidence` or it is not a landing site.
        self.declare_parameter("confirm_detections", 3)
        self.declare_parameter("confirm_confidence", 0.60)
        self.declare_parameter("confirm_timeout_s", 25.0)
        self.declare_parameter("fresh_detection_s", 1.0)

        # ── Landing, and the plumbing ───────────────────────────────────────
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
        self.takeoff_alt = float(p("takeoff_alt"))
        self.target_bases = int(p("target_bases"))
        self.rotation_step = math.radians(float(p("rotation_step_deg")))
        self.max_rotations = int(p("max_rotations"))
        self.settle_s = float(p("settle_s"))
        self.yaw_tol = math.radians(float(p("yaw_tol_deg")))
        self.rotate_timeout = float(p("rotate_timeout_s"))
        self.arrive_tol = float(p("arrive_tol_m"))
        self.travel_timeout = float(p("travel_timeout_s"))
        self.confirm_detections = int(p("confirm_detections"))
        self.confirm_conf = float(p("confirm_confidence"))
        self.confirm_timeout = float(p("confirm_timeout_s"))
        self.fresh_s = float(p("fresh_detection_s"))
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

        # Where we armed. The fallback for the return leg if the map somehow has
        # no takeoff-base entry.
        self.home: tuple[float, float] | None = None
        self.base_registered = False
        self.landed_count = 0
        # Candidates the belly camera refused. Mission-local on purpose: the map
        # is a record of what was SEEN, and a pad that failed confirmation was
        # genuinely seen. Deciding it is not worth a second visit is this
        # mission's judgement, so this mission keeps it.
        self.blacklist: set[int] = set()
        self.target_id: int | None = None
        self.rotations_done = 0
        self.landing_for = self.LAND_PAD
        # Set only by the fallback: the next takeoff exists to be followed by a
        # landing, not by a search.
        self._land_after_takeoff = False

        # [x, y, z, yaw] — yaw is commanded, not just carried: this mission
        # turns on the spot, and a setpoint with a fixed orientation would fight
        # the very thing the search is made of.
        self.setpoint: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.stream_setpoint = False
        self._settle_since: float | None = None
        self._call: _Call | None = None
        self._pending: str | None = None    # what _call is for
        self._last_cmd_t = 0.0              # when the last command went out
        self._takeoff_tries = 0
        # Down-camera looks accepted during the current CONFIRM.
        self._confirm_hits = 0
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
        # high", "PreArm: VisOdom: not healthy", ...). MAVROS publishes
        # statustext/recv BEST_EFFORT; a RELIABLE subscription is
        # QoS-incompatible and receives nothing at all.
        self.create_subscription(StatusText, "/mavros/statustext/recv",
                                 self._cb_statustext, sensor_qos)

        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.cli_visited = self.create_client(MarkPadVisited,
                                              "/hydrone/pads/mark_visited")
        self.cli_base = self.create_client(
            RegisterTakeoffBase, "/hydrone/pads/register_takeoff_base")

        self.create_service(Trigger, "/hydrone/mission/start", self._svc_start)
        self.create_service(Trigger, "/hydrone/mission/abort", self._svc_abort)

        self.create_timer(1.0 / max(float(p("setpoint_hz")), 1.0),
                          self._stream)
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"phase1_mission ready — takeoff to {self.takeoff_alt:.1f} m, "
            f"search by {math.degrees(self.rotation_step):.0f} deg turns "
            f"(max {self.max_rotations}), land on {self.target_bases} base(s), "
            "then home. "
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
        """Keep the freshest belly-camera look.

        Only the down camera is kept HERE, and only for confirmation. The
        forward camera is not ignored by the mission — it is the thing that
        finds bases in the first place — but it works through the map, which is
        where its many partial sightings get fused into one position. A raw
        forward frame is a lead, not a landing decision.
        """
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
        self.base_registered = False
        self.landed_count = 0
        self.blacklist.clear()
        self.target_id = None
        self.rotations_done = 0
        self.landing_for = self.LAND_PAD
        self._land_after_takeoff = False

    # ────────────────────────────────────────────────────────────────────────
    # Setpoint stream
    # ────────────────────────────────────────────────────────────────────────

    def _stream(self):
        """Publish the current position + yaw target.

        Deliberately silent whenever the FCU owns the descent (LAND, DWELL): a
        position setpoint arriving mid-landing is at best ignored and at worst
        fights the flare.
        """
        if not self.stream_setpoint:
            return
        x, y, z, yaw = self.setpoint
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation.z = math.sin(yaw / 2.0)
        sp.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_sp.publish(sp)

    def _goto(self, x: float, y: float, z: float, yaw: float):
        self.setpoint = [x, y, z, yaw]
        self.stream_setpoint = True

    def _hold(self, yaw: float | None = None):
        """Hold the current setpoint, optionally re-aiming the yaw."""
        if yaw is not None:
            self.setpoint[3] = yaw
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
            self.REGISTER: self._do_register,
            self.TAKEOFF: self._do_takeoff,
            self.SELECT: self._do_select,
            self.SETTLE: self._do_settle,
            self.ROTATE: self._do_rotate,
            self.TRAVEL: self._do_travel,
            self.CONFIRM: self._do_confirm,
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

        self.home = (self.pose.pose.position.x, self.pose.pose.position.y)
        self.get_logger().info(
            f"home = ({self.home[0]:.2f}, {self.home[1]:.2f}), heading "
            f"{math.degrees(yaw_of(self.pose)):.0f} deg.")
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
            self._enter(self.REGISTER if not self.base_registered
                        else self.TAKEOFF)
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

    # ── REGISTER ─────────────────────────────────────────────────────────────

    def _do_register(self):
        """Declare the base under us, once, before we ever leave it.

        This runs between arming and takeoff because that is the only moment the
        drone's position IS the base's position, to the centimetre, with no
        camera in the loop. pad_map_node maps nothing before this point, so the
        takeoff base is guaranteed to be the first entry in the map rather than
        something that has to be reconciled with an earlier sighting.

        A failure here is not fatal. The mission still knows `home` and still
        refuses to land on anything within `arrive_tol` of it, so the worst case
        is a takeoff base that RViz draws as an ordinary pad.
        """
        if self.base_registered or self.pose is None:
            if self.base_registered:
                self._enter(self.TAKEOFF)
            return

        status = self._poll_call()
        if status == "pending":
            return
        if status == "ok":
            self.base_registered = True
            self.get_logger().info("takeoff base registered.")
            self._enter(self.TAKEOFF)
            return

        if not self.cli_base.service_is_ready():
            if self._since_entered() > 5.0:
                self.get_logger().warn(
                    "/hydrone/pads/register_takeoff_base never came up — "
                    "flying without a registered takeoff base. The map may "
                    "offer the start base as a candidate; the mission will "
                    "still refuse to land on anything at home.")
                self.base_registered = True
                self._enter(self.TAKEOFF)
            return

        if self._now() - self._last_cmd_t >= self.retry_period:
            req = RegisterTakeoffBase.Request()
            req.position.x = float(self.pose.pose.position.x)
            req.position.y = float(self.pose.pose.position.y)
            req.position.z = float(self.pose.pose.position.z)
            self._start_call("register", self.cli_base, req)

    # ── TAKEOFF ──────────────────────────────────────────────────────────────

    def _do_takeoff(self):
        """Ask the FCU to climb to takeoff_alt.

        ArduCopter will not climb from a bare position setpoint in GUIDED — it
        needs an explicit takeoff — so this is a command, not a setpoint, and the
        setpoint stream only starts once we are up. The first setpoint holds the
        position and heading we reached, so nothing moves at the handover.
        """
        if (self.pose is not None
                and self.pose.pose.position.z >= self.takeoff_alt - 0.15):
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            yaw = yaw_of(self.pose)
            self.get_logger().info(
                f"airborne at {self.pose.pose.position.z:.2f} m, "
                f"heading {math.degrees(yaw):.0f} deg.")
            self._goto(x, y, self.takeoff_alt, yaw)
            self.rotations_done = 0
            if self._land_after_takeoff:
                # The fallback's second hop: up, then straight back down.
                self._land_after_takeoff = False
                self.get_logger().info(
                    "fallback hop complete — landing to end the run.")
                self._begin_landing()
                return
            self._enter(self.SELECT)
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
            req.altitude = float(self.takeoff_alt)
            self._start_call("takeoff", self.cli_takeoff, req)

    # ── SELECT ───────────────────────────────────────────────────────────────

    def _do_select(self):
        """Decide what to do with the air we are holding.

        Quota first, then a known lead, then the search. Checking the quota here
        rather than after each landing means the "go home" decision is made in
        exactly one place, and it is made while airborne with the map in hand.
        """
        if self.pose is None:
            return

        if self.landed_count >= self.target_bases:
            hx, hy = self._takeoff_base_xy()
            self.get_logger().info(
                f"{self.landed_count} base(s) landed on — returning to the "
                f"takeoff base at ({hx:.2f}, {hy:.2f}).")
            self.target_id = None
            self.landing_for = self.LAND_FINAL
            self._goto(hx, hy, self.takeoff_alt, self.setpoint[3])
            self._enter(self.TRAVEL)
            return

        pad = self._best_candidate()
        if pad is not None:
            self.target_id = int(pad.id)
            self.landing_for = self.LAND_PAD
            self.get_logger().info(
                f"pad {pad.id} at ({pad.position.x:.2f}, {pad.position.y:.2f}) "
                f"is confirmed in the map ({pad.observations} looks, conf "
                f"{pad.confidence:.2f}) — flying over it.")
            self._goto(pad.position.x, pad.position.y, self.takeoff_alt,
                       self.setpoint[3])
            self._enter(self.TRAVEL)
            return

        self._enter(self.SETTLE)

    def _best_candidate(self):
        """The nearest pad worth flying to, or None.

        Worth flying to = confirmed by the map (three fused sightings, so not
        one frame of noise), not already landed on, not the base we took off
        from, and not one the belly camera has already refused. Nearest wins:
        in a 5x5 m arena the differences are small, and the shortest leg is the
        least drift.
        """
        if self.pad_map is None or self.pose is None:
            return None
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        best, best_d = None, float("inf")
        for pad in self.pad_map.pads:
            if not self._is_candidate(pad):
                continue
            d = math.hypot(pad.position.x - px, pad.position.y - py)
            if d < best_d:
                best, best_d = pad, d
        return best

    def _is_candidate(self, pad) -> bool:
        if pad.is_takeoff_base or pad.visited:
            return False
        if int(pad.id) in self.blacklist:
            return False
        if pad.observations < 3:
            return False
        # Belt and braces for the case where registration failed: never treat
        # anything sitting where we armed as a landing site.
        if self.home is not None:
            if math.hypot(pad.position.x - self.home[0],
                          pad.position.y - self.home[1]) < 1.0:
                return False
        return True

    def _takeoff_base_xy(self) -> tuple[float, float]:
        """Where home is. The map's registered entry if there is one, else the
        position we armed at."""
        if self.pad_map is not None:
            for pad in self.pad_map.pads:
                if pad.is_takeoff_base:
                    return (pad.position.x, pad.position.y)
        return self.home if self.home is not None else (0.0, 0.0)

    # ── SETTLE ───────────────────────────────────────────────────────────────

    def _do_settle(self):
        """Hold still, let the estimate stop moving, then read the map.

        The map is NOT read while this is counting down. That is the whole point
        of the state: a detection taken while yaw was still slewing is projected
        through a moving estimate, and the position it produces is wrong by
        metres. Waiting costs two seconds and buys a map entry that means what
        it says.
        """
        self._hold()
        if self._since_entered() < self.settle_s:
            return

        pad = self._best_candidate()
        if pad is not None:
            self.rotations_done = 0
            self._enter(self.SELECT)
            return

        if self.rotations_done >= self.max_rotations:
            self.get_logger().warn(
                f"{self.rotations_done} turns and no new base in sight — "
                "falling back: landing, taking off once, landing again.")
            self.landing_for = self.LAND_FALLBACK
            self._begin_landing()
            return

        # Aim the next turn here rather than inside ROTATE, so ROTATE is a pure
        # "are we there yet" and has no first-tick special case to get wrong.
        self._hold(yaw=wrap_pi(self.setpoint[3] - self.rotation_step))
        self.get_logger().info(
            f"turn {self.rotations_done + 1}/{self.max_rotations}: "
            f"heading for {math.degrees(self.setpoint[3]):.0f} deg.")
        self._enter(self.ROTATE)

    # ── ROTATE ───────────────────────────────────────────────────────────────

    def _do_rotate(self):
        """Turn one step clockwise, on the spot.

        Clockwise is NEGATIVE yaw: the map frame is ENU and yaw runs
        counter-clockwise from east, so a clockwise turn subtracts. The x/y/z of
        the setpoint do not change — the FCU holds position while it yaws, and
        the turn rate is ATC_SLEW_YAW's business, not this node's.
        """
        if self.pose is None:
            return
        self._hold()

        error = abs(wrap_pi(yaw_of(self.pose) - self.setpoint[3]))
        if error <= self.yaw_tol:
            self.rotations_done += 1
            self._enter(self.SETTLE)
            return

        if self._since_entered() > self.rotate_timeout:
            self.get_logger().warn(
                f"yaw still {math.degrees(error):.0f} deg off after "
                f"{self.rotate_timeout:.0f} s — counting the turn anyway and "
                "looking from here.")
            self.rotations_done += 1
            self._enter(self.SETTLE)

    # ── TRAVEL ───────────────────────────────────────────────────────────────

    def _do_travel(self):
        """Fly to the setpoint SELECT placed. One leg, one setpoint.

        Heading is untouched on the way: turning while translating would put the
        detector's geometry and the position controller's demand in motion at
        the same time, and there is nothing to gain — the belly camera looks
        straight down and does not care which way the nose points.
        """
        if self.pose is None:
            return
        self._hold()

        d = math.hypot(self.pose.pose.position.x - self.setpoint[0],
                       self.pose.pose.position.y - self.setpoint[1])
        if d <= self.arrive_tol:
            if self.landing_for == self.LAND_FINAL:
                self.get_logger().info(
                    "over the takeoff base — landing to finish the run.")
                self._begin_landing()
            else:
                self.get_logger().info(
                    f"over pad {self.target_id} — confirming on the belly "
                    "camera.")
                self._confirm_hits = 0
                self._enter(self.CONFIRM)
            return

        if self._since_entered() > self.travel_timeout:
            if self.landing_for == self.LAND_FINAL:
                self.get_logger().warn(
                    f"could not reach the takeoff base in "
                    f"{self.travel_timeout:.0f} s ({d:.2f} m short) — landing "
                    "where we are rather than pushing a setpoint we cannot "
                    "hold.")
                self._begin_landing()
                return
            self.get_logger().warn(
                f"could not reach pad {self.target_id} in "
                f"{self.travel_timeout:.0f} s ({d:.2f} m short) — blacklisting "
                "it and searching again.")
            self._reject_target()

    # ── CONFIRM ──────────────────────────────────────────────────────────────

    def _do_confirm(self):
        """Prove the thing below us really is a landing site.

        The forward camera found it across the arena, where the ring and the
        cross are a handful of pixels and the detector's confidence is capped by
        design (docs/LANDING-SITES.md §3). From directly above at 1 m the same
        structure is hundreds of pixels across, so this is the look that decides.
        `confirm_detections` separate frames must clear `confirm_confidence` —
        one frame can be a glint on something blue.

        A candidate that cannot manage that inside `confirm_timeout` is
        blacklisted, and the search resumes from here. That is what makes a blue
        tarp cost half a minute instead of the mission.
        """
        self._hold()

        fresh = (self._last_down is not None
                 and self._now() - self._last_down_t <= self.fresh_s)
        if fresh and self._last_down.confidence >= self.confirm_conf:
            self._confirm_hits += 1
            # One detection must not be counted twice: the belly camera runs at
            # 10 Hz and this tick at 10 Hz, so without clearing it a single
            # frame would satisfy the whole quota on its own.
            self._last_down = None
            if self._confirm_hits >= self.confirm_detections:
                self.get_logger().info(
                    f"pad {self.target_id} CONFIRMED on the belly camera "
                    f"({self._confirm_hits} looks) — landing.")
                self._begin_landing()
                return

        if self._since_entered() > self.confirm_timeout:
            self.get_logger().warn(
                f"pad {self.target_id} did not confirm in "
                f"{self.confirm_timeout:.0f} s ({self._confirm_hits}/"
                f"{self.confirm_detections} looks) — not a landing site. "
                "Blacklisting it and searching from here.")
            self._reject_target()

    def _reject_target(self):
        """Give up on the current candidate and go back to turning."""
        if self.target_id is not None:
            self.blacklist.add(int(self.target_id))
        self.target_id = None
        # A fresh search, not a continuation: the drone is somewhere new, facing
        # a direction it has not searched from, so the turns it already made
        # tell us nothing about what is visible from here.
        self.rotations_done = 0
        if self.pose is not None:
            self._goto(self.pose.pose.position.x, self.pose.pose.position.y,
                       self.takeoff_alt, self.setpoint[3])
        self._enter(self.SETTLE)

    # ── LAND ─────────────────────────────────────────────────────────────────

    def _begin_landing(self):
        """Hand the descent to the FCU and stop talking to it.

        The setpoint stream stops here. ArduPilot's LAND has a rangefinder flare
        and it owns the vehicle from this point; a position setpoint arriving
        mid-descent is at best ignored and at worst fights it.
        """
        self.stream_setpoint = False
        self._settle_since = None
        self._enter(self.LAND)
        self._set_mode("LAND")

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

        # Landed = the FCU disarmed us, or we are sitting near the takeoff
        # plane. Everything in this arena is at ground level for now, so 0.5 m
        # is comfortably below any hover and above any resting altitude.
        disarmed = not self.mav_state.armed
        low = self.pose is not None and self.pose.pose.position.z <= 0.5

        if disarmed or low:
            if self._settle_since is None:
                self._settle_since = self._now()
            elif self._now() - self._settle_since >= self.land_settle:
                z = self.pose.pose.position.z if self.pose else 0.0
                if self.landing_for == self.LAND_PAD:
                    self.landed_count += 1
                    self.get_logger().info(
                        f"LANDED on base #{self.landed_count} of "
                        f"{self.target_bases} — resting at z={z:.2f} m.")
                    self._mark_visited(z)
                else:
                    self.get_logger().info(
                        f"LANDED ({self.landing_for}) at z={z:.2f} m.")
                self._enter(self.DWELL)
        else:
            self._settle_since = None

        if self._since_entered() > self.land_timeout:
            self.get_logger().warn(
                f"no touchdown within {self.land_timeout:.0f} s — carrying on "
                "anyway so the mission does not stall here.")
            self._enter(self.DWELL)

    def _mark_visited(self, height: float):
        """Tell the pad map we landed, so it stops offering this pad."""
        pad_id = self.target_id if self.target_id is not None \
            else self._pad_id_below()
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
        """Sit on the pad, then decide whether there is more flying to do."""
        if self._since_entered() < self.dwell_s:
            return

        if self.landing_for == self.LAND_FINAL:
            self.get_logger().info(
                f"mission complete — {self.landed_count} base(s) landed on, "
                "home on the takeoff base.")
            self._enter(self.DONE)
            return

        if self.landing_for == self.LAND_FALLBACK:
            # The agreed fallback: touch down, take off once, land again where
            # we are, stop. No leg home — the whole reason we are here is that
            # the position estimate has stopped being worth flying on, and a
            # cross-arena leg is the last thing to attempt on it.
            self.get_logger().info(
                "fallback: taking off once more, then landing in place to end "
                "the run.")
            self.landing_for = self.LAND_FINAL
            self._land_after_takeoff = True
            self._takeoff_tries = 0
            self._enter(self.ARMING)
            return

        self._takeoff_tries = 0
        self.target_id = None
        self.get_logger().info(
            f"taking off again — {self.landed_count}/{self.target_bases} "
            "base(s) done.")
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
        x = self.pose.pose.position.x if self.pose else float("nan")
        y = self.pose.pose.position.y if self.pose else float("nan")
        z = self.pose.pose.position.z if self.pose else float("nan")
        yaw = math.degrees(yaw_of(self.pose)) if self.pose else float("nan")
        self.pub_status.publish(String(data=(
            f"state={self.state} mode={self.mav_state.mode} "
            f"armed={self.mav_state.armed} "
            f"x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.0f} "
            f"landed={self.landed_count}/{self.target_bases} "
            f"turns={self.rotations_done}/{self.max_rotations} "
            f"target={self.target_id} blacklisted={sorted(self.blacklist)}")))


def main(args=None):
    rclpy.init(args=args)
    node = Phase1MissionNode()
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
