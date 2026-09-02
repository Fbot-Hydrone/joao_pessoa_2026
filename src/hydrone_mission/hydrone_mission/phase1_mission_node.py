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
      /hydrone/pads/down/…        hydrone_msgs/PadDetection  (down cam confirms)
      /mavros/state, /mavros/local_position/pose
out:  /mavros/setpoint_position/local
      /hydrone/mission/status     std_msgs/String
srv:  /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff
      /hydrone/pads/mark_visited, /hydrone/pads/register_takeoff_base

Dry run — `dry_run:=true`, and phase1_dry.launch.py
---------------------------------------------------
The same state machine, the same map, the same belly camera, and NOTHING SENT
TO THE FLIGHT CONTROLLER. A human carries the drone and is the actuator: the
node prints `>>> RAISE …`, `>>> TURN … 45 deg`, `>>> CARRY … 2.4 m`, `>>> PUT
IT DOWN`, and every transition still waits on the MEASURED pose, so the
rehearsal advances only when the drone has physically been moved. It exists
because the mission is worth debugging in the arena, against the real pads and
the real estimate, without motors — a spinning propeller in someone's hands is
the one failure this whole file must never produce.

What makes that structural rather than a promise: the arm, mode and takeoff
CLIENTS and the setpoint PUBLISHER are never created (see the I/O section), so
no path through this node reaches the FCU — including one added later by
someone who did not read this. `_start_call`, `_set_mode` and `_stream` all
refuse on None, so a forgotten guard is a log line rather than a command.

This is a guarantee about THIS STACK, not about the vehicle. MAVROS runs
normally, /mavros/cmd/arming still exists for anyone who types one, and the
transmitter never went through MAVROS at all. Arming is settled at the flight
controller: set an arming check the vehicle cannot pass before a rehearsal, and
take the props off. `_dry_audit` checks the one thing it usefully can — that
nothing ELSE in the graph is publishing setpoints, which is what a forgotten
phase1_real in another terminal looks like.

What still happens, because it is the point: the pretend-arm registers the
takeoff base, that opens pad_map_node's gate, and the map, its markers and the
feature map build from there exactly as they would in flight.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from octomap_msgs.msg import Octomap

from hydrone_map import octree
from hydrone_nav import coverage, planner, route, servo
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
    TRAVEL = "TRAVEL"
    CONFIRM = "CONFIRM"
    LAND = "LAND"
    DWELL = "DWELL"
    DONE = "DONE"
    ABORTED = "ABORTED"

    # ── Why we are landing. Decides what happens after the dwell. ───────────
    #   PAD      a confirmed landing site: count it, then go find the next one
    #   FINAL    the last landing of the run: stay down
    #
    # There used to be a FALLBACK: when the search came up empty the vehicle
    # touched down WHERE IT WAS. That is an off-base landing, which is
    # eliminatory, so an exhausted search now flies home like any other ending.
    LAND_PAD = "pad"
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
        # 8 x 45 deg = one full turn. Past that the drone is looking at scenery
        # it has already rejected.
        # Held stationary before the map is believed. Short on purpose: long
        # enough for the yaw estimate to stop moving, short enough not to spend
        # the flight hovering. Your ceiling was 2 s.
        self.declare_parameter("settle_s", 2.0)
        self.declare_parameter("yaw_tol_deg", 8.0)
        self.declare_parameter("rotate_timeout_s", 20.0)

        # ── Flying to a candidate ───────────────────────────────────────────
        self.declare_parameter("arrive_tol_m", 0.35)

        # ── Confirming it on the belly camera ───────────────────────────────
        # The forward camera IDENTIFIES (it sees across the arena, at ranges
        # where the ring and cross are not resolvable and confidence is capped);
        # the down camera VALIDATES from directly above, where they are. A
        # candidate has to earn `confirm_detections` fresh looks above
        # `confirm_confidence` or it is not a landing site.
        # The belly camera's detection topic. Its own, separate from the
        # /hydrone/pads/detections bus pad_map_node fuses: the down camera does
        # not project (competition bases are raised, and a flat-floor cast from
        # overhead lands past the pad), so its detections have no position and
        # have no business in the map.
        self.declare_parameter("detections_topic",
                               "/hydrone/pads/down/detections")
        self.declare_parameter("confirm_detections", 3)
        self.declare_parameter("confirm_confidence", 0.60)
        self.declare_parameter("confirm_timeout_s", 25.0)
        # How long a leg may go without getting closer before its target is
        # written off. Generous on purpose: this must never fire on a leg that
        # is merely slow, only on one that has stopped closing, and the removed
        # 60 s budget is the cautionary tale (see _do_travel). At the mission's
        # cruise a real approach improves by centimetres every tick, so 20 s of
        # NO improvement at all is already far outside normal.
        self.declare_parameter("travel_stall_s", 20.0)
        # How much closer counts as progress. Above the position estimate's
        # own jitter, so hovering noise cannot masquerade as an approach and
        # keep a dead leg alive forever.
        self.declare_parameter("travel_progress_m", 0.05)
        # Coverage search: when a full sweep from one spot finds nothing, fly
        # to where the occupancy map still has unobserved space instead of
        # giving up. Off restores the pure turn-in-place search.
        self.declare_parameter("coverage_search", True)
        # How finely coverage is COUNTED, and how finely candidate positions
        # are TRIED. Two grids on purpose: making the second as fine as the
        # first squares the work for viewpoints that differ by less than the
        # vehicle's own position error.
        self.declare_parameter("coverage_viewpoint_m", 1.0)
        # How far a viewpoint is credited with seeing. SHORTER than the
        # detector's true reach on purpose: a base at the far edge of the frame
        # is a handful of pixels, and counting it would let the search tick off
        # arena it only technically looked at.
        self.declare_parameter("coverage_range_m", 5.0)
        # Fewer newly visible cells than this does not pay for the drift the
        # trip costs.
        self.declare_parameter("coverage_min_gain", 4)
        # Turn isolated relief in the occupancy map into weak candidates to
        # The survey cannot run for ever. Phase 1 allows 3 attempts in 30
        # minutes, so a sweep that eats the attempt has cost the run whatever
        # it learned. These are the two ways it ends other than running out of
        # unseen arena.
        # Point the camera where the map is still unobserved, instead of
        # turning a blind full circle. False restores the old sweep.
        # Sweep the arena by flying a rectangle rather than by spinning.
        self.declare_parameter("survey_circuit", True)
        # How far the circuit is inset from the arena bounds. Far enough that
        # the drone is not skimming the wall, close enough that the camera
        # still reaches the far side.
        self.declare_parameter("survey_inset_m", 1.2)
        # Waypoint spacing along a LAWNMOWER lane (search level 4 only). The U
        # has no intermediate points: a leg is a straight line on one heading,
        # so a point in the middle of it does nothing but tell the vehicle to
        # stop, and GUIDED stops dead at every position target.
        # THE LENGTH OF THE U'S LEGS, in metres, stated outright. Zero means
        # derive it from the arena: leg = arena_size - 2 * survey_inset_m.
        # Set it when the sweep should be a particular size for a reason the
        # arena dimensions do not express — a smaller rectangle in a big hall,
        # or a shape matched to what the camera actually reaches.
        self.declare_parameter("u_side_x_m", 0.0)
        self.declare_parameter("u_side_y_m", 0.0)
        # Height of the sweep. ABOVE THE HOUSE (1.5 m roof in the competition
        # arena) and below the net at 2.5 m — the passes fly over it, and at
        # the 1 m cruise height they would fly INTO it. Higher also widens what
        # each pass sees.
        # Altura da hover de CONFIRMACAO, separada do takeoff_alt.
        #
        # _do_confirm foi desenhado para "directly above at 1 m", onde o anel e
        # a cruz ocupam centenas de pixels. Com takeoff_alt = 2.5 m a confirmacao
        # herdava 2.5 m e o pad caia para ~128 px. MEDIDO 2026-09-01: um pad real
        # a 0.44 m do ponto sobrevoado nao produziu UM frame de barriga em 25 s e
        # foi para a blacklist.
        #
        # 1.5 m dobra o pad em pixels e a pegada da camera (FOV 90) ainda e ~3 m,
        # entao um erro de meio metro continua dentro do quadro. Descer mais
        # aumentaria o pad mas encolheria a pegada abaixo do erro que a projecao
        # comete — e ai o pad sai de cena.
        # How far the map has to move a pad before the leg re-aims at it.
        # Below this it is fusion noise and chasing it would re-issue a setpoint
        # every tick; above it the vehicle is flying to the wrong place.
        # How many times a pad may be refused by the planner before it is
        # written off. One refusal is about the MAP, not about the pad.
        # Interrupt a search level to land on a pad that is already confirmed,
        # instead of flying the level to its end first. Level 1 is always flown
        # whole — that sweep is what stops the mission chasing its first
        # sighting.
        self.declare_parameter("land_during_survey", True)
        self.declare_parameter("unreachable_tries", 3)
        # Seconds a refused pad must wait before another refusal counts against
        # it. Long enough for the search to fly somewhere else and put new rays
        # into the octomap.
        self.declare_parameter("retarget_tol_m", 0.25)
        # How far ABOVE THE PAD'S TOP the confirmation hover sits. This is the
        # number that decides how big the pad is in the belly frame, and it is
        # the same for every pad regardless of how tall the base is.
        # Floor and ceiling for that hover, as absolute altitudes. The ceiling
        # keeps a tall base from pushing the vehicle into the arena net.
        # LEVEL 2 is the same U this much higher. Half a metre changes what the
        # camera can see over and past without changing the flight.
        # LEVEL 4's lane spacing. Tighter is more thorough and costs
        # proportionally more flight.
        # How far the ladder is allowed to climb. 4 spends everything; lower it
        # to cap what an attempt may cost.
        self.declare_parameter("max_search_level", 2)
        # Centre the vehicle over the pad during the confirmation hover, using
        # the belly camera. The pixel-to-metre mapping is learned in flight;
        # see hydrone_nav.servo for why it cannot be a constant.
        self.declare_parameter("centre_on_pad", True)
        # Where the pad should sit in the belly image. The image centre unless
        # the lens is off-centre on the airframe — which the servo CANNOT
        # learn, because it is what "centred" means. Measure it once by
        # hovering over a known pad and reading where it lands in frame.
        self.declare_parameter("pad_target_uv", [320.0, 240.0])
        self.declare_parameter("survey_max_stalls", 2)
        # How much the predicted gain must fall for a trip to count as
        # learning something.
        self.declare_parameter("survey_progress_cells", 5)
        # Deliberately BELOW the detector's own floor: relief is a reason to go
        # look, not a sighting of a pad, and it must not outvote the camera in
        # the map's fusion.
        # Where the arena FLOOR is in this frame. The map's origin is the top
        # of the base the drone armed on, not the ground, so the floor sits
        # BELOW zero. relief's band is measured from here.
        self.declare_parameter("ground_z", -0.7)
        # Half-width of the ARENA, m. NOT `plan_bounds`, which is deliberately
        # a metre slacker on every side so the planner has room to round a
        # corner. Relief needs the real thing: its wall margin is measured from
        # the boundary it is given, and measured from +-5 the band it trims
        # lands OUTSIDE the arena while the actual wall at +-4 stays in and
        # fuses every base to it. MEASURED: 285 cells, 181 of them one cluster,
        # every group then rejected for touching the wall, 0 candidates.
        # Measured from the arena edge. 0.4 m clears the wall voxels at +-4.0
        # without eating a base: bases spawn at most ~3.5 m out, because the
        # spawner keeps half a base clear of the wall.
        # Two relief hits this close are the same lump seen twice.
        # Regions the relief scan ignores, flattened [min_x, min_y, max_x,
        # max_y, ...]. The house by default: it is known, it is big, and it
        # would otherwise dominate every cluster. From config.yaml's `house`.
        # The house, as (min_x, min_y, max_x, max_y) IN THE MAP FRAME. It was
        # written in the simulator's world coordinates, and the map is that
        # world turned 90 deg — `map = (-y_world, x_world)`, confirmed against
        # the takeoff base and every spawned base. So the box excluded a patch
        # of open arena while the house itself, 1.5 m tall and squarely inside
        # the height band, stayed in and helped fuse every cluster into one.
        # World x in [-4, 2], y in [2, 4]  ->  map x in [-4, -2], y in [-4, 2].
        # Where the planned route is published for RViz. Informational only —
        # nothing in the mission reads it back.
        self.declare_parameter("out_plan", "/hydrone/nav/plan")
        self.declare_parameter("fresh_detection_s", 1.0)

        # ── Landing, and the plumbing ───────────────────────────────────────
        self.declare_parameter("dwell_s", 4.0)
        self.declare_parameter("land_timeout_s", 60.0)
        self.declare_parameter("land_settle_s", 2.0)
        # Touchdown = the reported altitude STOPS CHANGING. How much movement
        # still counts as stopped, m, measured peak-to-peak over land_settle_s.
        # A descending vehicle covers far more than this in that window (even a
        # slow 0.05 m/s descent moves 0.10 m in 2 s), a resting one covers only
        # estimator noise. Raise it if a landing is never declared; lower it if
        # one is declared while still visibly descending.
        self.declare_parameter("land_still_tol_m", 0.05)
        # Stillness ALONE is not touchdown: a hover is perfectly still too, and
        # between the LAND mode command and ArduPilot actually starting the
        # descent there is a gap long enough to fill the window. On 2026-08-23
        # that gap declared LANDED 2.1 s after CONFIRM, at z=1.50 m — the full
        # hover altitude, having descended nothing — and then wrote that 1.50 m
        # into the pad's height as if the drone were resting on it.
        #
        # So the vehicle must also have GONE DOWN. The confirmation hover sits
        # takeoff_alt above the pad, so a real landing covers far more than
        # this; a hover covers none of it.
        self.declare_parameter("min_descent_m", 0.30)
        self.declare_parameter("takeoff_timeout_s", 45.0)
        self.declare_parameter("service_timeout_s", 30.0)
        self.declare_parameter("setpoint_hz", 10.0)
        # The box the planner may search in, [min_x, min_y, min_z, max_x,
        # max_y, max_z] in the pose frame. Worth setting: without bounds a
        # search that cannot reach its goal expands outwards through open
        # space until the expansion cap stops it, which costs a second of
        # nothing at the moment a leg begins. The default is the competition
        # arena with a metre of slack, floored above the landing pads and
        # ceilinged at the net.
        self.declare_parameter("plan_bounds",
                               [-5.0, -5.0, 0.3, 5.0, 5.0, 2.5])
        # Whether a plan may cross space no ray has reached. FALSE.
        #
        # This was true, on the argument that the fallback — the straight line
        # this mission always flew — crosses unknown space without asking, so a
        # plan that at least avoids the KNOWN obstacles was strictly safer.
        # That argument was wrong in practice and the drone hit a wall.
        #
        # What it missed: a straight line to a pad is short and aimed at
        # somewhere the camera has been looking. A PLANNED path is free to
        # detour anywhere the search can reach, and with unknown space
        # traversable the cheapest detour is very often straight through the
        # part of the arena nothing has mapped — which is where the walls the
        # drone has not seen yet are. Permission to plan through the unmapped
        # is not the same risk as flying a short straight line through it.
        # TRUE for Phase 1, which is what the comment in _goto_via_map has
        # always said this mission does — and it was never actually set.
        #
        # With False, `unknown` is impassable, and the planner refuses a goal
        # whose own cell has never been hit by a ray. In an open arena most
        # voxels are exactly that: never measured, because nothing ever looked
        # there. MEASURED 2026-09-02: a real base, CONFIRMED at 0.75, reported
        # "no way round it exists in the map" three times in an EMPTY 8x8 m
        # arena and was blacklisted. Nothing was in the way. The space over it
        # had simply never been rayed.
        #
        # The fallback when planning fails is the straight line, which crosses
        # unknown space without asking anything — so refusing to plan through
        # unknown does not make the flight safer, it just replaces a path that
        # avoids KNOWN obstacles with one that ignores them.
        #
        # Phase 4's confined space is where this should be False, and where the
        # map is dense enough to afford it.
        self.declare_parameter("plan_allow_unknown", True)
        self.declare_parameter("auto_start", True)

        # ── DRY RUN: the vehicle never arms, a human is the actuator ────────
        # The whole state machine runs, against the REAL map, the REAL pose and
        # the REAL belly camera — but nothing is ever sent to the FCU. Every
        # command becomes an instruction to the person holding the drone, and
        # every transition still waits on the measured pose, so the rehearsal
        # only advances when that person actually raises, turns, carries or sets
        # the drone down.
        #
        # This is NOT a soft switch. In dry run the arm, mode and takeoff
        # service clients and the setpoint publisher are never CREATED, so
        # there is no object for a bug in the state machine to send through.
        # It stops THIS STACK commanding the vehicle; it does not stop the
        # vehicle arming — that belongs to the flight controller, as an arming
        # check it cannot pass.
        self.declare_parameter("dry_run", False)
        # How long to sit on the takeoff base before the rehearsal treats the
        # vehicle as armed. It is the moment the takeoff base is registered and
        # the map starts accepting detections, exactly as a real arm is, so it
        # wants to be long enough for the person to have the drone still and
        # square on the base.
        self.declare_parameter("dry_arm_delay_s", 3.0)
        # Gap between repeats of a mode/arm/takeoff command. MAVROS acking a
        # command is not the same as ArduPilot accepting it, so every command is
        # re-sent on this period until /mavros/state shows the effect.
        self.declare_parameter("retry_period_s", 2.0)

        p = lambda n: self.get_parameter(n).value
        self.takeoff_alt = float(p("takeoff_alt"))
        self.target_bases = int(p("target_bases"))
        self.settle_s = float(p("settle_s"))
        self.yaw_tol = math.radians(float(p("yaw_tol_deg")))
        self.rotate_timeout = float(p("rotate_timeout_s"))
        self.arrive_tol = float(p("arrive_tol_m"))
        self.confirm_detections = int(p("confirm_detections"))
        self.confirm_conf = float(p("confirm_confidence"))
        self.confirm_timeout = float(p("confirm_timeout_s"))
        self.travel_stall_s = float(p("travel_stall_s"))
        self.travel_progress_m = float(p("travel_progress_m"))
        self.coverage_search = bool(p("coverage_search"))
        self.coverage_viewpoint_m = float(p("coverage_viewpoint_m"))
        self.coverage_range_m = float(p("coverage_range_m"))
        self.coverage_min_gain = int(p("coverage_min_gain"))
        self.survey_circuit = bool(p("survey_circuit"))
        self.survey_inset_m = float(p("survey_inset_m"))
        self.u_side_x_m = float(p("u_side_x_m"))
        self.u_side_y_m = float(p("u_side_y_m"))
        self.land_during_survey = bool(p("land_during_survey"))
        self.retarget_tol_m = float(p("retarget_tol_m"))
        self.max_search_level = int(p("max_search_level"))
        self.centre_on_pad = bool(p("centre_on_pad"))
        t = [float(v) for v in p("pad_target_uv")]
        self._servo = servo.VisualServo(target_uv=(t[0], t[1]))
        self._level = 1
        self._survey_path = None
        self.survey_max_stalls = int(p("survey_max_stalls"))
        self.survey_progress_cells = int(p("survey_progress_cells"))
        self._survey_visits = 0
        self._survey_stalls = 0
        self._survey_last_gain = None
        self._survey_path = None
        self.ground_z = float(p("ground_z"))
        # found during the sweep is remembered, not flown to.
        self.survey_done = False
        # True while working through leads the search could not confirm.
        self.investigating = False
        self.fresh_s = float(p("fresh_detection_s"))
        self.dwell_s = float(p("dwell_s"))
        self.land_timeout = float(p("land_timeout_s"))
        self.land_settle = float(p("land_settle_s"))
        self.land_still_tol = float(p("land_still_tol_m"))
        self.min_descent = float(p("min_descent_m"))
        self.takeoff_timeout = float(p("takeoff_timeout_s"))
        self.svc_timeout = float(p("service_timeout_s"))
        self.auto_start = bool(p("auto_start"))
        self.retry_period = float(p("retry_period_s"))
        self.dry_run = bool(p("dry_run"))
        self.dry_arm_delay = float(p("dry_arm_delay_s"))

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
        self.landing_for = self.LAND_PAD
        # Set only by the fallback: the next takeoff exists to be followed by a
        # landing, not by a search.
        self._land_after_takeoff = False

        # [x, y, z, yaw] — yaw is commanded, not just carried: this mission
        # turns on the spot, and a setpoint with a fixed orientation would fight
        # the very thing the search is made of.
        self.setpoint: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.stream_setpoint = False
        # Waypoints still to fly on the current leg, [] for a straight one.
        # See _goto_via_map: a straight leg is one setpoint and this stays
        # empty, which is exactly what every leg was before there was a
        # planner.
        self._leg: list[tuple[float, float, float, float]] = []
        # Closest this leg has come, and when it last improved. See _do_travel.
        self._travel_best: float | None = None
        self._travel_progress_t = 0.0
        # Is the current leg going somewhere to LOOK rather than to land?
        self._viewpoint_leg = False
        # Did the last _goto_via_map refuse to fly? See _do_travel.
        self._blocked_target = False
        # Viewpoints the vehicle could not reach. Not retried: the search would
        # otherwise loop between turning eight times and failing the same trip.
        self._octomap_msg = None        # raw and latched; see _cb_octomap
        self.octree_tree = None         # decoded per leg, in _goto_via_map
        b = [float(v) for v in self.get_parameter("plan_bounds").value]
        self.plan_bounds = (tuple(b[:3]), tuple(b[3:]))
        self.plan_allow_unknown = bool(
            self.get_parameter("plan_allow_unknown").value)
        # pad id -> (refusals that counted, when the last one counted).
        self._call: _Call | None = None
        self._pending: str | None = None    # what _call is for
        self._last_cmd_t = 0.0              # when the last command went out
        self._takeoff_tries = 0
        # Down-camera looks accepted during the current CONFIRM.
        self._confirm_hits = 0
        self._confirm_seen = 0          # belly frames that arrived at all
        self._confirm_best = 0.0        # best confidence any of them reached
        self._z_hist: list[tuple[float, float]] = []
        self._land_entry_z: float | None = None
        self._takeoff_start_z = 0.0
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

        # THE SETPOINT PUBLISHER IS NOT CREATED IN DRY RUN, and neither are the
        # three FCU clients below. A flag consulted at each call site is one
        # missed `if` away from commanding a vehicle somebody is holding; an
        # object that does not exist cannot be sent through by any path,
        # including one added later by someone who never read this comment.
        # _stream, _set_mode and _start_call all refuse on None, so the failure
        # mode of forgetting a guard is a log line, not a spinning motor.
        self.pub_plan = self.create_publisher(Path, p("out_plan"), 10)
        self.pub_sp = None if self.dry_run else self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_status = self.create_publisher(
            String, "/hydrone/mission/status", 10)

        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, sensor_qos)
        self.create_subscription(PadMap, "/hydrone/pads/map", self._cb_map, 10)
        # The occupancy map, latched: octomap_server publishes TRANSIENT_LOCAL
        # so a subscriber that connects late is handed the current tree at
        # once. With the default volatile QoS this subscription would match
        # nothing and the mission would silently fly every leg unchecked.
        self.create_subscription(
            Octomap, "/octomap/octomap_binary", self._cb_octomap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST))
        # The belly camera's OWN topic, not the shared /hydrone/pads/detections
        # the map is built from. Those detections carry no position — see
        # _cb_detection — so they must not reach pad_map_node, and keeping them
        # off its bus is what guarantees it.
        self.create_subscription(PadDetection,
                                 self.get_parameter("detections_topic").value,
                                 self._cb_detection, 20)
        # ArduPilot explains its refusals in STATUSTEXT ("Arm: Throttle too
        # high", "PreArm: VisOdom: not healthy", ...). MAVROS publishes
        # statustext/recv BEST_EFFORT; a RELIABLE subscription is
        # QoS-incompatible and receives nothing at all.
        self.create_subscription(StatusText, "/mavros/statustext/recv",
                                 self._cb_statustext, sensor_qos)

        if self.dry_run:
            self.cli_mode = None
            self.cli_arm = None
            self.cli_takeoff = None
        else:
            self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
            self.cli_arm = self.create_client(CommandBool,
                                              "/mavros/cmd/arming")
            self.cli_takeoff = self.create_client(CommandTOL,
                                                  "/mavros/cmd/takeoff")
        # The map's own services are NOT part of the lockdown: they move no
        # vehicle. Registering the takeoff base and marking a pad visited are
        # exactly the bookkeeping a rehearsal is there to exercise.
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

        if self.dry_run:
            # The human needs a REPEATING cue, not a one-shot line that has
            # scrolled away by the time they have both hands on the drone.
            self.create_timer(1.0, self._pilot_cue)
            # And one late check that nothing ELSE is driving the vehicle.
            # See _dry_audit.
            self._audit_timer = self.create_timer(10.0, self._dry_audit)
            self.get_logger().warn(
                "════════════════════════════════════════════════════════\n"
                "  DRY RUN — NOTHING IS SENT TO THE FLIGHT CONTROLLER.\n"
                "  No arm, mode or takeoff client exists in this node and no\n"
                "  setpoint is published. A human is the actuator: raise,\n"
                "  turn, carry and set the drone down when told to, and the\n"
                "  mission advances on the MEASURED pose exactly as it would\n"
                "  in flight. Instructions are the >>> lines below.\n"
                "\n"
                "  This stops the STACK commanding the vehicle. It does not\n"
                "  stop the vehicle ARMING — set an arming check it cannot\n"
                "  pass, and take the props off.\n"
                "════════════════════════════════════════════════════════")

        self.get_logger().info(
            f"phase1_mission ready — takeoff to {self.takeoff_alt:.1f} m, "
            f"land on {self.target_bases} base(s), "
            "then home. "
            f"{'Auto-starting.' if self.auto_start else 'Call /hydrone/mission/start.'}")

    # ────────────────────────────────────────────────────────────────────────
    # Inputs
    # ────────────────────────────────────────────────────────────────────────

    def _cb_state(self, msg: State):
        self.mav_state = msg

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_octomap(self, msg: Octomap):
        """Keep the newest tree as BYTES. Decoding happens when a leg is planned.

        This decoded eagerly until it was flown and watched. Two things were
        wrong with that, both measured on a 5.5 minute run:

        * the arena's tree reaches ~86000 nodes, and decoding it at the map's
          2 Hz means paying for a full deserialize 660 times a flight to answer
          the two questions a mission actually asks
        * octomap-python's readBinary prints "Tree size mismatch" to stderr on
          every call — expected and harmless (see hydrone_map.octree), but at
          2 Hz it buried the mission's own log. Finding out WHY this run hung
          meant digging the state lines out from under 600 copies of it.

        The map is latched, so a message kept here is always the current one.
        """
        self._octomap_msg = msg

    def _tree(self):
        """The current tree, decoded now, or None.

        Called at the top of a leg — twice a mission, not twice a second.
        """
        if self._octomap_msg is None:
            return None
        try:
            return octree.tree_from_msg(self._octomap_msg)
        except ValueError as exc:
            self.get_logger().warn(f"octomap: {exc}",
                                   throttle_duration_sec=20.0)
            return None

    def _cb_map(self, msg: PadMap):
        self.pad_map = msg

    def _cb_detection(self, msg: PadDetection):
        """Keep the freshest belly-camera look.

        CONFIDENCE ONLY. `msg.position` is not read here and is not populated
        by the down detector at all: this camera answers "is there a base under
        me", and the answer to "where is it" comes from the ZED, through the
        map, which is the estimate the drone was flown here on.

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
        self.landing_for = self.LAND_PAD
        self._land_after_takeoff = False
        # An abort mid-leg must not leave waypoints behind for the next one.
        self._leg = []
        self._travel_best = None
        self._viewpoint_leg = False
        self.survey_done = False
        self.investigating = False
        self._survey_visits = 0
        self._survey_stalls = 0
        self._survey_last_gain = None
        # A new attempt sweeps again, whatever the last one flew.
        self._survey_path = None
        self._level = 1

    # ────────────────────────────────────────────────────────────────────────
    # Setpoint stream
    # ────────────────────────────────────────────────────────────────────────

    def _stream(self):
        """Publish the current position + yaw target.

        Deliberately silent whenever the FCU owns the descent (LAND, DWELL): a
        position setpoint arriving mid-landing is at best ignored and at worst
        fights the flare.
        """
        # Dry run: there IS no publisher. The state machine still sets
        # `stream_setpoint` and still keeps `self.setpoint` up to date — that is
        # what _pilot_cue reads to tell the human where to put the drone — it
        # just has nowhere to send it.
        if self.pub_sp is None or not self.stream_setpoint:
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

    def _goto_via_map(self, x: float, y: float, z: float, yaw: float):
        """Fly to (x, y, z), around whatever the occupancy map says is there.

        Until 2026-08-27 every leg was a single setpoint on a straight line and
        nothing consulted the map — which is survivable in an 8x8 m open arena
        and is not survivable in Phase 4's confined space. The map has known
        what is occupied for weeks; this is what makes the mission read it.

        Three outcomes, and the fall-back is deliberate:

        * the straight line is clear (the usual case) -> one setpoint, exactly
          the old behaviour, no waypoints and no extra decelerations
        * it is not, and A* finds a way round -> the simplified waypoints
        * there is no map yet, or it is too sparse to plan in -> the straight
          line, with a warning. Refusing to fly because the map is thin would
          ground the vehicle at takeoff, when the map is always thin. The
          straight line is what this mission did for its whole life so far, so
          falling back to it is the status quo, not a new risk.
        """
        self._leg = []
        self._blocked_target = False
        target = (x, y, z)
        here = (self.pose.pose.position.x, self.pose.pose.position.y,
                self.pose.pose.position.z)
        # Decoded once, here, and used for every question this leg asks.
        self.octree_tree = self._tree()
        occ = self._occupancy()
        if occ is None:
            self.get_logger().warn(
                "no occupancy map — flying the leg straight, unchecked",
                throttle_duration_sec=20.0)
            self._goto(x, y, z, yaw)
            self._publish_plan([here, target], yaw)
            return

        # `path_hits_obstacle`, not `path_is_clear_inflated`. The strict
        # version demands the whole leg be MEASURED empty, and in a
        # half-explored arena almost no leg is — every one would be reported
        # blocked, which is a warning that means nothing. What has to trigger a
        # detour is something actually in the way.
        if not octree.path_hits_obstacle(self.octree_tree, here, target):
            self._goto(x, y, z, yaw)
            self._publish_plan([here, target], yaw)
            return

        self.get_logger().warn(
            f"the straight leg to ({x:.2f}, {y:.2f}) runs into the map — "
            f"planning around it")
        # allow_unknown=True, and this is a deliberate choice for THIS mission,
        # not a default to carry into Phase 4. The fallback if planning fails
        # is the straight line, which flies through unknown space without
        # asking; so a plan that avoids what is known to be occupied and is
        # otherwise willing to cross unknown is strictly better than what this
        # mission did before there was a planner. Phase 4's confined space is
        # where allow_unknown should be False and the map should be dense
        # enough to afford it.
        path = planner.plan(occ, here, target,
                            resolution=self.octree_tree.getResolution(),
                            bounds=self.plan_bounds,
                            allow_unknown=self.plan_allow_unknown)
        if path is None:
            # The straight line is KNOWN to run into something and no way round
            # it exists in the map. Flying it anyway was what this did, "relying
            # on the supervisor" — and what that produced was the drone hitting
            # a wall. There is no supervisor input in a 2 m leg at cruise.
            #
            # Refusing costs one target. Flying it costs the aircraft, and in
            # the competition it costs the attempt.
            # SAY WHAT IS THERE. "No way round" is a conclusion, not evidence,
            # and it has already sent one confirmed base to the blacklist in an
            # empty arena. These are the states A* actually saw.
            raw = octree.query(self.octree_tree, (x, y, z))
            infl = occ((x, y, z))
            col = " ".join(
                f"{zz:+.1f}:{octree.query(self.octree_tree, (x, y, zz))[:4]}"
                for zz in (z - 0.6, z - 0.3, z, z + 0.3, z + 0.6))
            self.get_logger().error(
                f"({x:.2f}, {y:.2f}, {z:.2f}) is blocked and no way round it "
                f"exists in the map — REFUSING the leg. Holding position. "
                f"[goal raw={raw} inflated={infl} | column {col}]")
            self._leg = []
            self._hold()
            self._blocked_target = True
            return

        path = planner.simplify(
            path, lambda a, b: not octree.path_hits_obstacle(
                self.octree_tree, a, b))
        self.get_logger().info(
            f"planned {len(path)} waypoints around the obstruction")
        # path[0] is where we already are; the rest are the leg.
        self._leg = [(p[0], p[1], p[2], yaw) for p in path[1:]]
        self._publish_plan(path, yaw)
        wx, wy, wz, wyaw = self._leg.pop(0)
        self._goto(wx, wy, wz, wyaw)

    def _occupancy(self):
        """The map as a callable for the planner, already inflated, or None.

        Inflated here rather than in the planner because the planner must not
        know what an octree is — and because the radius is a property of the
        airframe, which is this node's business.
        """
        if self.octree_tree is None:
            return None
        return lambda p: octree.inflated_state(self.octree_tree, p)

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
        # Not in dry run: those clients do not exist, so there is nothing to
        # ask. Waiting on them would hold the rehearsal at WAIT_FCU forever.
        if not self.dry_run and not (self.cli_arm.service_is_ready()
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

        DRY RUN: none of that happens. After `dry_arm_delay_s` on the base the
        rehearsal simply declares the vehicle armed and moves on, because the
        arm is not the interesting part — what the arm TRIGGERS is. Registering
        the takeoff base and opening pad_map's gate both hang off this moment,
        and both still happen, from REGISTER, exactly as they do in flight.
        """
        if self.dry_run:
            if self._since_entered() < self.dry_arm_delay:
                return
            self.get_logger().warn(
                "[dry] WOULD ARM (GUIDED, then arm) — nothing sent. Treating "
                "the vehicle as armed from here; the takeoff base is about to "
                "be registered and the map starts accepting detections.")
            self._enter(self.REGISTER if not self.base_registered
                        else self.TAKEOFF)
            return

        if self.mav_state.mode == "GUIDED" and self.mav_state.armed:
            # `_takeoff_tries` is NOT reset here. TAKEOFF bounces back to this
            # state on every refusal, and clearing the counter on the way
            # through means the three-strike abort can never accumulate —
            # MEASURED 2026-08-28: after landing on an elevated base at
            # z=0.89 m the FCU refused takeoff, and the mission sat in
            # ARMING <-> TAKEOFF for the rest of the flight, retrying every
            # two seconds and never saying anything but "failed: no reason
            # given by the FCU".
            #
            # DWELL zeroes it when a genuinely new landing cycle starts, which
            # is the place that means "this is a fresh attempt".
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
        # CLIMBED, not absolute altitude. takeoff_alt is a height above the
        # surface we are leaving; pose.z is measured from the plane of the FIRST
        # takeoff. On any pad at a different height the two disagree by exactly
        # that difference, and comparing them directly is why the mission hung
        # here on 2026-08-23: after landing on a pad 0.76 m below the start
        # plane, a perfect 1.5 m climb reached z=0.74 while this test wanted
        # 1.35, so the mission re-sent takeoff forever — and ArduPilot rejected
        # every one of them, because the vehicle was already flying.
        climbed = (self.pose.pose.position.z - self._takeoff_start_z
                   if self.pose is not None else 0.0)
        if self.pose is not None and climbed >= self.takeoff_alt - 0.15:
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            yaw = yaw_of(self.pose)
            self.get_logger().info(
                f"airborne — climbed {climbed:.2f} m to z="
                f"{self.pose.pose.position.z:.2f} m, "
                f"heading {math.degrees(yaw):.0f} deg.")
            self._goto(x, y, self.takeoff_alt, yaw)
            if self._land_after_takeoff:
                # The fallback's second hop: up, then straight back down.
                self._land_after_takeoff = False
                self.get_logger().info(
                    "fallback hop complete — landing to end the run.")
                self._begin_landing()
                return
            self._enter(self.SELECT)
            return

        # DRY RUN: no command, and deliberately no timeout. The climb check
        # above is the REAL one — it is satisfied when the person actually
        # raises the drone — and ArduPilot is not here to refuse anything, so
        # the "takeoff refused three times, aborting" path below would only
        # ever fire on somebody being slow with their hands. _pilot_cue does
        # the asking.
        if self.dry_run:
            return

        if self._poll_call() == "pending":
            return

        if self._since_entered() > self.takeoff_timeout:
            self._takeoff_tries += 1
            if self._takeoff_tries > 3:
                self.get_logger().error(
                    f"takeoff refused three times from z="
                    f"{self.pose.pose.position.z if self.pose else 0.0:.2f} m "
                    f"after {self.landed_count} landing(s) — check EKF "
                    f"origin/home (docs/DEVELOP-PIPELINES.md: no origin -> no "
                    f"home -> NAV_TAKEOFF fails). Aborting rather than "
                    f"retrying for the rest of the attempt.")
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
            self._viewpoint_leg = False
            self._goto_via_map(hx, hy, self.takeoff_alt, self.setpoint[3])
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
            self._viewpoint_leg = False
            # CLEARANCE OVER THE PAD, not an altitude. A competition base may be
            # anywhere from 0 to 1.5 m tall, so a fixed confirmation altitude
            # means the camera is a different distance from every pad it looks
            # at — 2.2 m over one on the floor and 0.77 m over a 1.5 m one, from
            # the same setpoint. That is the difference between a pad filling a
            # third of the frame and filling most of it, and it moves the servo
            # scale by the same factor.
            #
            # Held above the pad's own measured top instead, so every
            # confirmation is flown from the same distance. Floored at
            # the old behaviour, and clamped so a tall base cannot push the
            # vehicle into the net.
            # ONE ALTITUDE. The vehicle cruises, searches and confirms at
            # `takeoff_alt` and only ever leaves it to land, returning to it
            # afterwards. Three separate heights used to be computed here from
            # the pad's own height; that bought nothing the belly camera could
            # not do from a single fixed height, and every one of them was
            # another number to get wrong.
            self._goto_via_map(pad.position.x, pad.position.y,
                               self.takeoff_alt, self.setpoint[3])
            self._enter(self.TRAVEL)
            return

        self._enter(self.SETTLE)

    def _best_candidate(self):
        """The nearest pad worth flying to, or None.

        The rule lives in hydrone_nav.route, so a later phase can reuse it (or
        swap the nearest-first choice for a real tour) without reaching into a
        mission node. This stays as the node's way of asking with its own state.
        """
        if self.pad_map is None or self.pose is None:
            return None
        return route.nearest_candidate(
            self.pad_map.pads,
            self.pose.pose.position.x,
            self.pose.pose.position.y,
            blacklist=self.blacklist,
            home=self.home,
        )

    def _is_candidate(self, pad) -> bool:
        """Is this pad worth flying to?

        While INVESTIGATING the bar drops to a single sighting. That is not the
        bar being wrong the rest of the time — it is that the cost of being
        wrong has changed. During the search a doubtful lead competes with
        finding real bases; once the search has stalled short of the quota, the
        only thing a doubtful lead competes with is climbing half a metre and
        flying the whole U again. A hover settles it either way, and a failed
        one blacklists the pad.
        """
        # A pad the planner just refused is DEFERRED, not merely uncounted.
        # Leaving it selectable while the cooldown runs is a livelock: the
        # mission picks the same pad, the same octomap refuses the same leg, and
        # nothing else is ever tried. MEASURED 2026-09-01: four refusals of
        # pad 2 in twenty seconds, all logged (1/3), no progress in between.
        #
        # Deferring it lets the search fly somewhere else — which is the only
        # thing that can put new rays in the map and change the answer.
        n = 1 if self.investigating else route.MIN_OBSERVATIONS
        return route.is_candidate(pad, blacklist=self.blacklist,
                                  home=self.home, min_observations=n)

    def _takeoff_base_xy(self) -> tuple[float, float]:
        """Where home is. The map's registered entry if there is one, else the
        position we armed at."""
        pads = self.pad_map.pads if self.pad_map is not None else ()
        return route.takeoff_base_xy(
            pads, fallback=self.home if self.home is not None else (0.0, 0.0))

    # ── SETTLE ───────────────────────────────────────────────────────────────

    def _do_settle(self):
        """Hold still, let the estimate stop moving, then decide what is next.

        The map is NOT read while this is counting down. That is the whole point
        of the state: a detection taken while yaw was still slewing is projected
        through a moving estimate, and the position it produces is wrong by
        metres. Waiting costs two seconds and buys a map entry that means what
        it says.

        The order below is the mission's whole strategy, and each step exists
        because the one before it was measured to be insufficient:

        1. **Fly the circuit.** Sweep the arena before committing to anything.
        2. **Land on what was found**, best candidate first.
        3. **Short of the quota? Investigate the relief** the occupancy map
           found — that is where an ELEVATED base hides, because the
           ground-plane projection cannot place one.
        4. **Still short? Go and look from somewhere new.**
        5. **Otherwise go home** — and home, never here, because an off-base
           landing is eliminatory.
        """
        self._hold()
        if self._since_entered() < self.settle_s:
            return

        # ── 1. the circuit ──────────────────────────────────────────────────
        #
        # Turning on the spot cannot map an arena — what limits the map is not
        # where the camera POINTS but where it has PARALLAX, and a camera that
        # never translates never sees behind anything. MEASURED 2026-08-28, the
        # "directed" version of spinning came back 22, -22, 68, -68, 112, -112,
        # 158, -158: a full circle with the turns merely reordered.
        #
        # The rectangle that replaced it was better and still wrong in the same
        # direction: it re-aimed the camera at the arena centre at every step,
        # so it turned CONTINUOUSLY along every edge.
        #
        # This turns ONCE. Two straight passes across the arena, heading fixed
        # in each, looking back the other way on the return — so a pad that is
        # edge-on or back-lit going out is face-on coming back. A fixed heading
        # during the translation is worth more than the angles it gives up: the
        # detector gets a stable scene, the depth camera sweeps a clean band
        # into the occupancy map, and the odometry is never asked to do the one
        # thing this arena breaks it on.
        #
        # house, whose roof is at 1.5 m, and the cruise height is 1 m.
        if not self.survey_done and self.survey_circuit:
            if self._survey_path is None:
                if not self._begin_level():
                    # No level left. Land on whatever was found.
                    self.survey_done = True
                    self._enter(self.SELECT)
                    return
                if self.survey_done:
                    # A level with no path of its own (level 3 is the
                    # rotate-and-investigate behaviour further down). Hand over
                    # rather than judging it flown on the spot — MEASURED
                    # 2026-08-28, without this level 3 escalated 0.08 s after
                    # starting and never ran at all.
                    return
            # LAND ON WHAT IS ALREADY CONFIRMED, mid-level.
            #
            # The rule used to be "fly the whole level, then commit", and the
            # reason was real: chasing the first sighting spends the battery on
            # whatever happened to be in front of the camera at take-off, and
            # the bases nobody turned to look at are never found at all.
            #
            # That reason has expired. A confirmed pad now means three separate
            # looks agreeing, and MEASURED 2026-09-02 those land 0.04-0.20 m
            # from the real base. Meanwhile the levels got long: the lawnmower
            # is 42 points, minutes of flight, and the vehicle was flying all of
            # it with confirmed bases sitting untouched in the map.
            #
            # So the barrier stays where it earns its keep — level 1, which is
            # the sweep that stops the mission being a slave to its first
            # sighting — and from level 2 on, a confirmed pad is worth landing
            # on NOW. The level resumes afterwards: `_survey_path` is not
            # cleared, so the remaining points are still flown.
            if (self.land_during_survey and self._level >= 2
                    and self.landed_count < self.target_bases
                    and self._best_candidate() is not None):
                self.get_logger().info(
                    f"a confirmed base is in the map and level {self._level} "
                    f"still has {len(self._survey_path or [])} point(s) to fly "
                    f"— landing on it first, then resuming the level.")
                self._enter(self.SELECT)
                return

            if self._survey_path:
                x, y, z, yaw = self._survey_path.pop(0)
                self._viewpoint_leg = True   # to LOOK: never confirms, never lands
                self._goto_via_map(x, y, z, yaw)
                self._enter(self.TRAVEL)
                return

            # This level is flown. Did it find enough?
            found = self._candidate_count()
            if found + self.landed_count >= self.target_bases:
                self.survey_done = True
                self.get_logger().info(
                    f"SEARCH LEVEL {self._level} found all "
                    f"{self.target_bases} base(s) — landing phase begins.")
                self._enter(self.SELECT)
                return

            # INVESTIGATE BEFORE CLIMBING. A pad the map holds but has not
            # confirmed is a lead the belly camera can settle in one hover;
            # climbing half a metre and flying the whole U again is minutes.
            #
            # MEASURED 2026-08-28: a run ended a level with 4 confirmed and 2
            # unconfirmed candidates and escalated anyway — investigating those
            # two would have completed the quota there and then, and instead
            # two more levels were flown and still came back 5 of 6.
            weak = self._uninvestigated()
            if weak and found + self.landed_count < self.target_bases:
                self.investigating = True
                self.survey_done = True
                self.get_logger().info(
                    f"SEARCH LEVEL {self._level} flown: {found} confirmed and "
                    f"{len(weak)} unconfirmed candidate(s). Investigating "
                    f"those before climbing — a hover settles one, a whole "
                    f"level costs minutes.")
                self._enter(self.SELECT)
                return

            self.get_logger().warn(
                f"SEARCH LEVEL {self._level} flown and only {found} of "
                f"{self.target_bases} base(s) are in the map — escalating.")
            self._level += 1
            self._survey_path = None
            self.investigating = False
            return

        # ── 2. land on what was found ───────────────────────────────────────
        #
        # Only once the sweep is over. Running at the first sighting is
        # explore-nothing/exploit-everything in the worst order: the battery
        # goes on whichever base happened to be in front of the camera at
        # takeoff, and the ones never turned towards are never found at all.
        pad = self._best_candidate()
        if pad is not None and self.survey_done:
            self._enter(self.SELECT)
            return

        if (not self.investigating and self.landed_count < self.target_bases
                and self._uninvestigated()):
            self.investigating = True
            if self._best_candidate() is not None:
                self.get_logger().info(
                    f"the search is spent at "
                    f"{self.landed_count}/{self.target_bases} — investigating "
                    f"the unconfirmed lead(s) still in the map before "
                    f"escalating; a hover settles one, a level costs minutes.")
                self._enter(self.SELECT)
                return
            self.investigating = False


        # ── 5. escalate, or go home ─────────────────────────────────────────
        #
        # Level 3 (rotate + relief) has no path of its own, so it ends here:
        # out of candidates, out of viewpoints, quota unmet. That is the moment
        # to spend the expensive level rather than to give up.
        if (self.landed_count < self.target_bases
                and self._level < self.max_search_level):
            self._level += 1
            self._survey_path = None
            self.survey_done = False
            self.investigating = False
            self.get_logger().warn(
                f"still {self.landed_count}/{self.target_bases} base(s) and "
                f"level {self._level - 1} is spent — escalating to search "
                f"level {self._level}.")
            return

        hx, hy = self._takeoff_base_xy()
        self.get_logger().warn(
            f"nothing left to find — returning to the takeoff base at "
            f"({hx:.2f}, {hy:.2f}) to end the run. NOT landing here: off-base "
            f"landings are eliminatory.")
        self.target_id = None
        self.landing_for = self.LAND_FINAL
        self._viewpoint_leg = False
        self._goto_via_map(hx, hy, self.takeoff_alt, self.setpoint[3])
        self._enter(self.TRAVEL)

    def _uninvestigated(self):
        """Pads the map holds but has not confirmed, and nobody has looked at.

        Below the targeting bar — one sighting, or a relief lead the occupancy
        map raised — so `_is_candidate` refuses them and they would otherwise
        sit in the map untouched for the whole attempt. They are exactly the
        leads a confirmation hover exists to settle.
        """
        pads = self.pad_map.pads if self.pad_map else []
        return [p for p in pads
                if not p.is_takeoff_base
                and not p.visited
                and int(p.id) not in self.blacklist
                and not self._is_candidate(p)]

    def _candidate_count(self) -> int:
        """Bases in the map that are still worth flying to."""
        pads = self.pad_map.pads if self.pad_map else []
        return sum(1 for p in pads if self._is_candidate(p))

    def _begin_level(self) -> bool:
        """Build the path for the current search level. False when none is left.

        LEVELS, in the order they are spent. Each exists because the one before
        it can miss a base, and each costs more than the one before — which is
        the whole reason for the ladder rather than starting with the thorough
        one.

          1  the U at cruise height. Three sides, two corner turns, camera
             facing in. Cheapest shape that sees the whole floor.
          2  the same U half a metre higher. A base the first pass saw
             edge-on, or that the house occluded, opens up from higher —
             raising the camera changes the geometry without changing the
             flight.
          3  turn on the spot and investigate the RELIEF the occupancy map
             found. This is where an ELEVATED base is caught: the ground-plane
             projection cannot place one, so the blue detector's answer for it
             is in the wrong place however well it was seen.
          4  the lawnmower. Lanes across the whole arena, several times the
             flight time, and it finds what every other level looked past.

        Level 3 has no path — it is the rotate-and-investigate behaviour that
        the rest of _do_settle already implements, so this returns an empty
        list for it and lets the sweep fall through.
        """
        if self._level == 1:
            self._survey_path = coverage.u_sweep(
                self.plan_bounds, inset_m=self.survey_inset_m,
                z=self.takeoff_alt, start_corner=self._nearest_corner(),
                side_x_m=self.u_side_x_m, side_y_m=self.u_side_y_m)
            self.get_logger().info(
                f"SEARCH LEVEL 1: the U at {self.takeoff_alt:.1f} m — "
                f"{len(self._survey_path)} setpoints, two corner turns, one "
                f"per leg, camera facing into the arena.")
            return True

        if self._level == 2:
            # THE SAME U AGAIN, at the same height. It used to climb
            # `level2_climb_m`; that is gone with the second altitude. What
            # makes a second pass worth flying now is not the height, it is the
            # map: the belly camera has been projecting positions the whole
            # first pass, so the arena the second one flies over is better
            # known than the one the first saw.
            self._survey_path = coverage.u_sweep(
                self.plan_bounds, inset_m=self.survey_inset_m,
                z=self.takeoff_alt, start_corner=self._nearest_corner(),
                side_x_m=self.u_side_x_m, side_y_m=self.u_side_y_m)
            self.get_logger().info(
                f"SEARCH LEVEL 2: the same U at {self.takeoff_alt:.1f} m — a "
                f"second pass over an arena the first one mapped.")
            return True

        # THE LADDER IS TWO LEVELS. There used to be four.
        #
        # Level 3 turned on the spot, scanned the octomap for relief and
        # repositioned to computed viewpoints. MEASURED across every run on
        # 2026-09-01/02 it produced ZERO candidates — the relief scan never
        # returned a spot — so it was minutes of flight that could not
        # contribute a base, and it is what made the vehicle look like it was
        # wandering the arena.
        #
        # Level 4 was a 42-point lawnmower. It found bases, but it also planned
        # points OUTSIDE the arena (plan_bounds is +-5 m, the arena is +-4) and
        # cost several times the flight of the U for them.
        #
        # What replaced both is cheaper and already proven: the belly camera
        # projects positions with the rangefinder, so the two U sweeps now find
        # what only a slow pass overhead used to.
        return False

    def _nearest_corner(self) -> int:
        """Which inset corner the vehicle is closest to, so the sweep starts
        where it already is instead of transiting first."""
        (min_x, min_y, _), (max_x, max_y, _) = self.plan_bounds
        i = self.survey_inset_m
        corners = [(min_x + i, min_y + i), (max_x - i, min_y + i),
                   (max_x - i, max_y - i), (min_x + i, max_y - i)]
        if self.pose is None:
            return 0
        here = (self.pose.pose.position.x, self.pose.pose.position.y)
        return min(range(4), key=lambda k: math.dist(corners[k], here))

    def _publish_plan(self, points, yaw):
        """The route, for RViz and for a human to check. Purely informational."""
        if self.pub_plan is None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = (self.pose.header.frame_id
                                if self.pose is not None else "map")
        for p in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
            ps.pose.position.z = float(p[2])
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.pub_plan.publish(path)

    def _do_travel(self):
        """Fly to the setpoint SELECT placed. One leg, one setpoint.

        Heading is untouched on the way: turning while translating would put the
        detector's geometry and the position controller's demand in motion at
        the same time, and there is nothing to gain — the belly camera looks
        straight down and does not care which way the nose points.

        The leg ends when the vehicle arrives, or when it stops getting closer.

        There used to be a flat 60 s budget and it was removed for a good
        reason: it blacklisted candidates that were still closing — pad 4 on
        2026-08-23 was 0.70 m away when it fired. What replaces it is not that
        budget back. It is a STALL test, and the difference is the whole point:
        a pad still being approached is making progress and never trips it, no
        matter how slow the leg is.

        Why anything is needed at all. MEASURED 2026-08-27, a plain --phase1
        run flying on the VO:

            [ARMING -> REGISTER -> TAKEOFF -> SELECT -> TRAVEL]   and then
            nothing, for four and a half minutes, with the vehicle parked.

        The pose had drifted 4-5 m in an 8 m arena, so the target sat 4 m from
        where the vehicle believed it was — permanently. `d` never fell below
        arrive_tol because it COULD not. One leg, one mission, no landings.
        "Every run is supervised, so a slow leg is a human's call" holds for a
        leg that is slow; it does not hold for one that will never finish, and
        this cannot tell a human anything if it never says a word.

        Blacklisting is the right response and not a guess: the pad is
        unreachable FROM THIS ESTIMATE, and the search resuming is what gives
        the estimate a chance to change before anything else is attempted.
        """
        if self.pose is None:
            return
        self._hold()

        if self._blocked_target:
            # _goto_via_map refused to fly this leg. Do not sit here waiting
            # for an arrival that was never commanded.
            self._blocked_target = False
            if self._viewpoint_leg:
                self._viewpoint_leg = False
            elif self.target_id is not None:
                self.get_logger().warn(
                    f"pad {self.target_id} is unreachable in the map — "
                    f"blacklisting it and searching on.")
                self.blacklist.add(int(self.target_id))
                self.target_id = None
            self._enter(self.SETTLE)
            return

        # RE-AIM IF THE MAP MOVED THE PAD. The setpoint was fixed when SELECT
        # committed to this pad, but the map keeps fusing looks while the leg
        # flies, and closer looks are worth far more than the distant one that
        # first proposed it (pad_map weights by 1/range^2). So the estimate that
        # sent the vehicle here is routinely the WORST one the mission will
        # have.
        #
        # MEASURED 2026-09-01: pad 3 was proposed at (2.93, -0.96) from 7.8 m
        # and corrected to (1.91, -1.18) — 1.02 m — while the leg was in the
        # air. The vehicle flew to the stale point, ended up 0.99 m from the pad
        # it believed it was over, the belly camera saw nothing in 25 s, and a
        # REAL base was blacklisted.
        #
        # Only while there is a target pad and no bent path in progress: a
        # `_leg` is a route around an obstacle and re-aiming mid-detour would
        # throw away the avoidance.
        if (self.target_id is not None and not self._viewpoint_leg
                and not self._leg):
            tgt = self._target_xy()
            if tgt is not None:
                moved = math.hypot(tgt[0] - self.setpoint[0],
                                   tgt[1] - self.setpoint[1])
                if moved > self.retarget_tol_m:
                    self.get_logger().info(
                        f"pad {self.target_id} moved {moved:.2f} m in the map "
                        f"while flying to it — re-aiming at "
                        f"({tgt[0]:.2f}, {tgt[1]:.2f})")
                    self._goto(tgt[0], tgt[1], self.setpoint[2],
                               self.setpoint[3])
                    # The stall test measures progress toward a target; that
                    # target just changed, so its history is about somewhere
                    # else and would fire on the jump.
                    self._travel_best = None
                    self._travel_progress_t = self._now()

        d = math.hypot(self.pose.pose.position.x - self.setpoint[0],
                       self.pose.pose.position.y - self.setpoint[1])

        # Progress, not elapsed time. `_travel_best` is the closest this leg has
        # ever been; improving it resets the clock.
        now = self._now()
        if self._travel_best is None or d < self._travel_best - self.travel_progress_m:
            self._travel_best = d
            self._travel_progress_t = now
        elif now - self._travel_progress_t > self.travel_stall_s:
            self._on_travel_stalled(d)
            return

        # ARRIVED MEANS POSITION AND HEADING, measured from the pose — never
        # from a timer.
        #
        # A corner of the U is two setpoints at the SAME PLACE: one that only
        # turns, then the leg that flies away on the new heading. Testing
        # arrival by distance alone makes the turning setpoint "arrive" the
        # instant it is issued, because the distance is already zero — so the
        # next leg was released while the vehicle was still rotating, and it
        # flew the corner as a curve with the camera sweeping through it.
        #
        # `yaw_tol` was declared for this and had no reader; it belonged to a
        # rotate state that no longer exists.
        dyaw = abs(wrap_pi(yaw_of(self.pose) - self.setpoint[3]))
        if d <= self.arrive_tol and dyaw > self.yaw_tol:
            self.get_logger().info(
                f"in place, still turning: {math.degrees(dyaw):.0f} deg to go "
                f"(tolerance {math.degrees(self.yaw_tol):.0f})",
                throttle_duration_sec=2.0)
            return

        if d <= self.arrive_tol:
            # Intermediate waypoints from _goto_via_map come first: arriving at
            # one is not arriving at the pad, it is the corner of a leg that
            # had to bend around something.
            if self._leg:
                wx, wy, wz, wyaw = self._leg.pop(0)
                self.get_logger().info(
                    f"waypoint reached — {len(self._leg)} to go, next "
                    f"({wx:.2f}, {wy:.2f}, {wz:.2f})")
                self._goto(wx, wy, wz, wyaw)
                return
            # A COVERAGE leg went somewhere to LOOK. Arriving is the end of
            # the trip, not the start of a landing — there is no target pad
            # under it and nothing has said there is a base here at all.
            #
            # This was missing when coverage first flew, and the result is the
            # worst failure this mission has: MEASURED 2026-08-27, a run that
            # reported "6 of 6 bases" had landed on ONE. The other five were
            # this branch — "over pad None — confirming on the belly camera" —
            # putting the vehicle down mid-arena on whatever happened to look
            # blue from 1 m. Landing off a base is ELIMINATORY, so a viewpoint
            # arrival must never reach CONFIRM.
            if self._viewpoint_leg:
                self._viewpoint_leg = False
                self.get_logger().info(
                    f"arrived at the viewpoint ({self.setpoint[0]:.2f}, "
                    f"{self.setpoint[1]:.2f}) — looking around from here.")
                self._enter(self.SETTLE)
                return
            if self.landing_for == self.LAND_FINAL:
                self.get_logger().info(
                    "over the takeoff base — landing to finish the run.")
                self._begin_landing()
            elif self.target_id is None:
                # Belt and braces behind the check above. Nothing may descend
                # without a pad it is descending ONTO: "over pad None" is how
                # the vehicle ends up on the floor, and that ends the run.
                self.get_logger().error(
                    "arrived with no target pad — refusing to confirm or land. "
                    "Resuming the search.")
                self._leg = []
                self._enter(self.SETTLE)
            else:
                self.get_logger().info(
                    f"over pad {self.target_id} — confirming on the belly "
                    "camera.")
                self._confirm_hits = 0
                self._confirm_seen = 0
                self._confirm_best = 0.0
                self._servo.reset()
                self._enter(self.CONFIRM)
            return

    def _on_travel_stalled(self, d: float):
        """The leg stopped closing. Give up on this target and search again.

        Three kinds of leg end up here and they deserve different endings:

        * a **coverage** leg is going somewhere to LOOK, so there is no pad to
          blacklist. What has to be remembered is the VIEWPOINT, or the next
          sweep picks the same unreachable spot and the search loops between
          turning eight times and failing to fly to the same place.
        * a **pad** leg blacklists its target: it is unreachable from this
          estimate, and resuming the search is what gives the estimate a chance
          to change before anything else is tried.
        * the **final return** leg is neither. There is no other candidate to
          fall back to and the run has to end on the takeoff base, so stalling
          there is reported and the leg is left running.
        """
        if self._viewpoint_leg:
            self.get_logger().warn(
                f"could not reach the viewpoint at "
                f"({self.setpoint[0]:.2f}, {self.setpoint[1]:.2f}) — stopped "
                f"{d:.2f} m out. Not going back to it; resuming the search "
                f"from here.")
            self._viewpoint_leg = False
            self._leg = []
            self._enter(self.SETTLE)
            return

        if self.landing_for == self.LAND_FINAL:
            self.get_logger().error(
                f"the return leg has not closed in {self.travel_stall_s:.0f} s "
                f"and is stuck {d:.2f} m out — the estimate has drifted. "
                f"Holding the leg; a human decides this one.",
                throttle_duration_sec=30.0)
            return

        self.get_logger().error(
            f"pad {self.target_id} stopped getting closer {d:.2f} m out and "
            f"has not improved in {self.travel_stall_s:.0f} s — unreachable "
            f"from this estimate. Blacklisting it and resuming the search.")
        if self.target_id is not None:
            self.blacklist.add(int(self.target_id))
        self.target_id = None
        self._leg = []
        self._enter(self.SELECT)

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
        if fresh:
            # Counted BEFORE the confidence gate, which is the whole point: a
            # frame that arrives and scores 0.2 and a frame that never arrives
            # are the same "0/6 looks" in the log, and they have opposite
            # fixes. One is the detector, the other is where the vehicle is.
            self._confirm_seen += 1
            self._confirm_best = max(self._confirm_best,
                                     float(self._last_down.confidence))
        if fresh and self._last_down.confidence >= self.confirm_conf:
            # CENTRE THE PAD FIRST. The hover is directly over the thing it is
            # about to land on, and "directly" is the word doing the work: the
            # position came from a projection made across the arena, and the
            # belly camera at 1 m is the only sensor that can say where the pad
            # actually is relative to the vehicle.
            #
            # The pixel-to-metre mapping is LEARNED, not assumed — see
            # hydrone_nav.servo. It is a rotation, a sign and a scale, none of
            # which can be written down for an airframe whose camera may be
            # bolted on differently from the simulator's, and getting the sign
            # wrong does not centre slowly, it flies away.
            self._centre_on_pad(self._last_down)
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
            # WHICH failure was it. `seen` counts belly frames that arrived at
            # all; `best` is the highest confidence any of them reached against
            # the gate. seen>0 with best below the gate means the camera was
            # looking at the pad and the detector would not call it — lighting,
            # threshold, exposure. seen==0 means the camera was pointed at
            # empty floor, and no detector change can help that.
            tgt = self._target_xy()
            pos = (self.pose.pose.position if self.pose is not None else None)
            if pos is not None and tgt is not None:
                d = math.hypot(pos.x - tgt[0], pos.y - tgt[1])
                self.get_logger().warn(
                    f"  confirm autopsy: {self._confirm_seen} belly frame(s) "
                    f"arrived, best conf {self._confirm_best:.2f} vs gate "
                    f"{self.confirm_conf:.2f} | vehicle at "
                    f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), pad believed at "
                    f"({tgt[0]:.2f}, {tgt[1]:.2f}), off by {d:.2f} m")
            self._reject_target()

    def _height_over_pad(self) -> float:
        """Camera height above the surface being centred on, in metres.

        The pad's own `height` when the map has one (it is corrected from the
        rangefinder on the first hover), else the arena floor. Never below a
        floor of 0.2 m: a non-positive or absurdly small value would blow the
        servo's scale up instead of down.
        """
        z = self.pose.pose.position.z if self.pose is not None else 0.0
        top = self.ground_z
        if self.target_id is not None and self.pad_map is not None:
            for pad in self.pad_map.pads:
                if int(pad.id) == int(self.target_id):
                    if pad.height_measured:
                        top = float(pad.height)
                    break
        return max(z - top, 0.2)

    def _target_xy(self):
        """Where the map believes the pad being confirmed is, or None."""
        if self.target_id is None or self.pad_map is None:
            return None
        for pad in self.pad_map.pads:
            if int(pad.id) == int(self.target_id):
                return (pad.position.x, pad.position.y)
        return None

    def _centre_on_pad(self, det):
        """Nudge the setpoint so the belly camera's pad moves to `target_uv`.

        The step comes back in the BODY frame and is rotated into the world by
        the vehicle's own yaw before it becomes a setpoint — the servo knows
        about pixels and the airframe, not about where north is.
        """
        if not self.centre_on_pad or self.pose is None:
            return
        # HEIGHT ABOVE THE PAD, not altitude. The pixel-to-metre scale is a
        # function of how far the camera is from the SURFACE it is looking at,
        # and `position.z` is measured from the takeoff plane — which is the
        # top of the base the drone armed on, not the ground and not this pad.
        #
        # MEASURED 2026-09-01, confirmation hover at z = 1.5 m:
        #
        #     pad on the floor (top at -0.70)  ->  really 2.20 m, servo told 1.50
        #     base 1.43 m tall (top at  0.73)  ->  really 0.77 m, servo told 1.50
        #
        # Twice the real height on a tall base means every correction comes out
        # twice too big. It does not converge, it hunts: the nudges on pad 5 ran
        # +0.25 then -0.25 m until the vehicle slid off the base and landed on
        # the floor beside it — an off-base landing, which is eliminatory.
        step = self._servo.update((det.u, det.v), self._height_over_pad())
        if step is None:
            return
        yaw = yaw_of(self.pose)
        dx = step[0] * math.cos(yaw) - step[1] * math.sin(yaw)
        dy = step[0] * math.sin(yaw) + step[1] * math.cos(yaw)
        self.setpoint[0] += dx
        self.setpoint[1] += dy
        self.get_logger().info(
            f"centring on pad {self.target_id}: ({det.u:.0f}, {det.v:.0f}) px "
            f"-> nudging ({dx:+.2f}, {dy:+.2f}) m",
            throttle_duration_sec=2.0)

    def _reject_target(self):
        """Give up on the current candidate and go back to turning."""
        if self.target_id is not None:
            self.blacklist.add(int(self.target_id))
        self.target_id = None
        # A fresh search, not a continuation: the drone is somewhere new, facing
        # a direction it has not searched from, so the turns it already made
        # tell us nothing about what is visible from here.
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
        self._z_hist = []
        # The altitude the descent starts from, so touchdown can require that
        # the vehicle actually left it.
        self._land_entry_z = (self.pose.pose.position.z
                              if self.pose is not None else None)
        self._enter(self.LAND)
        self._set_mode("LAND")

    def _do_land(self):
        """Wait for touchdown. Two independent signals, whichever comes first."""
        # Keep asking until /mavros/state agrees we are in LAND. An acked mode
        # command that ArduPilot then declined would otherwise leave us hovering
        # here until the timeout.
        if (not self.dry_run
                and self.mav_state.mode != "LAND"
                and self._poll_call() != "pending"
                and self._now() - self._last_cmd_t >= self.retry_period):
            self.get_logger().warn("not in LAND yet; re-sending the mode.")
            self._set_mode("LAND")
            return

        # Landed = the FCU disarmed us, or the reported altitude has STOPPED
        # CHANGING.
        #
        # It used to be "z <= 0.5 m", an absolute height above the takeoff
        # plane, and that is what made the vehicle bail out of LAND just before
        # touchdown: descending through 0.5 m satisfied it, land_settle_s later
        # the mission called it landed and moved on to DWELL and TAKEOFF while
        # the vehicle was still in the air — and ArduPilot then rejected the
        # takeoff because it had never landed. The threshold was also wrong for
        # the competition outright: it is measured from the takeoff plane, so a
        # pad higher than the one we left never reaches 0.5 m at all.
        #
        # Stillness has neither problem. It is relative, so it does not care
        # what height the pad is at, and it cannot be satisfied on the way down:
        # a descending vehicle moves far more than land_still_tol_m across the
        # window, a resting one moves only estimator noise.
        # In a DRY RUN the vehicle is disarmed for the whole run by
        # construction, so this signal is permanently true and would declare
        # touchdown the instant LAND was entered — at hover height, writing that
        # height into the pad. Ignore it there and let stillness-plus-descent
        # decide, which is what the person setting the drone down produces.
        disarmed = (not self.dry_run) and (not self.mav_state.armed)
        still = self._z_is_still()
        descended = (self._land_entry_z is not None
                     and self.pose is not None
                     and (self._land_entry_z - self.pose.pose.position.z)
                     >= self.min_descent)

        # No extra debounce: _z_is_still already demands a FULL land_settle_s
        # window of stillness before it returns true, and a disarm is definitive.
        if disarmed or (still and descended):
            z = self.pose.pose.position.z if self.pose else 0.0
            why = "disarmed" if disarmed else "descended and stopped"
            if self.landing_for == self.LAND_PAD:
                self.landed_count += 1
                self.get_logger().info(
                    f"LANDED on base #{self.landed_count} of "
                    f"{self.target_bases} — resting at z={z:.2f} m ({why}).")
                self._mark_visited(z)
            else:
                self.get_logger().info(
                    f"LANDED ({self.landing_for}) at z={z:.2f} m ({why}).")
                self._report_landing_anchor()
            self._enter(self.DWELL)
            return

        if self._since_entered() > self.land_timeout:
            self.get_logger().warn(
                f"no touchdown within {self.land_timeout:.0f} s — carrying on "
                "anyway so the mission does not stall here.")
            self._enter(self.DWELL)

    def _report_landing_anchor(self):
        """The one measurement of drift against the WORLD this stack can make.

        Everything else compares two quantities that live in the same drifting
        frame. hydrone_localization.landmark was built to correct the pose from
        re-observed pads and MEASURED, on 2026-08-27, that it cannot: the map
        entry and the fresh detection are both projected through the same pose,
        so when that pose walks 7 m they walk together and their difference is
        noise. The anchor there is worse — `map` is the EKF's own frame and the
        takeoff base was registered at the vehicle's position in it, so the
        difference is zero by construction. A frame's drift is not observable
        from inside it.

        This is observable, because the evidence is not a projection. The
        vehicle is PHYSICALLY resting on the base it armed from. Its pose
        should therefore read `home`. Whatever it reads instead is the
        accumulated error, measured by contact — no camera, no depth, no
        association, no ambiguity.

        Reported, not applied, and for the same reason the landmark node is an
        observer: /zed/zed_node/odom is what the EKF flies on with GPS off, and
        a correction injected there on the strength of an unmeasured idea is an
        aircraft flying to the wrong place. What this produces is the number
        that has to be checked against the odom_error CSV's `err_norm` first —
        they are the same quantity, so the comparison is arithmetic rather than
        opinion. It also arrives once per attempt, at the end, which is the
        right cadence to trust before it is the right cadence to steer on.
        """
        if self.home is None or self.pose is None:
            return
        dx = self.pose.pose.position.x - self.home[0]
        dy = self.pose.pose.position.y - self.home[1]
        self.get_logger().info(
            f"LANDING ANCHOR: resting on the takeoff base, which is at "
            f"({self.home[0]:.2f}, {self.home[1]:.2f}); the estimate says "
            f"({self.pose.pose.position.x:.2f}, {self.pose.pose.position.y:.2f}). "
            f"Accumulated drift {math.hypot(dx, dy):.2f} m "
            f"(x {dx:+.2f}, y {dy:+.2f}). Compare against err_norm at the end "
            f"of the odom_error CSV — they are the same quantity.")

    def _z_is_still(self) -> bool:
        """Has the reported altitude stopped moving?

        Peak-to-peak z over the last `land_settle_s`, compared against
        `land_still_tol_m`. Returns False until the window is actually full, so
        entering LAND cannot read as "already stopped".
        """
        if self.pose is None:
            return False
        now = self._now()
        self._z_hist.append((now, self.pose.pose.position.z))

        # Drop what has aged out, but KEEP the first sample at or before the
        # cutoff — trimming to exactly the window would leave a span of
        # land_settle minus one sample period, which never reaches the length
        # the check below asks for.
        cutoff = now - self.land_settle
        while len(self._z_hist) > 1 and self._z_hist[1][0] <= cutoff:
            self._z_hist.pop(0)

        if now - self._z_hist[0][0] < self.land_settle:
            # Not a full window yet: entering LAND must not read as stopped.
            return False
        zs = [z for _, z in self._z_hist]
        return (max(zs) - min(zs)) <= self.land_still_tol

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
        if self.cli_mode is None:      # dry run
            self.get_logger().info(f"[dry] would set mode {mode}.")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self._start_call("mode", self.cli_mode, req)

    def _start_call(self, tag: str, client, request):
        # The last gate. In dry run the FCU clients are None, so a call site
        # that forgot its own guard lands here and is refused rather than
        # reaching MAVROS.
        if client is None:
            self.get_logger().warn(
                f"[dry] refused to send '{tag}' — no client exists in dry run.")
            return
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
        if state == self.TAKEOFF:
            # The altitude this climb starts from. takeoff_alt is a height ABOVE
            # WHATEVER WE ARE STANDING ON, and pose.z is absolute in the FCU's
            # local frame (zeroed at the FIRST takeoff plane), so the two are
            # only comparable on the first climb of a run. See _do_takeoff.
            self._takeoff_start_z = (self.pose.pose.position.z
                                     if self.pose is not None else 0.0)
        if state == self.TRAVEL:
            # Each leg gets its own progress record. Carrying the previous
            # leg's best distance over would make a new leg look stalled from
            # its first tick, because it starts FARTHER from its target than
            # the last one ended from its own.
            self._travel_best = None
            self._travel_progress_t = self._now()
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

    # ────────────────────────────────────────────────────────────────────────
    # Dry run: talking to the person holding the drone
    # ────────────────────────────────────────────────────────────────────────

    def _pilot_cue(self):
        """Once a second, say what the mission wants the human to do NOW.

        Derived entirely from the live state, setpoint and pose rather than
        emitted once at each transition: the person has both hands on a drone
        and is not watching the moment a line scrolls past, and a cue that is
        recomputed cannot drift from what the state machine is actually waiting
        for. Every number in here is measured — the same numbers the transition
        is testing — so when a cue stops changing, that IS the mission being
        stuck, not the reporting.
        """
        pose = self.pose
        cue = None

        if self.state == self.WAIT_FCU:
            if not self.mav_state.connected:
                cue = "waiting for the MAVROS link to the FCU."
            elif pose is None:
                cue = ("waiting for /mavros/local_position/pose. No position "
                       "estimate yet — the EKF needs vision; check that "
                       "/mavros/vision_pose/pose is being published.")
            else:
                cue = "ready."
        elif self.state == self.ARMING:
            left = max(0.0, self.dry_arm_delay - self._since_entered())
            cue = (f"HOLD STILL on the base — treating the vehicle as armed in "
                   f"{left:.0f} s. Nothing will be sent to the FCU.")
        elif self.state == self.REGISTER:
            cue = "HOLD STILL — registering this spot as the takeoff base."
        elif self.state == self.TAKEOFF and pose is not None:
            climbed = pose.pose.position.z - self._takeoff_start_z
            cue = (f"RAISE the drone to {self.takeoff_alt:.2f} m above what it "
                   f"is standing on — {climbed:.2f} m of "
                   f"{self.takeoff_alt:.2f} m so far.")
        elif self.state == self.SETTLE:
            left = max(0.0, self.settle_s - self._since_entered())
            cue = (f"HOLD STILL where you are for {left:.0f} s — the map is not "
                   "read while anything is moving.")
        elif self.state == self.TRAVEL and pose is not None:
            dx = self.setpoint[0] - pose.pose.position.x
            dy = self.setpoint[1] - pose.pose.position.y
            d = math.hypot(dx, dy)
            if d <= self.arrive_tol:
                cue = "HOLD — you are over the target."
            else:
                # A compass-style bearing relative to the nose, because the
                # person is behind the drone and cannot read an ENU heading.
                rel = math.degrees(wrap_pi(math.atan2(dy, dx) - yaw_of(pose)))
                side = "left" if rel > 0 else "right"
                cue = (f"CARRY the drone {d:.2f} m to "
                       f"({self.setpoint[0]:.2f}, {self.setpoint[1]:.2f}) — "
                       f"{abs(rel):.0f} deg to the {side} of where the nose "
                       f"points. Keep it at {self.setpoint[2]:.2f} m and do "
                       "not turn it.")
        elif self.state == self.CONFIRM:
            left = max(0.0, self.confirm_timeout - self._since_entered())
            cue = (f"HOLD THE DRONE OVER THE PAD, lens down — "
                   f"{self._confirm_hits}/{self.confirm_detections} good looks, "
                   f"{left:.0f} s before this candidate is rejected.")
        elif self.state == self.LAND:
            cue = ("PUT THE DRONE DOWN on the pad and let go — touchdown is "
                   "declared when the altitude stops changing.")
        elif self.state == self.DWELL:
            left = max(0.0, self.dwell_s - self._since_entered())
            cue = f"RESTING on the pad — {left:.0f} s, then pick it up again."
        elif self.state == self.DONE:
            cue = "rehearsal complete."
        elif self.state == self.ABORTED:
            cue = "rehearsal aborted."

        if cue is not None:
            self.get_logger().info(f">>> {cue}")

    def _dry_audit(self):
        """Check, once, that nothing else is driving the vehicle either.

        This node publishes no setpoint in dry run — it has no publisher. But a
        rehearsal is usually started while something else was already up, and
        the failure that produces is silent: phase1_real or landing_sites left
        running in another terminal goes on commanding the vehicle for real
        while this window prints >>> lines as though the drone were inert.
        Every launch file in this package warns that two of them fight over
        /mavros/setpoint_position/local; this is that warning, at the one moment
        it can be checked rather than remembered.

        Deliberately NOT a check on whether the vehicle can be armed. It cannot
        be one: MAVROS runs normally here, /mavros/cmd/arming exists, and the
        transmitter was never going through MAVROS in the first place. Arming
        is settled at the flight controller — an arming check the vehicle
        cannot pass — and a green line here would only invite someone to skip
        setting it.
        """
        self._audit_timer.cancel()
        others = self.count_publishers("/mavros/setpoint_position/local")
        if others == 0:
            self.get_logger().info(
                "[dry] nothing is publishing setpoints — this is the only "
                "thing talking to the vehicle, and it is talking to you.")
            return
        self.get_logger().error(
            "════════════════════════════════════════════════════════\n"
            f"  {others} NODE(S) ARE PUBLISHING SETPOINTS.\n"
            "  It is not this one — in dry run this node has no setpoint\n"
            "  publisher at all. Something else is commanding the vehicle\n"
            "  for real: most likely phase1_real.launch.py or\n"
            "  landing_sites.launch.py still running in another terminal.\n"
            "  Stop it before you pick the drone up. The >>> lines below\n"
            "  are NOT the only thing the vehicle is being told.\n"
            "════════════════════════════════════════════════════════")

    def _publish_status(self):
        x = self.pose.pose.position.x if self.pose else float("nan")
        y = self.pose.pose.position.y if self.pose else float("nan")
        z = self.pose.pose.position.z if self.pose else float("nan")
        yaw = math.degrees(yaw_of(self.pose)) if self.pose else float("nan")
        self.pub_status.publish(String(data=(
            ("DRY " if self.dry_run else "")
            + f"state={self.state} mode={self.mav_state.mode} "
            f"armed={self.mav_state.armed} "
            f"x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.0f} "
            f"landed={self.landed_count}/{self.target_bases} "
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
