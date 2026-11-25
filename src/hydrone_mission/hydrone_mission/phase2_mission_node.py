#!/usr/bin/env python3
"""
phase2_mission_node — Fase 2: carry three first-aid kits, then fly home.

THE STRATEGY ONLY. Every "how" is a block (docs/ARQUITETURA.md):

  fly            hydrone_controller.vehicle.Vehicle   (arm, takeoff, land, setpoint)
  grab/release   hydrone_controller.gripper.Gripper   (sim: ardubridge; real: AX-12A)
  legs           /hydrone/nav/navigate_to             (nav_node: A* over the octomap)
  descend        hydrone_nav.visual_descent           (centre by camera, then LAND)
  where/order    hydrone_nav.arena + route.plan_deliveries
  the rest       hydrone_mission.phase2_core          (ROS-free, tested)

Where the bases are: one parameter per base (pickup_1..3, delivery_1..3),
falling back to the layout file; `kits` (default 3) keeps only slots 1..kits.
If ANY of those is unknown the drone first flies
Phase 1's map sweep and fills the gaps from pad_map (kit on it = pickup).

The cycle, per kit, in the order route.plan_deliveries picks:

  GOTO pickup -> DESCEND on the KIT -> ALIGN (turn the kit along the gripper)
  -> TOUCH (kit between the legs, legs on the base) -> LAND -> GRAB
  -> ARMING/TAKEOFF -> turn back -> GOTO delivery -> DESCEND on the BASE
  -> LAND -> RELEASE -> ARMING/TAKEOFF -> next kit

then GOTO home -> LAND -> DONE.

ONE HEADING. The heading the vehicle armed with is held everywhere except
over a kit, which may lie at any angle (rules) while the gripper only closes
across it: the drone turns there, standing still, and turns back before the
next leg. Translation and rotation never happen together.

Rules this encodes (docs/REGRAS-CBR-2026.pdf, Fase 2): every landing gear part
inside the base on pickup and delivery; one kit at a time; landing off a base
ends the attempt; 10 min per attempt on the VEHICLE's clock (/clock in sim);
an autonomous return after >= 1 delivery doubles the score; the task log.
An arm, disarm or mode change the mission did not ask for (the RC is
kill-only, and a human may send it home) ends the run: never re-armed.
"""

import math
import os
import statistics
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hydrone_msgs.action import NavigateTo
from hydrone_msgs.msg import KitAxis, PadDetection, PadMap
from hydrone_msgs.srv import RegisterTakeoffBase

from hydrone_controller.gripper import Gripper
from hydrone_controller.vehicle import Job, Vehicle
from hydrone_mission import phase2_core as core
from hydrone_nav import coverage, route
from hydrone_nav.arena import arena_to_map, load_layout
from hydrone_nav import precision_landing as pl
from hydrone_nav.precision_landing import body_to_world
from hydrone_nav.visual_descent import LOST, READY, VisualDescent


def yaw_of(pose):
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tilt_deg(pose):
    q = pose.pose.orientation
    c = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Frames:
    """All detections of the newest camera frame (one msg per object)."""

    def __init__(self, node, topic, min_conf):
        self.node, self.min_conf = node, min_conf
        self.stamp, self.dets, self.t = None, [], -1e9
        node.create_subscription(PadDetection, topic, self._cb, 20)

    def _cb(self, m):
        st = (m.header.stamp.sec, m.header.stamp.nanosec)
        if st != self.stamp:
            self.stamp, self.dets = st, []
        if m.confidence >= self.min_conf:
            self.dets.append((float(m.u), float(m.v), float(m.radius_px)))
        self.t = self.node.now()

    def fresh(self, max_age):
        return self.dets if self.node.now() - self.t <= max_age else []


class Phase2MissionNode(Node):
    WAIT, OPENING, ARMING, TAKEOFF, SCAN, TURN, GOTO, DESCEND, ALIGN, TOUCH, \
        LAND, ACT, VERIFY, DONE, ABORTED = (
            "WAIT", "OPENING", "ARMING", "TAKEOFF", "SCAN", "TURN", "GOTO",
            "DESCEND", "ALIGN", "TOUCH", "LAND", "ACT", "VERIFY", "DONE",
            "ABORTED")

    def __init__(self, **kwargs):
        super().__init__("phase2_mission", **kwargs)
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        default_layout = os.path.join(
            get_package_share_directory("hydrone_bringup"),
            "config", "phase2_bases.yaml")
        self.layout = load_layout(p("layout_file", "") or default_layout)
        # One per base, arena frame like the layout: "" = the layout's entry,
        # "scan" = unknown, else "[x, y]" / "[x, y, z]" / "{grid: [c, r]}".
        self.pick_text = [p(f"pickup_{i}", "") for i in (1, 2, 3)]
        self.deliv_text = [p(f"delivery_{i}", "") for i in (1, 2, 3)]
        # How many kits are in the arena: only pickup/delivery 1..kits exist,
        # so the scan neither waits for nor fills the slots above it.
        kits = p("kits", 3)
        if not 1 <= kits <= 3:
            raise ValueError(f"kits must be 1..3, got {kits}")
        self.pick_text, self.deliv_text = (self.pick_text[:kits],
                                           self.deliv_text[:kits])
        self.arena_yaw = p("arena_yaw_deg", -90.0)
        # The arena floor in the FCU's z (sim -0.7; real 0 with surface_offset).
        self.ground_z = p("ground_z", -0.7)
        # Cruise height ABOVE THE FLOOR. 3.2 in the sim = the old 2.5 in map z.
        self.cruise_alt = p("cruise_alt", 3.2)
        self.cruise = self.ground_z + self.cruise_alt
        self.climb = p("takeoff_climb_m", 1.0)
        self.budget = p("mission_budget_s", 600.0)
        self.reserve = p("return_reserve_s", 60.0)
        self.fresh_s = p("fresh_detection_s", 1.0)
        self.descent_timeout = p("descent_timeout_s", 45.0)
        self.verify_s = p("verify_s", 3.0)
        self.verify_hits = p("verify_hits", 3)
        # A kit ON the base seen from cruise (~1.7 m over its top) is ~15 px
        # in radius; one HANGING in the gripper right under the lens is ~90.
        # 150 counted the carried kit as "still on the base" — MEASURED
        # 2026-10-09, while the engine log said "segurando: BP_Lipo...".
        self.verify_max_px = p("verify_max_radius_px", 45.0)
        self.verify_centre_px = p("verify_centre_px", 160.0)
        self.pick_retries = p("pick_retries", 1)
        # Pickup, kit lost on the way down: climb back to cruise over the known
        # position and look again this many times, then SKIP the kit. Landing
        # blind on a pickup base can put a leg on an off-centre kit, 1.5 m up.
        self.takeoff_ok_range = p("takeoff_ok_range_m", 0.7)
        self.reacquire_max = p("reacquire_max", 2)
        self._reacquires = 0
        # How a grab is confirmed:
        #   "gripper" — the Gripper result's `holding` (the AX-12A measures it
        #               by load; the sim reads the engine log). Default.
        #   "vision"  — hover over the pickup and check the kit left the base.
        #               Needs a kit detector that does not fire on the base's
        #               reflections: the current 'lipo' model does (2026-10-09
        #               it "saw" a kit while the engine log said the gripper
        #               was holding it). Retrain on ~/Documents/kit_dataset_sim.
        self.pick_check = p("pick_check", "gripper")
        # PICKUP descent below the camera's reach. ArduPilot's LAND from 1.2 m
        # has no visual correction and slid 15-20 cm on every try (2026-10-09:
        # the vehicle touched down beside the kit, the gripper closed on air).
        # Like Black Bee's gripper POC: hold the kit's XY under POSITION
        # control, descend to just above contact, close, THEN land.
        self.pickup_touch = p("pickup_touch", True)
        self.touch_range = p("touch_range_m", 0.22)
        # Close the gripper ONCE, on the ground (ACT), by default. Closing in
        # TOUCH, at touch_range over the kit, puts the real finger tips ~8 cm
        # over the base (rangefinder ~0.17 landed, tips 3 cm over the skids)
        # with a 7 cm kit: above it. The sim's grab volume hid that.
        # True = close in TOUCH and reuse that result in ACT (no 2nd close).
        self.close_in_touch = p("close_in_touch", False)
        # False: wait for /hydrone/mission/start (the arena countdown).
        self.auto_start = p("auto_start", True)
        self.touch_timeout = p("touch_timeout_s", 20.0)
        # Keep correcting the touch point by camera on the way down. The fix
        # taken at 1.2 m comes from ONE frame and missed the kit by 25-54 cm
        # (2026-10-16/27: a reflection picked, or the vehicle drifting below).
        # Between these heights the reflections are gone and the kit is still
        # whole in the frame (MEASURED 2026-10-16: one detection below ~0.9 m,
        # the kit overflows the image under ~0.35 m).
        self.touch_track_max_h = p("touch_track_max_m", 0.9)
        self.touch_track_min_h = p("touch_track_min_m", 0.35)
        self.touch_track_gate = p("touch_track_gate_m", 0.35)
        self.touch_track_gain = p("touch_track_gain", 0.5)
        # Legs on the pickup base: the kit may sit anywhere in the 0.8 m ring
        # but the whole gear must stay on the 1 m base. The drone centre may
        # be this far from the base centre: 0.5 (base) - 0.22 (farthest foot,
        # CAD v3 feet: skids at +-0.18 side, +-0.127 fore-aft) - 0.06 margin.
        # A circle, so it holds at any heading the kit alignment picks.
        self.touch_max_off = p("touch_max_off_m", 0.22)
        # Pulled further in than this, the fingers miss the kit and a foot can
        # land on it: skip the kit instead.
        self.touch_skip_shift = p("touch_skip_shift_m", 0.05)
        # Base centre seen on the way down (base YOLO), kept within this of
        # the known position; < 3 looks -> the known position.
        self.base_fix_gate = p("base_fix_gate_m", 0.6)
        # Turn the kit along the gripper before touching down. The fingers
        # close sideways (gripper_axis_deg 0 = kit nose to tail).
        self.align_kit = p("align_kit", True)
        self.gripper_axis = math.radians(p("gripper_axis_deg", 0.0))
        self.align_min = math.radians(p("align_min_deg", 5.0))
        # Vehicle time: hold still `settle` first (a frame grabbed while the
        # descent settles read 40 deg off, sim 2026-11-23), then up to
        # `align_looks` readings within `look`; look again after the turn and
        # correct once more (align_passes).
        self.align_settle_s = p("align_settle_s", 1.0)
        self.align_look_s = p("align_look_s", 3.0)
        self.align_looks = p("align_looks", 5)
        self.align_passes = p("align_passes", 2)
        self.align_min_coh = p("align_min_coherence", 0.3)
        self.turn_tol = math.radians(p("turn_tol_deg", 4.0))
        self.turn_timeout = p("turn_timeout_s", 12.0)
        # Phase 1's guard: stopped tilted on a base = a foot on something.
        # A takeoff from there flips the drone off the block.
        self.tip_rearm_max = p("tip_rearm_max_deg", 12.0)
        # An FCU that keeps refusing to re-arm on a base was most likely
        # stopped by the kill switch; when that is released, arming would
        # take off in someone's hands.
        self.rearm_timeout = p("rearm_timeout_s", 30.0)
        # The scan (only when a base is unknown): Phase 1's map sweep.
        self.plan_bounds = list(p("plan_bounds",
                                  [-5.0, -5.0, 0.3, 5.0, 5.0, 3.0]))
        self.sweep_side = p("sweep_side", "right")
        self.survey_inset = p("survey_inset_m", 1.2)
        self.sweep_overlap = p("sweep_overlap", 0.25)
        self.sweep_max_surface = p("sweep_max_surface_m", 1.6)
        self.scan_min_obs = p("scan_min_observations", 3)
        self.min_kit_hits = p("scan_min_kit_hits", 3)
        self.descend = VisualDescent(
            descend_to_m=p("visual_descend_to_m", 1.2),
            fine_px=p("visual_fine_px", 40.0),
            gain=p("visual_gain", 0.6),
            axes=tuple(p("visual_axes", [-1.0, -1.0])))
        self._centre_uv = self.descend.target_uv
        # Belly camera position from the vehicle centre, m, body [x forward,
        # y right] — same convention as phase 1's belly_offset_xy. The real
        # drone moves the webcam off the gripper's axis so the carried kit
        # does not cover the base (2026-10-16); the sim camera is on the axis.
        self.belly_offset = tuple(p("belly_offset_xy", [0.0, 0.0]))
        # The task log the rules ask for, as its own file ("" = ROS log only).
        self.task_path = os.path.expanduser(p("task_log", ""))
        self._task_file = None

        self.vehicle = Vehicle(self)
        self.gripper = Gripper(self)
        self.nav = ActionClient(self, NavigateTo, "/hydrone/nav/navigate_to")
        self.bases = Frames(self, p("base_topic", "/hydrone/pads/down/detections"),
                            p("base_confidence", 0.30))
        self.kits = Frames(self, p("kit_topic", "/hydrone/kits/detections"),
                           p("kit_confidence", 0.50))
        # Delivery only: the yellow ring fitted around the kit hanging under
        # the lens, which hides the base from the YOLO (ring_detector.py).
        self.rings = Frames(self, p("ring_topic", "/hydrone/pads/down/ring"),
                            p("ring_confidence", 0.50))
        # Delivery, base not in sight: how close over the layout position the
        # vehicle must hold before it lands blind. The old blind path landed
        # on NavigateTo's 0.25 m arrival — MEASURED 2026-10-09, seed 6: that
        # arrival fired at once and the vehicle landed on the floor ~1.3 m
        # from base A.
        self.blind_xy_tol = p("blind_xy_tol_m", 0.10)

        self.pose, self.range_m, self.cam, self.sim_clock = None, None, None, None
        self.mav, self.pad_map, self.axes_seen = None, None, []
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Range, p("range_topic",
                                          "/mavros/distance_sensor/rangefinder"),
                                 self._cb_range, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/down_cam/camera_info",
                                 lambda m: setattr(self, "cam", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self._cb_clock,
                                 qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state",
                                 lambda m: setattr(self, "mav", m), 10)
        self.create_subscription(PadMap, "/hydrone/pads/map",
                                 lambda m: setattr(self, "pad_map", m), 10)
        self.create_subscription(KitAxis, p("kit_axis_topic",
                                            "/hydrone/kits/axis"),
                                 self._cb_axis, 20)
        self.cli_register = self.create_client(
            RegisterTakeoffBase, "/hydrone/pads/register_takeoff_base")
        self.pub_status = self.create_publisher(String, "/hydrone/mission/status", 10)
        self.create_service(Trigger, "/hydrone/mission/start", self._svc_start)

        self.state, self._since, self._job = self.WAIT, self.now(), None
        self.home = self.home_yaw = self.hold_yaw = None
        self.pick_xy, self.deliv_xy = [], []
        self.tour, self.leg = [], 0          # [(pickup_i, delivery_i)], index
        self.carrying = False
        self.retries = 0
        self.takeoff_tries = 0
        self.target = None                   # ("pickup"|"delivery"|"home", x, y)
        self.after_takeoff = None
        self.t0 = None
        self.delivered = 0
        self._used_stamp = None
        self._blind = False
        self._give_up_visual = False
        self._touch_xy = None
        self._touch_grab = None              # (ok, holding, message) of a TOUCH close
        self._open_after, self._open_hold = None, False
        self._ours = False                   # armed by us, not landed yet
        self._settled = True                 # seen disarmed since our landing
        self._takeoff_z0 = 0.0
        self._base_fix, self._base_stamp = [], None
        self._aligned, self._align_pass = False, 0
        self._turn_to = self._after_turn = self._turn_xyz = None
        self._turn_in = None
        self._scan, self._scan_i, self._scanning = [], 0, False
        self._kits_seen, self._kit_stamp = [], None
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"phase2_mission ready — cruise {self.cruise_alt} m above the "
            f"floor (z {self.cruise:.2f}), kit alignment "
            f"{'on' if self.align_kit else 'off'}, gripper closes "
            f"{'in TOUCH' if self.close_in_touch else 'on the ground'}. "
            f"{'Auto-starting.' if self.auto_start else 'Call /hydrone/mission/start.'}")

    # ── plumbing ────────────────────────────────────────────────────────────
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def mission_now(self):
        """Vehicle time: sim /clock when there is one, wall clock otherwise."""
        return self.sim_clock if self.sim_clock is not None else self.now()

    def _cb_clock(self, m):
        self.sim_clock = m.clock.sec + m.clock.nanosec * 1e-9

    def _cb_range(self, m):
        self.range_m = float(m.range) if math.isfinite(m.range) else None

    def _cb_axis(self, m):
        self.axes_seen.append((self.mission_now(), m.u, m.v, m.angle,
                               m.coherence))
        del self.axes_seen[:-50]

    def _enter(self, state):
        if state != self.state:
            self.get_logger().info(f"[{self.state} -> {state}]")
        if state == self.TAKEOFF and self.pose is not None:
            self._takeoff_z0 = self.pose.pose.position.z
        self.state, self._since, self._job = state, self.now(), None

    def _elapsed(self):
        return self.now() - self._since

    def _clock_note(self):
        if self.t0 is None:
            return ""
        return (f"[mission clock {self.mission_now() - self.t0:.0f} s of "
                f"{self.budget:.0f} s]")

    def _task(self, text):
        """The task log the rules ask for (kit picked, delivered, ...)."""
        line = f"TASK: {text} {self._clock_note()}"
        self.get_logger().info(line)
        if not self.task_path:
            return
        try:
            if self._task_file is None:
                os.makedirs(os.path.dirname(self.task_path) or ".",
                            exist_ok=True)
                self._task_file = open(self.task_path, "a")
            self._task_file.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} {line}\n")
            self._task_file.flush()
        except OSError as e:
            self.get_logger().error(f"task log {self.task_path}: {e}",
                                    throttle_duration_sec=30.0)

    def _map(self, p_arena):
        return arena_to_map(p_arena, self.home, self.layout.takeoff,
                            self.arena_yaw)

    def _hold(self):
        p = self.pose.pose.position
        self.vehicle.setpoint(p.x, p.y, p.z, self.hold_yaw)

    def _svc_start(self, _req, res):
        self.auto_start = True
        res.success = self.state == self.WAIT
        res.message = ("starting" if res.success
                       else f"already running ({self.state})")
        return res

    def _open_then(self, after, hold=False):
        """Open the gripper, WAIT for it, then `after()`. hold: airborne."""
        self._open_after, self._open_hold = after, hold
        self._enter(self.OPENING)

    def _do_opening(self):
        if self._open_hold and self.pose is not None:
            self._hold()
        if self._job is None:
            self._job = self.gripper.open()
            return
        if not self._job.done:
            return
        if not self._job.ok:
            self.get_logger().warn(f"gripper open failed: {self._job.message}")
        after, self._open_after = self._open_after, None
        after()

    def _abort(self, why):
        self._task(f"ABORTED: {why}. The mission will not arm again.")
        self._enter(self.ABORTED)

    # ── tick ────────────────────────────────────────────────────────────────
    def _tick(self):
        if self.state in (self.DONE, self.ABORTED):
            return
        if self._taken_over():
            return
        self._check_budget()
        if self._scanning:
            self._collect_kits()
        getattr(self, f"_do_{self.state.lower()}")()
        self.pub_status.publish(String(data=(
            f"state={self.state} kit={self.leg + 1}/{len(self.tour) or '?'} "
            f"carrying={self.carrying} delivered={self.delivered}")))

    def _taken_over(self):
        if self.mav is None or self.pose is None:
            return False
        airborne = self.pose.pose.position.z - self._takeoff_z0 > 0.3
        if not self._ours and not self.mav.armed:
            self._settled = True
        why = core.takeover(self.state, self.mav.armed, self.mav.mode,
                            ours=self._ours, airborne=airborne,
                            settled=self._settled)
        if why is None:
            return False
        self._abort(f"{why} — safety pilot, failsafe or crash")
        return True

    def _check_budget(self):
        if self.t0 is None or self.target and self.target[0] == "home":
            return
        if self.state in (self.LAND, self.ACT, self.OPENING):
            return
        if self.mission_now() - self.t0 < self.budget - self.reserve:
            return
        self.get_logger().warn(f"BUDGET: going home {self._clock_note()}")
        if self._job is not None:
            self._job.cancel()
        self._scanning = False
        self._go_home()

    # ── states ──────────────────────────────────────────────────────────────
    def _do_wait(self):
        if self.pose is None or not (self.vehicle.ready()
                                     and self.gripper.ready()
                                     and self.nav.server_is_ready()):
            self.get_logger().info(
                "waiting for pose, controller, gripper and nav...",
                throttle_duration_sec=5.0)
            return
        if not self.auto_start:
            self.get_logger().info("phase2_mission ready, waiting. Call "
                                   "/hydrone/mission/start.",
                                   throttle_duration_sec=10.0)
            return
        p = self.pose.pose.position
        self.home, self.home_yaw = (p.x, p.y), yaw_of(self.pose)
        self.hold_yaw = self.home_yaw
        grid = self.layout.grid
        pick = core.merge_positions(self.pick_text, self.layout.pickup, grid)
        deliv = core.merge_positions(self.deliv_text, self.layout.delivery,
                                     grid)
        self.pick_xy = [self._map(q[:2]) if q else None for q in pick]
        self.deliv_xy = [self._map(q[:2]) if q else None for q in deliv]
        missing = ([f"pickup {i + 1}" for i, q in enumerate(pick) if q is None]
                   + [f"delivery {i + 1}" for i, q in enumerate(deliv)
                      if q is None])
        if missing:
            self._task(f"unknown: {', '.join(missing)} — scanning the arena "
                       "(Phase 1's map sweep) before the first kit")
            self.after_takeoff = self._start_scan
        else:
            self._plan_tour()
            self.after_takeoff = self._next_target
        # Arm only once the gripper is confirmed open (the AX-12A homes itself
        # at boot and may sit anywhere).
        self._open_then(lambda: self._enter(self.ARMING))

    def _plan_tour(self):
        pi = [i for i, q in enumerate(self.pick_xy) if q is not None]
        di = [i for i, q in enumerate(self.deliv_xy) if q is not None]
        tour = route.plan_deliveries(self.home, [self.pick_xy[i] for i in pi],
                                     [self.deliv_xy[i] for i in di])
        self.tour = [(pi[a], di[b]) for a, b in tour]
        self.get_logger().info(
            "home = ({:.2f}, {:.2f}); tour (pickup->delivery): {}".format(
                *self.home, ", ".join(f"{a + 1}->{chr(65 + b)}"
                                      for a, b in self.tour) or "none"))

    def _do_arming(self):
        if self._job is None and self.t0 is not None:
            # Re-arming on a base: the guards that keep a kill a kill.
            if self.tip_rearm_max > 0.0 and tilt_deg(self.pose) > self.tip_rearm_max:
                self._abort(f"stopped {tilt_deg(self.pose):.0f} deg tilted on a "
                            "base (a foot on the kit or the edge); a takeoff "
                            "from here flips")
                return
            if self._elapsed() > self.rearm_timeout:
                self._abort(f"FCU refused to re-arm for {self.rearm_timeout:.0f}"
                            " s (kill switch?)")
                return
        if self._job is None:
            self._job = self.vehicle.arm()
        elif self._job.done:
            if self._job.ok:
                self._ours = True
                if self.t0 is None:
                    self._register_takeoff_base()
                self._enter(self.TAKEOFF)
            else:
                self.get_logger().warn(f"arm: {self._job.message} — retrying",
                                       throttle_duration_sec=2.0)
                self._job = None

    def _register_takeoff_base(self):
        """pad_map maps nothing before this, and never counts home as a base."""
        if not self.cli_register.service_is_ready():
            self.get_logger().warn("pad_map not up: the takeoff base is not "
                                   "registered (only matters for the scan).")
            return
        req = RegisterTakeoffBase.Request()
        p = self.pose.pose.position
        req.position.x, req.position.y, req.position.z = p.x, p.y, p.z
        self.cli_register.call_async(req)

    def _do_takeoff(self):
        if self._job is None:
            self._job = self.vehicle.takeoff(self.climb, hold_z=self.cruise)
            return
        if not self._job.done:
            return
        # ArduPilot climbs to `climb` over the surface under the rangefinder, so
        # from a base it can stop short of the climb the controller counts and
        # the job fails with the vehicle hovering fine. MEASURED 2026-10-30,
        # seed 1: z 0.71 -> 1.50, range 1.00, level; four "failures", no return.
        airborne = (self.range_m is not None
                    and self.range_m >= self.takeoff_ok_range)
        if self._job.ok or airborne:
            if self.t0 is None:
                self.t0 = self.mission_now()
                self.get_logger().info(
                    f"MISSION CLOCK started "
                    f"({'sim /clock' if self.sim_clock is not None else 'wall'}"
                    f"), budget {self.budget:.0f} s.")
            self.takeoff_tries = 0
            self.after_takeoff()
            return
        self.takeoff_tries += 1
        q = self.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.get_logger().warn(
            f"takeoff failed ({self._job.message}): roll "
            f"{math.degrees(roll):.0f} pitch {math.degrees(pitch):.0f} deg, "
            f"z {self.pose.pose.position.z:.2f}, range {self.range_m}")
        if self.takeoff_tries > 3:
            self._abort(f"takeoff refused 3x: {self._job.message}")
        else:
            self._enter(self.ARMING)

    # ── the scan: only when a base is unknown ──────────────────────────────
    def _swath(self):
        """Belly footprint over the tallest surface (Phase 1's _sweep_swath_m)."""
        if self.cam is None:
            return None
        agl = self.cruise - self.ground_z - self.sweep_max_surface
        if agl < 0.3:
            return None
        across, along = coverage.ground_swath(
            float(self.cam.k[0]), float(self.cam.k[4]), self.cam.width,
            self.cam.height, agl)
        return min(across, along) if across > 0.0 and along > 0.0 else None

    def _start_scan(self):
        p = self.pose.pose.position
        swath = self._swath()
        self._scan = core.scan_path(
            self.plan_bounds, self.home_yaw, (p.x, p.y), side=self.sweep_side,
            inset_m=self.survey_inset, z=self.cruise, swath_m=swath,
            overlap=self.sweep_overlap)
        self._scan_i, self._scanning, self._kits_seen = 0, True, []
        self._task(f"scan: {len(self._scan)} setpoints, "
                   + (f"lanes for a {swath:.2f} m belly swath" if swath
                      else "perimeter only (no belly CameraInfo / too low)"))
        self._enter(self.SCAN)

    def _collect_kits(self):
        """Every kit the belly sees on the scan, in map xy (pickup evidence)."""
        if self.kits.stamp == self._kit_stamp or self.range_m is None:
            return
        self._kit_stamp = self.kits.stamp
        h = self.range_m
        self.descend.target_uv = pl.target_uv(self._centre_uv, self.belly_offset,
                                              self._fx(), h)
        for u, v, _ in self.kits.fresh(self.fresh_s):
            self._kits_seen.append(self._kit_world_xy((u, v), h))

    def _pads(self):
        return [(pd.position.x, pd.position.y, pd.height, pd.observations,
                 pd.is_takeoff_base)
                for pd in (self.pad_map.pads if self.pad_map else [])]

    def _scan_found_all(self):
        """Every missing base already found on clear evidence: stop early."""
        pick, deliv, _ = core.fill_missing(
            self.pick_xy, self.deliv_xy, self._pads(), self._kits_seen,
            home=self.home, ground_z=self.ground_z, min_obs=self.scan_min_obs,
            min_kit_hits=self.min_kit_hits, strict=True)
        return None not in pick + deliv

    def _do_scan(self):
        if self._job is None:
            if self._scan_i >= len(self._scan):
                self._end_scan()
                return
            if self._scan_i > 0 and self._scan_found_all():
                self._task(f"scan: every missing base found — stopping at "
                           f"setpoint {self._scan_i}/{len(self._scan)}")
                self._end_scan()
                return
            x, y, z, yaw = self._scan[self._scan_i]
            p = self.pose.pose.position
            if (math.hypot(x - p.x, y - p.y) < 0.3
                    and abs(core.wrap_pi(yaw - yaw_of(self.pose))) > self.turn_tol):
                self._turn(yaw, self._scan_next)        # a corner: turn standing
                return
            g = NavigateTo.Goal(yaw=float(yaw), tolerance=0.25)
            g.position.x, g.position.y, g.position.z = float(x), float(y), float(z)
            self.hold_yaw = yaw
            self._job = Job(self.nav, g)
            return
        if not self._job.done:
            return
        if not self._job.ok:
            self.get_logger().warn(f"scan setpoint {self._scan_i}: "
                                   f"{self._job.message} — skipping it")
        self._scan_i += 1
        self._job = None

    def _scan_next(self):
        self._scan_i += 1
        self._enter(self.SCAN)

    def _end_scan(self):
        self._scanning = False
        pads = self._pads()
        was_p = [q is None for q in self.pick_xy]
        was_d = [q is None for q in self.deliv_xy]
        self.pick_xy, self.deliv_xy, notes = core.fill_missing(
            self.pick_xy, self.deliv_xy, pads, self._kits_seen,
            home=self.home, ground_z=self.ground_z, min_obs=self.scan_min_obs,
            min_kit_hits=self.min_kit_hits)
        for n in notes:
            self.get_logger().info(f"scan: {n}")
        for kind, slots, was in (("pickup", self.pick_xy, was_p),
                                 ("delivery", self.deliv_xy, was_d)):
            for i, q in enumerate(slots):
                if was[i]:
                    self._task(f"scan: {kind} {i + 1} "
                               + (f"FOUND at ({q[0]:.2f}, {q[1]:.2f})" if q
                                  else "NOT found — dropped"))
        self._plan_tour()
        self._next_target()

    # ── legs ────────────────────────────────────────────────────────────────
    def _next_target(self):
        """Decide where the next leg goes, from what is being carried."""
        if self.leg >= len(self.tour):
            self._go_home()
            return
        pi, di = self.tour[self.leg]
        if self.carrying:
            self._leg_to(("delivery", *self.deliv_xy[di]))
        else:
            self._new_pickup()
            self._leg_to(("pickup", *self.pick_xy[pi]))

    def _new_pickup(self):
        self._aligned, self._align_pass, self._base_fix = False, 0, []
        self._touch_grab = None

    def _go_home(self):
        self._leg_to(("home", *self.home))

    def _leg_to(self, target):
        """Every leg flies on the armed heading: turn back first, standing."""
        self.target = target
        if abs(core.wrap_pi(yaw_of(self.pose) - self.home_yaw)) > math.radians(10):
            self._turn(self.home_yaw, lambda: self._enter(self.GOTO))
        else:
            self.hold_yaw = self.home_yaw
            self._enter(self.GOTO)

    def _turn(self, yaw, then):
        p = self.pose.pose.position
        self._turn_to, self._after_turn = yaw, then
        self._turn_xyz, self._turn_in = (p.x, p.y, p.z), None
        self._enter(self.TURN)
        # The vehicle's clock: a turn takes vehicle time (the sim runs ~0.35x)
        self._turn_t0 = self.mission_now()

    def _do_turn(self):
        x, y, z = self._turn_xyz
        self.vehicle.setpoint(x, y, z, self._turn_to)
        err = abs(core.wrap_pi(yaw_of(self.pose) - self._turn_to))
        t = self.mission_now()
        if err > self.turn_tol:
            self._turn_in = None
        elif self._turn_in is None:
            self._turn_in = t
        settled = self._turn_in is not None and t - self._turn_in > 0.5
        if not settled and t - self._turn_t0 < self.turn_timeout:
            return
        if not settled:
            self.get_logger().warn(f"turn: still {math.degrees(err):.0f} deg "
                                   "off after the timeout — going on")
        self.hold_yaw = self._turn_to
        then, self._after_turn = self._after_turn, None
        then()

    def _do_goto(self):
        kind, x, y = self.target
        if self._job is None:
            g = NavigateTo.Goal(yaw=float(self.home_yaw), tolerance=0.25)
            g.position.x, g.position.y = float(x), float(y)
            g.position.z = float(self.cruise)
            self._job = Job(self.nav, g)
            self.get_logger().info(f"flying to {kind} ({x:.2f}, {y:.2f})")
            return
        if not self._job.done:
            return
        if not self._job.ok:
            self.get_logger().warn(f"leg to {kind}: {self._job.message} — retrying")
            self._job = None
            return
        self.descend.reset()
        self._give_up_visual = False
        blind, self._blind = self._blind, False
        self._enter(self.LAND if kind == "home" or blind else self.DESCEND)

    def _fx(self):
        return float(self.cam.k[0]) if self.cam is not None else 0.0

    def _do_descend(self):
        """Centre on the KIT (pickup) or the BASE (delivery) while descending."""
        kind, tx, ty = self.target
        h = self.range_m if self.range_m is not None else self.cruise_alt
        if kind == "pickup":
            dets = [(u, v) for u, v, _ in self.kits.fresh(self.fresh_s)]
        else:
            dets = [(u, v) for u, v, _ in self.bases.fresh(self.fresh_s)
                    + self.rings.fresh(self.fresh_s)]
        self.descend.target_uv = pl.target_uv(self._centre_uv, self.belly_offset,
                                              self._fx(), h)
        if kind == "pickup":
            self._fix_base(tx, ty, h)
        if kind == "delivery" and (not dets or self._give_up_visual):
            # The kit covers the middle of the image: from cruise the ring is
            # smaller than the kit and stays hidden until ~1.6 m (MEASURED
            # 2026-10-16). Come down on the KNOWN position and keep looking.
            self.descend.reset()
            self._descend_blind(tx, ty, h)
            return
        step = self.descend.update(dets, h, self._fx() or None, self.now())
        if step.phase == READY:
            self._reacquires = 0
            self.get_logger().info(
                f"{kind}: centred at {h:.2f} m ({step.err_px:.0f} px) — "
                + ("touching down on it" if kind == "pickup"
                   and self.pickup_touch else "landing"))
            if kind == "pickup" and self.pickup_touch and step.picked:
                if self.align_kit and not self._aligned:
                    self._align_t0 = self.mission_now()
                    self._enter(self.ALIGN)
                    return
                if self._set_touch(self._kit_world_xy(step.picked, h)):
                    self._enter(self.TOUCH)
            elif kind == "delivery" and self.pickup_touch and step.picked:
                # Same controlled descent onto the BASE centre: LAND from 1.2 m
                # slid to a base edge and tipped at the next spin-up (sim,
                # 2026-11-24, 33 deg, the tilt guard caught it).
                self._touch_xy = self._kit_world_xy(step.picked, h)
                self._enter(self.TOUCH)
            else:
                self._enter(self.LAND)
            return
        if kind == "delivery" and self._elapsed() > self.descent_timeout:
            self.get_logger().warn("delivery: camera timed out — finishing "
                                   "on the known position.")
            self._give_up_visual = True
            return
        if (kind == "pickup" and self._reacquires < self.reacquire_max
                and (step.phase == LOST or self._elapsed() > self.descent_timeout)):
            self._reacquires += 1
            self._task(f"pickup: kit lost at {h:.2f} m — climbing to look again "
                       f"({self._reacquires}/{self.reacquire_max})")
            self._enter(self.GOTO)
            return
        if kind == "pickup" and (step.phase == LOST
                                 or self._elapsed() > self.descent_timeout):
            self._reacquires = 0
            self._skip_kit(f"not seen after {self.reacquire_max} more looks")
            return
        if step.phase == LOST or self._elapsed() > self.descent_timeout:
            # Positions are KNOWN (rules): land on the layout position rather
            # than hover forever — but never off it, so re-centre there first.
            self.get_logger().warn(
                f"{kind}: camera {'lost it' if step.phase == LOST else 'timed out'}"
                f" at {h:.2f} m — landing on the known position "
                f"({tx:.2f}, {ty:.2f}).")
            # Fly back over it at cruise (the nudges may have moved us) and
            # land on arrival, blind.
            self._blind = True
            self._reacquires = 0
            self._enter(self.GOTO)
            return
        stamp = (self.kits.stamp if kind == "pickup"
                 else (self.bases.stamp, self.rings.stamp))
        if stamp != self._used_stamp and step.picked is not None:
            self._used_stamp = stamp
            wx, wy = body_to_world((step.dx, step.dy), yaw_of(self.pose))
            p = self.pose.pose.position
            self.vehicle.setpoint(p.x + wx, p.y + wy, p.z + step.dz,
                                  self.hold_yaw)

    def _fix_base(self, tx, ty, h):
        """Pickup: where the BASE is, from its detections while it is whole
        in the frame (above the READY height). The legs are kept on it."""
        if (h <= self.descend.descend_to + 0.1
                or self.bases.stamp == self._base_stamp):
            return
        self._base_stamp = self.bases.stamp
        seen = [self._kit_world_xy((u, v), h)
                for u, v, _ in self.bases.fresh(self.fresh_s)]
        seen = [w for w in seen
                if math.hypot(w[0] - tx, w[1] - ty) <= self.base_fix_gate]
        if seen:
            self._base_fix.append(min(
                seen, key=lambda w: math.hypot(w[0] - tx, w[1] - ty)))

    def _base_centre(self):
        if len(self._base_fix) >= 3:
            return (statistics.median(w[0] for w in self._base_fix),
                    statistics.median(w[1] for w in self._base_fix))
        _, tx, ty = self.target
        return (tx, ty)

    def _set_touch(self, kit_xy):
        """Touch point = the kit, kept where every foot stays on the base."""
        c = self._base_centre()
        q = core.clamp_to_disc(kit_xy, c, self.touch_max_off)
        shift = math.hypot(q[0] - kit_xy[0], q[1] - kit_xy[1])
        off = math.hypot(kit_xy[0] - c[0], kit_xy[1] - c[1])
        if shift > self.touch_skip_shift:
            self._skip_kit(f"{off * 100:.0f} cm from the base centre: the "
                           "legs would leave the base or land on it")
            return False
        self._touch_xy = q
        if shift > 0.01:
            self.get_logger().info(f"pickup: touch point pulled "
                                   f"{shift * 100:.0f} cm towards the base centre")
        return True

    def _skip_kit(self, why):
        pi, _ = self.tour[self.leg]
        self._task(f"kit {pi + 1}: SKIPPED — {why}")
        self.carrying = False
        self.leg += 1
        self.retries = 0
        self._next_target()

    def _do_align(self):
        """Hold over the kit, read its angle, turn it along the gripper."""
        self._hold()
        u0, v0 = self.descend.target_uv          # the pixel under the centre
        t0 = self._align_t0 + self.align_settle_s
        looks = [a for t, u, v, a, c in self.axes_seen
                 if t >= t0 and c >= self.align_min_coh
                 and math.hypot(u - u0, v - v0) <= 120.0]
        if (len(looks) < self.align_looks
                and self.mission_now() - t0 < self.align_look_s):
            return
        pi, _ = self.tour[self.leg]
        self._align_pass += 1
        axis = core.robust_axis(looks)
        self.descend.reset()
        if axis is None:
            self._aligned = True
            self._task(f"kit {pi + 1}: no clear angle ({len(looks)} looks) — "
                       "touching down on the current heading")
            self._enter(self.DESCEND)
            return
        d = core.turn_to_axis(axis, self.descend.axes, self.gripper_axis)
        if abs(d) <= self.align_min:
            self._aligned = True
            self._enter(self.DESCEND)
            return
        self._aligned = self._align_pass >= self.align_passes
        self._task(f"kit {pi + 1}: lies {math.degrees(d):.0f} deg off the "
                   f"gripper — turning to it (pass {self._align_pass})")
        self._turn(yaw_of(self.pose) + d, lambda: self._enter(self.DESCEND))

    def _descend_blind(self, tx, ty, h):
        """Delivery without the base in sight: down to visual_descend_to_m
        over the layout position, then land once held within blind_xy_tol."""
        p = self.pose.pose.position
        if h > self.descend.descend_to + 0.05:
            self.vehicle.setpoint(tx, ty, p.z - min(0.3, h - self.descend.descend_to),
                                  self.hold_yaw)
            return
        self.vehicle.setpoint(tx, ty, p.z, self.hold_yaw)
        if math.hypot(p.x - tx, p.y - ty) <= self.blind_xy_tol:
            self.get_logger().warn(
                f"delivery: base not seen down to {h:.2f} m — landing on the "
                f"known position ({tx:.2f}, {ty:.2f}).")
            if self.pickup_touch:
                self._touch_xy = (tx, ty)
                self._enter(self.TOUCH)
            else:
                self._enter(self.LAND)

    def _kit_world_xy(self, uv, h):
        """Where the kit IS, from its last pixel and the height over it.

        The same fixed belly mapping visual_descent uses, at gain 1: the full
        offset, not a fraction of it.
        """
        u0, v0 = self.descend.target_uv
        m = max(h, 0.2) / (self._fx() or 320.0)
        ax_u, ax_v = self.descend.axes
        bx, by = ax_v * (uv[1] - v0) * m, ax_u * (uv[0] - u0) * m
        wx, wy = body_to_world((bx, by), yaw_of(self.pose))
        p = self.pose.pose.position
        return p.x + wx, p.y + wy

    def _track_kit(self):
        """Pull the touch point towards the kit seen in the newest frame."""
        h = self.range_m
        if (h is None or not self.touch_track_min_h <= h <= self.touch_track_max_h
                or self.kits.stamp == self._used_stamp):
            return
        self._used_stamp = self.kits.stamp
        self.descend.target_uv = pl.target_uv(self._centre_uv, self.belly_offset,
                                              self._fx(), h)
        kx, ky = self._touch_xy
        seen = [self._kit_world_xy((u, v), h)
                for u, v, _ in self.kits.fresh(self.fresh_s)]
        seen = [w for w in seen
                if math.hypot(w[0] - kx, w[1] - ky) <= self.touch_track_gate]
        if not seen:
            return
        wx, wy = min(seen, key=lambda w: math.hypot(w[0] - kx, w[1] - ky))
        g = self.touch_track_gain
        self._touch_xy = core.clamp_to_disc(
            (kx + g * (wx - kx), ky + g * (wy - ky)), self._base_centre(),
            self.touch_max_off)
        self.get_logger().info(
            f"pickup: kit at {h:.2f} m is {math.hypot(wx - kx, wy - ky) * 100:.0f}"
            " cm from the touch point — correcting",
            throttle_duration_sec=0.5)

    def _do_touch(self):
        """Descend on the kit's (pickup) or the base's (delivery) XY under
        position control to just above contact, then LAND."""
        pickup = self.target[0] == "pickup"
        if self._job is None and pickup:
            self._track_kit()
        kx, ky = self._touch_xy
        p = self.pose.pose.position
        touched = (self.range_m is not None
                   and self.range_m <= self.touch_range)
        if self._job is None and (touched
                                  or self._elapsed() > self.touch_timeout):
            self.vehicle.setpoint(kx, ky, p.z, self.hold_yaw)   # hold here
            h = self.range_m if self.range_m is not None else -1
            if not (pickup and self.close_in_touch):
                self.get_logger().info(f"{self.target[0]}: at {h:.2f} m — "
                                       "landing")
                self._enter(self.LAND)
                return
            self._job = self.gripper.close()
            self.get_logger().info(f"pickup: at {h:.2f} m over the kit — "
                                   "closing the gripper, then landing")
            return
        if self._job is None:
            # The controller limits the speed; asking 0.3 m below keeps it
            # coming down steadily without overshooting the contact.
            self.vehicle.setpoint(kx, ky, p.z - 0.3, self.hold_yaw)
            return
        if self._job.done:
            r = self._job.result
            self._touch_grab = (self._job.ok,
                                bool(r.holding) if r is not None else False,
                                self._job.message)
            self._enter(self.LAND)

    def _do_land(self):
        if self._job is None:
            self._job = self.vehicle.land(disarm=True)
            return
        if not self._job.done:
            return
        self._ours, self._settled = False, False
        r = self._job.result
        z = r.z if r is not None else float("nan")
        kind = self.target[0]
        if kind == "home":
            self._task(f"home — {self.delivered} kit(s) delivered; landed at "
                       f"z={z:.2f}")
            self._enter(self.DONE)
            return
        if kind == "delivery" and not self._job.ok:
            # The FCU never agreed it was down (seed 6: on the floor beside
            # the base). Releasing there drops the kit off the base (-5).
            # Keep it, climb, and come down on the base again.
            self._task(f"delivery: landing NOT confirmed ({self._job.message})"
                       " — keeping the kit, trying again")
            self.after_takeoff = self._next_target
            self._enter(self.ARMING)
            return
        self._enter(self.ACT)

    def _do_act(self):
        kind = self.target[0]
        pi, di = self.tour[self.leg]
        grab = None
        if kind == "pickup" and self._touch_grab and self._touch_grab[0]:
            # Closed in TOUCH already: a 2nd close on the ground would drop
            # the AX-12A's hold torque for a moment.
            grab, self._touch_grab = self._touch_grab, None
        elif self._job is None:
            self._job = (self.gripper.close() if kind == "pickup"
                         else self.gripper.open())
            return
        elif not self._job.done:
            return
        if kind == "pickup":
            if grab is None:
                r = self._job.result
                grab = (self._job.ok, bool(r.holding) if r is not None
                        else False, self._job.message)
            held = grab[0] and grab[1]
            self._task(f"kit {pi + 1}: gripper closed on pickup {pi + 1} "
                       f"({grab[2]}); verifying from the air")
            self.carrying = bool(held)
            if self.pick_check == "vision":
                self.after_takeoff = self._start_verify
            elif held:
                self._task(f"kit {pi + 1}: PICKED UP (gripper reports holding)")
                self.after_takeoff = self._next_target
            else:
                self.after_takeoff = self._grab_failed
        else:
            self.carrying = False
            self.delivered += 1
            self._task(f"kit {pi + 1}: DELIVERED on base {chr(65 + di)} "
                       f"({self.delivered}/{len(self.tour)})")
            self.leg += 1
            self.retries = 0
            self.after_takeoff = self._next_target
        self._enter(self.ARMING)

    def _grab_failed(self):
        """The gripper closed on nothing: retry the pickup, or skip the kit."""
        pi, _ = self.tour[self.leg]
        self.carrying = False
        if self.retries < self.pick_retries:
            self.retries += 1
            self._task(f"kit {pi + 1}: gripper EMPTY — opening, retry "
                       f"{self.retries}/{self.pick_retries}")
            self._new_pickup()
            # closed jaws must not come down on the kit again
            self._open_then(lambda: self._leg_to(("pickup", *self.pick_xy[pi])),
                            hold=True)
        else:
            self._task(f"kit {pi + 1}: gripper empty {self.retries + 1}x — "
                       "opening, skipping this kit")
            self.leg += 1
            self.retries = 0
            self._open_then(self._next_target, hold=True)

    def _start_verify(self):
        self._hits = 0
        self._enter(self.VERIFY)

    def _do_verify(self):
        """Is the kit still sitting on the pickup base? Then the grab failed.

        A kit ON the base at cruise is small (radius < verify_max_radius_px);
        one hanging in the gripper right under the lens is huge or occluded.
        """
        self._hold()
        for u, v, r in self.kits.fresh(self.fresh_s):
            if r <= self.verify_max_px and math.hypot(
                    u - 320.0, v - 240.0) <= self.verify_centre_px:
                self._hits += 1
                self.get_logger().info(
                    f"verify: kit-like detection r={r:.0f} px at "
                    f"({u:.0f}, {v:.0f})", throttle_duration_sec=1.0)
        if self._hits >= self.verify_hits:
            pi, _ = self.tour[self.leg]
            if self.retries < self.pick_retries:
                self.retries += 1
                self.carrying = False
                self._task(f"kit {pi + 1}: still on the base — grab FAILED, "
                           f"opening, retry {self.retries}/{self.pick_retries}")
                self.descend.reset()
                self._new_pickup()
                self._open_then(lambda: self._enter(self.DESCEND), hold=True)
            else:
                self._task(f"kit {pi + 1}: grab failed {self.retries + 1}x — "
                           "opening, skipping this kit")
                self.leg += 1
                self.retries = 0
                self.carrying = False
                self._open_then(self._next_target, hold=True)
            return
        if self._elapsed() >= self.verify_s:
            pi, _ = self.tour[self.leg]
            self.carrying = True
            self._task(f"kit {pi + 1}: PICKED UP and lifted")
            self._next_target()

    def _do_done(self):
        pass

    def _do_aborted(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Phase2MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
