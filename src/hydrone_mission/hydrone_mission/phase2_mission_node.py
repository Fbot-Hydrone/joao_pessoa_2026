#!/usr/bin/env python3
"""
phase2_mission_node — Fase 2: carry three first-aid kits, then fly home.

THE STRATEGY ONLY. Every "how" is a block (docs/ARQUITETURA.md):

  fly            hydrone_controller.vehicle.Vehicle   (arm, takeoff, land, setpoint)
  grab/release   hydrone_controller.gripper.Gripper   (sim: ardubridge; real: AX-12A)
  legs           /hydrone/nav/navigate_to             (nav_node: A* over the octomap)
  descend        hydrone_nav.visual_descent           (centre by camera, then LAND)
  where/order    hydrone_nav.arena + route.plan_deliveries

The cycle, per kit, in the order route.plan_deliveries picks:

  GOTO pickup -> DESCEND on the KIT -> LAND (kit between the legs) -> GRAB
  -> ARMING/TAKEOFF -> VERIFY (is the kit still on the base?) -> GOTO delivery
  -> DESCEND on the BASE -> LAND -> RELEASE -> ARMING/TAKEOFF -> next kit

then GOTO home -> LAND -> DONE.

NO TURNS. The heading the vehicle armed with is held for the whole flight:
yaw changes are where the visual odometry loses itself. The day the kit
detector learns orientation, aligning to a rotated kit is an option.

Rules this encodes (docs/REGRAS-CBR-2026.pdf, Fase 2): every landing gear part
inside the base on pickup and delivery; one kit at a time; landing off a base
ends the attempt; 10 min per attempt on the VEHICLE's clock (/clock in sim);
an autonomous return after >= 1 delivery doubles the score.
"""

import math
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Range
from std_msgs.msg import String

from hydrone_msgs.action import NavigateTo
from hydrone_msgs.msg import PadDetection

from hydrone_controller.gripper import Gripper
from hydrone_controller.vehicle import Job, Vehicle
from hydrone_nav import route
from hydrone_nav.arena import arena_to_map, load_layout
from hydrone_nav.precision_landing import body_to_world
from hydrone_nav.visual_descent import LOST, READY, VisualDescent


def yaw_of(pose):
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
    WAIT, ARMING, TAKEOFF, GOTO, DESCEND, TOUCH, LAND, ACT, VERIFY, DONE, \
        ABORTED = ("WAIT", "ARMING", "TAKEOFF", "GOTO", "DESCEND", "TOUCH",
                   "LAND", "ACT", "VERIFY", "DONE", "ABORTED")

    def __init__(self, **kwargs):
        super().__init__("phase2_mission", **kwargs)
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        default_layout = os.path.join(
            get_package_share_directory("hydrone_bringup"),
            "config", "phase2_bases.yaml")
        self.layout = load_layout(p("layout_file", "") or default_layout)
        self.arena_yaw = p("arena_yaw_deg", -90.0)
        self.cruise = p("cruise_alt", 2.5)
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
        # How a grab is confirmed:
        #   "gripper" — the Gripper result's `holding` (the AX-12A measures it
        #               by load; the sim reports the command). Default.
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
        self.touch_timeout = p("touch_timeout_s", 20.0)
        self.descend = VisualDescent(
            descend_to_m=p("visual_descend_to_m", 1.2),
            fine_px=p("visual_fine_px", 40.0),
            gain=p("visual_gain", 0.6),
            axes=tuple(p("visual_axes", [-1.0, -1.0])))

        self.vehicle = Vehicle(self)
        self.gripper = Gripper(self)
        self.nav = ActionClient(self, NavigateTo, "/hydrone/nav/navigate_to")
        self.bases = Frames(self, p("base_topic", "/hydrone/pads/down/detections"),
                            p("base_confidence", 0.30))
        self.kits = Frames(self, p("kit_topic", "/hydrone/kits/detections"),
                           p("kit_confidence", 0.50))

        self.pose, self.range_m, self.fx, self.sim_clock = None, None, None, None
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda m: setattr(self, "pose", m),
                                 qos_profile_sensor_data)
        self.create_subscription(Range, p("range_topic",
                                          "/mavros/distance_sensor/rangefinder"),
                                 self._cb_range, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/down_cam/camera_info",
                                 lambda m: setattr(self, "fx", float(m.k[0])),
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self._cb_clock,
                                 qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, "/hydrone/mission/status", 10)

        self.state, self._since, self._job = self.WAIT, self.now(), None
        self.home = self.home_yaw = None
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
        self._touch_xy = None
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"phase2_mission ready — {len(self.layout.pickup)} kit(s), "
            f"{len(self.layout.delivery)} delivery base(s), cruise "
            f"{self.cruise} m, heading held (no turns).")

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

    def _enter(self, state):
        if state != self.state:
            self.get_logger().info(f"[{self.state} -> {state}]")
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
        self.get_logger().info(f"TASK: {text} {self._clock_note()}")

    def _map(self, p_arena):
        return arena_to_map(p_arena, self.home, self.layout.takeoff,
                            self.arena_yaw)

    def _hold(self):
        p = self.pose.pose.position
        self.vehicle.setpoint(p.x, p.y, p.z, self.home_yaw)

    # ── tick ────────────────────────────────────────────────────────────────
    def _tick(self):
        if self.state in (self.DONE, self.ABORTED):
            return
        self._check_budget()
        getattr(self, f"_do_{self.state.lower()}")()
        self.pub_status.publish(String(data=(
            f"state={self.state} kit={self.leg + 1}/{len(self.tour) or '?'} "
            f"carrying={self.carrying} delivered={self.delivered}")))

    def _check_budget(self):
        if self.t0 is None or self.target and self.target[0] == "home":
            return
        if self.state in (self.LAND, self.ACT):
            return
        if self.mission_now() - self.t0 < self.budget - self.reserve:
            return
        self.get_logger().warn(f"BUDGET: going home {self._clock_note()}")
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
        p = self.pose.pose.position
        self.home, self.home_yaw = (p.x, p.y), yaw_of(self.pose)
        pick = [self._map(q[:2]) for q in self.layout.pickup]
        deliv = [self._map(q[:2]) for q in self.layout.delivery]
        self.pick_xy, self.deliv_xy = pick, deliv
        self.tour = route.plan_deliveries(self.home, pick, deliv)
        self.get_logger().info(
            "home = ({:.2f}, {:.2f}); tour (pickup->delivery): {}".format(
                *self.home, ", ".join(f"{a + 1}->{chr(65 + b)}"
                                      for a, b in self.tour)))
        self.after_takeoff = self._next_target
        self._job = self.gripper.open()      # start with an open gripper
        self._enter(self.ARMING)

    def _do_arming(self):
        if self._job is None:
            self._job = self.vehicle.arm()
        elif self._job.done:
            if self._job.ok:
                self._enter(self.TAKEOFF)
            else:
                self.get_logger().warn(f"arm: {self._job.message} — retrying")
                self._job = None

    def _do_takeoff(self):
        if self._job is None:
            self._job = self.vehicle.takeoff(self.climb, hold_z=self.cruise)
            return
        if not self._job.done:
            return
        if self._job.ok:
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
        if self.takeoff_tries > 3:
            self.get_logger().error(f"takeoff refused 3x: {self._job.message}")
            self._enter(self.ABORTED)
        else:
            self._enter(self.ARMING)

    def _next_target(self):
        """Decide where the next leg goes, from what is being carried."""
        if self.leg >= len(self.tour):
            self._go_home()
            return
        pi, di = self.tour[self.leg]
        if self.carrying:
            self.target = ("delivery", *self.deliv_xy[di])
        else:
            self.target = ("pickup", *self.pick_xy[pi])
        self._enter(self.GOTO)

    def _go_home(self):
        self.target = ("home", *self.home)
        self._enter(self.GOTO)

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
        blind, self._blind = self._blind, False
        self._enter(self.LAND if kind == "home" or blind else self.DESCEND)

    def _do_descend(self):
        """Centre on the KIT (pickup) or the BASE (delivery) while descending."""
        kind, tx, ty = self.target
        frames = self.kits if kind == "pickup" else self.bases
        dets = [(u, v) for u, v, _ in frames.fresh(self.fresh_s)]
        h = self.range_m if self.range_m is not None else self.cruise
        step = self.descend.update(dets, h, self.fx, self.now())
        if step.phase == READY:
            self.get_logger().info(
                f"{kind}: centred at {h:.2f} m ({step.err_px:.0f} px) — "
                + ("touching down on it" if kind == "pickup"
                   and self.pickup_touch else "landing"))
            if kind == "pickup" and self.pickup_touch and step.picked:
                self._touch_xy = self._kit_world_xy(step.picked, h)
                self._enter(self.TOUCH)
            else:
                self._enter(self.LAND)
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
            self._enter(self.GOTO)
            return
        if frames.stamp != self._used_stamp and step.picked is not None:
            self._used_stamp = frames.stamp
            wx, wy = body_to_world((step.dx, step.dy), yaw_of(self.pose))
            p = self.pose.pose.position
            self.vehicle.setpoint(p.x + wx, p.y + wy, p.z + step.dz,
                                  self.home_yaw)

    def _kit_world_xy(self, uv, h):
        """Where the kit IS, from its last pixel and the height over it.

        The same fixed belly mapping visual_descent uses, at gain 1: the full
        offset, not a fraction of it.
        """
        u0, v0 = self.descend.target_uv
        m = max(h, 0.2) / (self.fx or 320.0)
        ax_u, ax_v = self.descend.axes
        bx, by = ax_v * (uv[1] - v0) * m, ax_u * (uv[0] - u0) * m
        wx, wy = body_to_world((bx, by), yaw_of(self.pose))
        p = self.pose.pose.position
        return p.x + wx, p.y + wy

    def _do_touch(self):
        """Descend on the kit's XY under position control; grab near contact."""
        kx, ky = self._touch_xy
        p = self.pose.pose.position
        touched = (self.range_m is not None
                   and self.range_m <= self.touch_range)
        if self._job is None and (touched
                                  or self._elapsed() > self.touch_timeout):
            self.vehicle.setpoint(kx, ky, p.z, self.home_yaw)   # hold here
            self._job = self.gripper.close()
            self.get_logger().info(
                f"pickup: at {self.range_m if self.range_m is not None else -1:.2f}"
                f" m over the kit — closing the gripper, then landing")
            return
        if self._job is None:
            # The controller limits the speed; asking 0.3 m below keeps it
            # coming down steadily without overshooting the contact.
            self.vehicle.setpoint(kx, ky, p.z - 0.3, self.home_yaw)
            return
        if self._job.done:
            self._enter(self.LAND)

    def _do_land(self):
        if self._job is None:
            self._job = self.vehicle.land(disarm=True)
            return
        if not self._job.done:
            return
        r = self._job.result
        z = r.z if r is not None else float("nan")
        kind = self.target[0]
        if kind == "home":
            self._task(f"home — {self.delivered} kit(s) delivered; landed at "
                       f"z={z:.2f}")
            self._enter(self.DONE)
            return
        self._enter(self.ACT)

    def _do_act(self):
        kind = self.target[0]
        pi, di = self.tour[self.leg]
        if self._job is None:
            self._job = (self.gripper.close() if kind == "pickup"
                         else self.gripper.open())
            return
        if not self._job.done:
            return
        if kind == "pickup":
            held = self._job.ok and self._job.result.holding
            self._task(f"kit {pi + 1}: gripper closed on pickup {pi + 1} "
                       f"({self._job.message}); verifying from the air")
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
            self._task(f"kit {pi + 1}: gripper EMPTY — retry "
                       f"{self.retries}/{self.pick_retries}")
            self.target = ("pickup", *self.pick_xy[pi])
            self._enter(self.GOTO)
        else:
            self._task(f"kit {pi + 1}: gripper empty {self.retries + 1}x — "
                       "skipping this kit")
            self.leg += 1
            self.retries = 0
            self._next_target()

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
                           f"retry {self.retries}/{self.pick_retries}")
                self.descend.reset()
                self._enter(self.DESCEND)
            else:
                self._task(f"kit {pi + 1}: grab failed {self.retries + 1}x — "
                           "skipping this kit")
                self.leg += 1
                self.retries = 0
                self.carrying = False
                self._next_target()
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
