#!/usr/bin/env python3
"""
hydrone_controller — the ONLY node that talks to the flight controller.

Every phase drives the vehicle through these actions (wrapped for Python by
hydrone_controller.vehicle.Vehicle), so no mission ever touches MAVROS:

  /hydrone/controller/arm      hydrone_msgs/action/Arm       GUIDED + arm
  /hydrone/controller/takeoff  hydrone_msgs/action/Takeoff   climb, then hold
  /hydrone/controller/go_to    hydrone_msgs/action/GoTo      straight setpoint
  /hydrone/controller/land     hydrone_msgs/action/Land      LAND + stop props

Streaming setpoint (no arrival wait — servoing, nudges, holds):
  /hydrone/controller/cmd_pose  geometry_msgs/PoseStamped

Legacy API of the June skeleton, kept for mission_node / nav_node:
  services /hydrone/controller/{arm,disarm,takeoff,land} (std_srvs; an
           action and a service may share a name — different endpoints)
  topics   /hydrone/controller/drone_pose, /hydrone/controller/status

The flight logic (retry on time not on acks, touchdown by stillness, disarm
until the props are confirmed stopped) was MEASURED inside phase1_mission_node
over hundreds of sim landings; it is ported here unchanged, with the reasons
kept next to the code that needs them.
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from hydrone_msgs.action import Arm, GoTo, Land, Takeoff

from hydrone_controller.fcu import FcuLink, yaw_of
from hydrone_controller.touchdown import TouchdownDetector

POLL_S = 0.05


class Cancelled(Exception):
    pass


class ControllerNode(Node):

    def __init__(self, **kwargs):
        super().__init__("hydrone_controller", **kwargs)
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731

        self.retry_period = p("retry_period_s", 2.0)
        self.arm_timeout = p("arm_timeout_s", 45.0)
        self.takeoff_timeout = p("takeoff_timeout_s", 45.0)
        self.takeoff_margin = p("takeoff_margin_m", 0.15)
        self.goto_tol = p("goto_tolerance_m", 0.25)
        self.land_timeout = p("land_timeout_s", 60.0)
        # Ceiling on the wait for the FCU to confirm the disarm. Above
        # DISARM_DELAY (10 s) on purpose: if our own disarm is refused because
        # ArduPilot does not yet agree the vehicle is down, the auto-disarm
        # still lands inside this window and is caught by the same test.
        self.disarm_timeout = p("disarm_timeout_s", 12.0)
        # How often to re-ask for the disarm, s. Deliberately much shorter than
        # `retry_period_s`, which is sized for mode and takeoff commands.
        #
        # The 7 s above is NOT ArduPilot being slow — it is us asking rarely.
        # Our touchdown test (land_settle_s of stillness) fires before
        # ArduPilot's own land detector has latched, so the first disarms are
        # refused and the wait is then quantised to the retry period. At 2.0 s
        # that turns a sub-second disagreement into whole seconds of standing
        # still with the props running. Asking ~5x more often costs nothing —
        # a refused disarm is a few bytes — and collects the accept the moment
        # ArduPilot is willing to give it.
        self.disarm_retry = p("disarm_retry_s", 0.4)
        self.legacy_takeoff_alt = p("takeoff_height", 1.2)
        self.touchdown = TouchdownDetector(
            settle_s=p("land_settle_s", 2.0),
            still_tol_m=p("land_still_tol_m", 0.05),
            min_descent_m=p("min_descent_m", 0.30))

        self.fcu = FcuLink(self, setpoint_hz=p("setpoint_hz", 10.0),
                           service_timeout_s=p("service_timeout_s", 30.0))
        cbg = self.fcu.cbg

        # One motion job at a time. A new goal pre-empts the running one.
        self._lock = threading.Lock()
        self._active = None           # goal handle (or "legacy") in charge
        self._landing = False         # LAND owns the vehicle: no setpoints

        for name, typ, fn in (("arm", Arm, self._exec_arm),
                              ("takeoff", Takeoff, self._exec_takeoff),
                              ("go_to", GoTo, self._exec_goto),
                              ("land", Land, self._exec_land)):
            ActionServer(self, typ, f"/hydrone/controller/{name}", fn,
                         goal_callback=lambda g: GoalResponse.ACCEPT,
                         cancel_callback=lambda g: CancelResponse.ACCEPT,
                         callback_group=cbg)

        self.create_subscription(PoseStamped, "/hydrone/controller/cmd_pose",
                                 self._cb_cmd_pose, 10, callback_group=cbg)

        # ── legacy (June skeleton) ───────────────────────────────────────────
        self.pub_drone_pose = self.create_publisher(
            PoseStamped, "/hydrone/controller/drone_pose",
            qos_profile_sensor_data)
        self.pub_status = self.create_publisher(
            String, "/hydrone/controller/status", 10)
        self.create_service(Trigger, "/hydrone/controller/arm",
                            self._svc_arm, callback_group=cbg)
        self.create_service(Trigger, "/hydrone/controller/disarm",
                            self._svc_disarm, callback_group=cbg)
        self.create_service(SetBool, "/hydrone/controller/takeoff",
                            self._svc_takeoff, callback_group=cbg)
        self.create_service(Trigger, "/hydrone/controller/land",
                            self._svc_land, callback_group=cbg)
        self.create_timer(1.0, self._publish_status, callback_group=cbg)

        self.get_logger().info("hydrone_controller ready — actions under "
                               "/hydrone/controller/{arm,takeoff,go_to,land}")

    # ────────────────────────────────────────────────────────────────────────
    # Plumbing
    # ────────────────────────────────────────────────────────────────────────

    def _now(self):
        return self.fcu.now()

    def _claim(self, owner):
        with self._lock:
            self._active = owner

    def _check(self, owner):
        """Raise if the job was cancelled or pre-empted by a newer one."""
        if self._active is not owner:
            raise Cancelled("pre-empted by a newer command")
        if hasattr(owner, "is_cancel_requested") and owner.is_cancel_requested:
            raise Cancelled("cancel requested")

    def _wait_call(self, call, owner):
        """Block (polling) until a MAVROS call resolves. Returns its status."""
        if call is None:
            return "unavailable"
        while True:
            st = call.poll()
            if st != call.PENDING:
                if st != call.OK:
                    self.get_logger().warn(
                        f"{call.name} ({call.tag}) -> {st}"
                        + (f": {self.fcu.fcu_reason()}"
                           if call.tag in ("arm", "takeoff") else ""))
                return st
            self._check(owner)
            time.sleep(POLL_S)

    def _cb_cmd_pose(self, msg):
        """Streaming setpoint: no arrival wait. Ignored while LAND owns it."""
        if self._landing:
            return
        self.fcu.go(msg.pose.position.x, msg.pose.position.y,
                    msg.pose.position.z, yaw_of(msg))

    def _publish_status(self):
        if self.fcu.pose is not None:
            self.pub_drone_pose.publish(self.fcu.pose)
        self.pub_status.publish(String(data=(
            f"mode={self.fcu.mode} armed={self.fcu.armed} "
            f"streaming={self.fcu.streaming}")))

    # ────────────────────────────────────────────────────────────────────────
    # Jobs — blocking, cancellable; shared by actions and legacy services
    # ────────────────────────────────────────────────────────────────────────

    def run_arm(self, owner, timeout_s=0.0, feedback=None):
        """GUIDED first, then arm, retried on TIME rather than on acks.

        MAVROS acks a mode change that ArduPilot then declines (EKF not ready,
        pre-arm pending), so "the call succeeded" is not "the vehicle is in
        GUIDED" — only /mavros/state settles it. GUIDED is checked BEFORE armed:
        after a landing the mode is still LAND, and a takeoff sent in LAND is
        refused forever.
        """
        deadline = self._now() + (timeout_s or self.arm_timeout)
        last = -1e9
        while True:
            self._check(owner)
            if self.fcu.mode == "GUIDED" and self.fcu.armed:
                # No setpoint yet: a position target streamed on the ground
                # would override the NAV_TAKEOFF that comes next.
                return True, "armed in GUIDED"
            if self._now() > deadline:
                return False, f"not armed: {self.fcu.fcu_reason()}"
            if feedback:
                feedback(self.fcu.mode, self.fcu.armed)
            if self._now() - last >= self.retry_period:
                last = self._now()
                call = (self.fcu.set_mode("GUIDED") if self.fcu.mode != "GUIDED"
                        else self.fcu.arm(True))
                self._wait_call(call, owner)
            time.sleep(POLL_S)

    def run_takeoff(self, owner, altitude, hold_z=float("nan"), feedback=None):
        """Climb `altitude` above whatever we stand on, then hold.

        CLIMBED, not absolute altitude: pose.z is measured from the FIRST
        takeoff plane, so on any base at a different height the two disagree
        by that difference (MEASURED 2026-08-23: after landing 0.76 m below the
        start plane a perfect climb never reached the absolute target, and the
        mission re-sent takeoff forever).

        Re-sent ONLY when the FCU refused and the vehicle is not climbing.
        Re-sending into a climb that was accepted gets "failed: no reason" from
        ArduPilot every time, which buried real refusals in noise.
        """
        if not self.fcu.armed or self.fcu.mode != "GUIDED":
            return False, "not armed in GUIDED — send Arm first", 0.0
        start_z = self.fcu.xyz()[2]
        deadline = self._now() + self.takeoff_timeout
        accepted, last = False, -1e9
        while True:
            self._check(owner)
            x, y, z = self.fcu.xyz()
            climbed = z - start_z
            if feedback:
                feedback(climbed)
            if climbed >= altitude - self.takeoff_margin:
                self.fcu.go(x, y, z if math.isnan(hold_z) else hold_z,
                            self.fcu.yaw())
                return True, f"climbed {climbed:.2f} m", z
            if self._now() > deadline:
                return (False, f"takeoff did not lift us from z={start_z:.2f}: "
                        f"{self.fcu.fcu_reason()}", z)
            if (not accepted and climbed < 0.2
                    and self._now() - last >= self.retry_period):
                last = self._now()
                accepted = self._wait_call(
                    self.fcu.takeoff(altitude), owner) == "ok"
            time.sleep(POLL_S)

    def run_goto(self, owner, x, y, z, yaw, tol=0.0, timeout_s=0.0,
                 feedback=None):
        tol = tol or self.goto_tol
        self.fcu.go(x, y, z, yaw)
        deadline = self._now() + timeout_s if timeout_s > 0 else None
        while True:
            self._check(owner)
            px, py, pz = self.fcu.xyz()
            d = math.dist((px, py, pz), (x, y, z))
            if feedback:
                feedback(d)
            if d <= tol:
                return True, f"arrived ({d:.2f} m)"
            if deadline and self._now() > deadline:
                return False, f"timeout {d:.2f} m short"
            time.sleep(POLL_S)

    def run_land(self, owner, disarm=True, feedback=None):
        """Hand the descent to ArduPilot's LAND; then stop the props.

        Touchdown = the FCU disarmed us, or the altitude STOPPED after a real
        descent (see touchdown.py). Then a NORMAL disarm, never the 21196 force:
        if ArduPilot refuses, it does not agree we are down, and the right
        answer is to keep asking, not to cut motors in mid-air.

        WHY THE DISARM, measured: across 262 landings nothing ever sent one;
        the mission relied on DISARM_DELAY (10 s) while re-arming after 4 s.
        The rules count a landing only "com hélices desligadas" — the run
        scored 0. Returns (touched, props_stopped, z, message).
        """
        self._landing = True
        try:
            return self._run_land(owner, disarm, feedback)
        finally:
            self._landing = False

    def _run_land(self, owner, disarm, feedback):
        self.fcu.stop_stream()
        self.touchdown.reset(self.fcu.xyz()[2] if self.fcu.pose else None)
        self._wait_call(self.fcu.set_mode("LAND"), owner)
        start, last = self._now(), self._now()
        touched = False
        while not touched:
            self._check(owner)
            if feedback:
                feedback("descending")
            # Keep asking until /mavros/state agrees: an acked mode command
            # that ArduPilot then declined would leave us hovering here.
            if (self.fcu.mode != "LAND"
                    and self._now() - last >= self.retry_period):
                last = self._now()
                self.get_logger().warn("not in LAND yet; re-sending the mode.")
                self._wait_call(self.fcu.set_mode("LAND"), owner)
            touched = (not self.fcu.armed) or self.touchdown.update(
                self._now(), self.fcu.xyz()[2])
            if not touched and self._now() - start > self.land_timeout:
                self.get_logger().warn(
                    f"no touchdown within {self.land_timeout:.0f} s — "
                    "carrying on to the disarm anyway.")
                break
            time.sleep(POLL_S)
        z = self.fcu.xyz()[2]
        why = "disarmed" if not self.fcu.armed else "descended and stopped"
        self.get_logger().info(f"touchdown at z={z:.2f} m ({why})")
        if not disarm:
            return touched, not self.fcu.armed, z, why

        start, last = self._now(), -1e9
        while self.fcu.armed:
            self._check(owner)
            if feedback:
                feedback("disarming")
            if self._now() - start > self.disarm_timeout:
                return (touched, False, self.fcu.xyz()[2],
                        f"STILL ARMED {self.disarm_timeout:.0f} s after "
                        f"touchdown — {self.fcu.fcu_reason()}")
            if self._now() - last >= self.disarm_retry:
                last = self._now()
                self._wait_call(self.fcu.arm(False), owner)
            time.sleep(POLL_S)
        return touched, True, self.fcu.xyz()[2], "props stopped"

    # ────────────────────────────────────────────────────────────────────────
    # Action servers
    # ────────────────────────────────────────────────────────────────────────

    def _run_action(self, gh, job, make_result):
        self._claim(gh)
        try:
            out = job()
        except Cancelled as e:
            gh.canceled() if gh.is_cancel_requested else gh.abort()
            res = make_result(None)
            res.message = str(e)
            return res
        res = make_result(out)
        (gh.succeed if res.success else gh.abort)()
        return res

    def _exec_arm(self, gh):
        def fb(mode, armed):
            m = Arm.Feedback(mode=mode, armed=armed)
            gh.publish_feedback(m)

        def result(out):
            r = Arm.Result()
            if out:
                r.success, r.message = out
            return r
        return self._run_action(
            gh, lambda: self.run_arm(gh, gh.request.timeout_s, fb), result)

    def _exec_takeoff(self, gh):
        def fb(climbed):
            gh.publish_feedback(Takeoff.Feedback(climbed=float(climbed)))

        def result(out):
            r = Takeoff.Result()
            if out:
                r.success, r.message, r.z = out[0], out[1], float(out[2])
            return r
        g = gh.request
        return self._run_action(
            gh, lambda: self.run_takeoff(gh, g.altitude, g.hold_z, fb), result)

    def _exec_goto(self, gh):
        def fb(d):
            gh.publish_feedback(GoTo.Feedback(distance=float(d)))

        def result(out):
            r = GoTo.Result()
            if out:
                r.success, r.message = out
            return r
        g = gh.request
        return self._run_action(
            gh, lambda: self.run_goto(gh, g.position.x, g.position.y,
                                      g.position.z, g.yaw, g.tolerance,
                                      g.timeout_s, fb), result)

    def _exec_land(self, gh):
        def fb(phase):
            gh.publish_feedback(Land.Feedback(phase=phase))

        def result(out):
            r = Land.Result()
            if out:
                touched, stopped, z, msg = out
                r.success, r.props_stopped = bool(touched), bool(stopped)
                r.z, r.message = float(z), msg
            return r
        return self._run_action(
            gh, lambda: self.run_land(gh, gh.request.disarm, fb), result)

    # ────────────────────────────────────────────────────────────────────────
    # Legacy services (blocking wrappers over the same jobs)
    # ────────────────────────────────────────────────────────────────────────

    def _legacy(self, response, job):
        owner = object()
        self._claim(owner)
        try:
            out = job(owner)
            response.success, response.message = bool(out[0]), str(out[1])
        except Cancelled as e:
            response.success, response.message = False, str(e)
        return response

    def _svc_arm(self, req, resp):
        return self._legacy(resp, lambda o: self.run_arm(o))

    def _svc_disarm(self, req, resp):
        def job(o):
            st = self._wait_call(self.fcu.arm(False), o)
            return st == "ok", f"disarm -> {st}"
        return self._legacy(resp, job)

    def _svc_takeoff(self, req, resp):
        def job(o):
            if req.data:
                ok, msg = self.run_arm(o)
                if not ok:
                    return ok, msg
            return self.run_takeoff(o, self.legacy_takeoff_alt)[:2]
        return self._legacy(resp, job)

    def _svc_land(self, req, resp):
        def job(o):
            touched, stopped, z, msg = self.run_land(o, disarm=True)
            return touched, msg
        return self._legacy(resp, job)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
