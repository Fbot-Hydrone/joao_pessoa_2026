#!/usr/bin/env python3
"""
phase3_hri_node — Phase 3: land on the bases guided only by the operator.

    WAIT_FCU -> ARMING -> TAKEOFF -> APPROACH (approach_forward_m ahead)
             -> FACE (turn approach_turn_deg to the right) -> WAIT_OPERATOR
             -> COMMAND  <-- gestures move / stop / land / return home
                  LAND -> LANDED --(TAKEOFF gesture)--> ARMING -> TAKEOFF -> COMMAND
                  RETURN_HOME -> (take off if on a base) -> HOMING -> LAND -> DONE

Everything up to WAIT_OPERATOR is autonomous, as the rules ask: take off by the
team computer, approach the operator standing in the middle of the arena.
From COMMAND on the drone moves ONLY on a gesture; nothing here tracks the
operator (forbidden, Obs. 5) or looks for bases. After `target_bases`
landings it takes off and flies home by itself; a RETURN_HOME gesture does
the same earlier (the score doubles on any autonomous return after >= 1 base).

Gestures arrive on /hydrone/vision/human_gesture (hydrone_msgs/HumanGesture);
vocabulary and frames in hydrone_mission/phase3/core.py. In BiguaSim there is
no operator to film, so `gesture_terminal` publishes the same messages from
the keyboard. Every gesture, and the operator detection, is printed with a
[FASE 3] tag for the judge.

in   /hydrone/vision/human_gesture, /mavros/state, /mavros/local_position/pose
out  /mavros/setpoint_position/local (streamed while airborne in GUIDED)
     /mavros/set_mode, /mavros/cmd/arming, /mavros/cmd/takeoff
     /hydrone/phase3/status (std_msgs/String, 1 Hz)
srv  /hydrone/phase3/start  (std_srvs/Trigger)  takeoff + approach, when auto_start is off
     /hydrone/phase3/abort  (std_srvs/Trigger)  LAND where it is, mission over

Everything is in MAVROS local ENU, so it does not matter what the EKF flies on
(LIO on the Kopis, VO on the Holybro).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import Trigger

from hydrone_msgs.msg import HumanGesture

from hydrone_mission.phase3 import core


def yaw_of(pose):
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class _Call:
    """One in-flight MAVROS service call with a deadline (never block the tick)."""

    def __init__(self, node, client, request, timeout_s):
        self.name = client.srv_name
        self.future = client.call_async(request)
        self.deadline = node.now() + timeout_s

    def poll(self, now):
        if self.future.done():
            r = self.future.result()
            if r is None:
                return "failed"
            ok = getattr(r, "success", None)
            if ok is None:
                ok = getattr(r, "mode_sent", False)
            return "ok" if ok else "failed"
        if now > self.deadline:
            self.future.cancel()
            return "timeout"
        return "pending"


class Phase3HriNode(Node):

    WAIT_START = "WAIT_START"
    WAIT_FCU = "WAIT_FCU"
    ARMING = "ARMING"
    TAKEOFF = "TAKEOFF"
    APPROACH = "APPROACH"
    FACE = "FACE"
    WAIT_OPERATOR = "WAIT_OPERATOR"
    COMMAND = "COMMAND"
    LAND = "LAND"
    LANDED = "LANDED"
    HOMING = "HOMING"
    DONE = "DONE"
    ABORTED = "ABORTED"

    AIRBORNE = (APPROACH, FACE, WAIT_OPERATOR, COMMAND, HOMING)

    def __init__(self):
        super().__init__("phase3_hri_node")
        params = self.declare_parameters("", [
            ("auto_start", True),          # false: wait for /hydrone/phase3/start
            ("takeoff_alt", 1.5),          # m above what it takes off from
            ("approach_forward_m", 2.0),
            ("approach_turn_deg", 90.0),   # to the RIGHT (clockwise)
            ("step_m", 0.5),               # one horizontal gesture
            ("step_z_m", 0.3),             # one up/down gesture
            ("yaw_step_deg", 45.0),        # one turn gesture
            ("min_alt", 0.6),              # m above the takeoff base, clamps MOVE_DOWN
            ("max_alt", 2.5),
            ("min_confidence", 0.5),
            ("repeat_cooldown_s", 1.0),    # same gesture again, after arriving
            ("target_bases", 6),           # then home, automatically
            ("arrive_tol_m", 0.2),
            ("home_tol_m", 0.08),          # LAND comes straight down: be over the base
            ("yaw_tol_deg", 8.0),
            ("move_timeout_s", 30.0),
            ("takeoff_timeout_s", 60.0),
            ("land_timeout_s", 90.0),
            ("land_settle_s", 2.0),
            ("land_still_tol_m", 0.05),
            ("min_descent_m", 0.3),
            ("dwell_s", 2.0),              # on the last base, before flying home
            ("service_timeout_s", 30.0),   # sim runs below real time: see timeouts.yaml
            ("retry_period_s", 2.0),
            ("setpoint_hz", 10.0),
            ("gesture_topic", "/hydrone/vision/human_gesture"),
            # the camera's hold-to-move gestures (frl_core, via gesture_camera)
            ("vel_speed", 0.5),            # m/s while APROXIMAR/AFASTAR/DIREITA/ESQUERDA is held
            ("vel_speed_z", 0.3),          # m/s while SUBIR/DESCER is held
            ("vel_lead_m", 0.5),           # how far the setpoint may run ahead of the drone
            ("vel_timeout_s", 0.5),        # no frame for this long => brake
            ("takeoff_hold_s", 1.5),       # SUBIR held this long on the ground => TAKEOFF
        ])
        self.p = {q.name: q.value for q in params}
        # step sizes can be changed mid-flight (gesture_terminal `passo`/`giro`)
        self.add_on_set_parameters_callback(self._on_params)

        self.state = self.WAIT_FCU if self.p["auto_start"] else self.WAIT_START
        self._since = self.now()
        self.mav = State()
        self.pose = None
        self.home = None              # [x, y, z, yaw] where it first took off
        self.sp = None                # [x, y, z, yaw] being streamed
        self.stream = False
        self.moving = None            # the action being flown, or None when holding
        self.move_since = 0.0
        self.last_gesture = (None, -1e9)   # (action, time it arrived)
        self.landings = 0
        self.going_home = False       # the landing in progress is the final one
        self._call = None
        self._last_cmd = 0.0
        self._takeoff_z = 0.0
        self._takeoff_acked = False
        self._after_takeoff = self.APPROACH
        self._auto_home_at = None     # time to fly home by itself, after the last base
        self._land_z = None
        self._z_hist = []
        # the gesture stream from the camera (frl_core vocabulary)
        self.vel = None               # (lateral, frente, vertical) being flown, or None
        self._stream_name = None      # last streamed gesture
        self._stream_since = 0.0      # when it started
        self._stream_t = -1e9         # last frame
        self._stream_block = None     # ignore this gesture until it changes (after a takeoff)

        self.create_subscription(State, "/mavros/state", self._cb_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, qos_profile_sensor_data)
        self.create_subscription(HumanGesture, self.p["gesture_topic"],
                                 self._cb_gesture, 10)
        self.pub_sp = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.pub_status = self.create_publisher(String, "/hydrone/phase3/status", 10)
        self.cli_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.cli_arm = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.cli_takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.create_service(Trigger, "/hydrone/phase3/start", self._srv_start)
        self.create_service(Trigger, "/hydrone/phase3/abort", self._srv_abort)

        self.create_timer(1.0 / max(self.p["setpoint_hz"], 1.0), self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"phase3_hri_node up — {self.state}. Gestures on {self.p['gesture_topic']}.")

    # ── inputs ────────────────────────────────────────────────────────────────

    def _on_params(self, params):
        for q in params:
            self.p[q.name] = q.value
            self.get_logger().info(f"parameter {q.name} = {q.value}")
        return SetParametersResult(successful=True)

    def _cb_state(self, m):
        self.mav = m

    def _cb_pose(self, m):
        self.pose = m

    def _srv_start(self, _req, res):
        if self.state != self.WAIT_START:
            res.success, res.message = False, f"already running ({self.state})"
            return res
        self.get_logger().info("start requested by the team computer.")
        self._enter(self.WAIT_FCU)
        res.success, res.message = True, "starting: takeoff + approach"
        return res

    def _srv_abort(self, _req, res):
        self.get_logger().warn("ABORT — landing where it is, mission over.")
        self.stream = False
        self._set_mode("LAND")
        self._enter(self.ABORTED)
        res.success, res.message = True, "LAND sent"
        return res

    def _cb_gesture(self, msg):
        if msg.confidence < self.p["min_confidence"]:
            return
        action = core.action_for(msg.gesture_name)
        now = self.now()

        if self.state == self.WAIT_OPERATOR:
            # any sighting of the operator is the detection the rules ask for
            hp = msg.human_position
            n = len(msg.skeleton_keypoints) // 3
            self.get_logger().info(
                "\n" + "=" * 60 +
                f"\n[FASE 3] OPERADOR DETECTADO  conf {msg.confidence:.2f}"
                f"\n         posição ({hp.x:.2f}, {hp.y:.2f}, {hp.z:.2f})"
                f"  esqueleto: {n} pontos"
                "\n         aguardando comandos por gesto" +
                "\n" + "=" * 60)
            self._enter(self.COMMAND)
            return

        if msg.gesture_name in core.STREAMED:
            self._on_stream(msg, now)
            return
        if action is None or action == core.HUMAN:
            return
        if self.state not in (self.COMMAND, self.LANDED):
            self.get_logger().info(
                f"[FASE 3] gesto '{msg.gesture_name}' ignorado em {self.state}",
                throttle_duration_sec=2.0)
            return

        # a pose held in front of the camera arrives every frame: act once per
        # arrival, and on the same gesture only after the cooldown
        if action in core.MOVES and self.moving is not None:
            return
        last_action, last_t = self.last_gesture
        if action == last_action and now - last_t < self.p["repeat_cooldown_s"]:
            return
        self.last_gesture = (action, now)

        self.get_logger().info(
            f"[FASE 3] COMANDO: {msg.gesture_name} -> {action}"
            f"   (humano em {msg.human_position.x:.2f}, {msg.human_position.y:.2f})")
        self._execute(action)

    def _on_stream(self, msg, now):
        """One frame of the camera's debounced gesture (frl_core vocabulary).

        Arrives every frame, so only a CHANGE is logged and acted on, except
        for the velocity, which is refreshed by every frame and dies with the
        stream (vel_timeout_s).
        """
        name = msg.gesture_name
        # a gap in the frames (operator out of view) restarts every hold
        changed = name != self._stream_name or now - self._stream_t > self.p["vel_timeout_s"]
        self._stream_t = now
        if changed:
            self._stream_name, self._stream_since = name, now
            if self._stream_block is not None and name != self._stream_block:
                self._stream_block = None
            hp = msg.human_position
            self.get_logger().info(
                f"[FASE 3] GESTO: {name} -> {core.velocity_for(name) or core.action_for(name)}"
                f"   (operador na imagem em {hp.x:.0f}, {hp.y:.0f} px)")
        if name == self._stream_block:
            return

        if self.state == self.LANDED:
            if name == core.FRL_TAKEOFF and now - self._stream_since >= self.p["takeoff_hold_s"]:
                self.get_logger().info(f"[FASE 3] {name} mantido {self.p['takeoff_hold_s']:.1f} s no chão -> DECOLAR")
                self._stream_block = name      # lower the arms before it climbs on SUBIR
                self._execute(core.TAKEOFF)
            return
        if self.state != self.COMMAND:
            if changed:
                self.get_logger().info(f"[FASE 3] gesto '{name}' ignorado em {self.state}")
            return

        vel = core.velocity_for(name)
        if vel is not None:
            if self.vel is None:
                self.moving = None             # a held gesture overrides a terminal step
            self.vel = vel
        elif self.vel is not None or (changed and name == "STOP"):
            self.vel = None
            self._hold_here()                  # HOVER / STOP: brake where it is
        if changed and core.action_for(name) == core.LAND:
            self._begin_landing(final=False)

    # ── gesture -> flight ─────────────────────────────────────────────────────

    def _execute(self, action):
        if self.state == self.LANDED:
            if action == core.TAKEOFF:
                self._after_takeoff = self.COMMAND
                self._enter(self.ARMING)
            elif action == core.RETURN_HOME:
                self._go_home()
            else:
                self.get_logger().info(f"[FASE 3] {action}: pousado — decole primeiro (TAKEOFF)")
            return

        # COMMAND, airborne
        if action == core.STOP:
            self._hold_here()
        elif action == core.LAND:
            self._begin_landing(final=False)
        elif action == core.RETURN_HOME:
            self._go_home()
        elif action == core.TAKEOFF:
            self.get_logger().info("[FASE 3] TAKEOFF: já está voando")
        elif action in core.MOVES:
            sp = core.step_setpoint(self.sp, action, self.p["step_m"], self.p["step_z_m"],
                                    math.radians(self.p["yaw_step_deg"]))
            zmin = self.home[2] + self.p["min_alt"]
            zmax = self.home[2] + self.p["max_alt"]
            if not zmin <= sp[2] <= zmax:
                sp[2] = min(max(sp[2], zmin), zmax)
                self.get_logger().warn(
                    f"[FASE 3] altura limitada a {sp[2] - self.home[2]:.2f} m "
                    f"(min_alt {self.p['min_alt']}, max_alt {self.p['max_alt']})")
            self._goto(sp, action)

    def _go_home(self):
        self.vel = None
        self.going_home = True
        self.get_logger().info(
            f"[FASE 3] RETORNO AUTÔNOMO à base de decolagem "
            f"({self.home[0]:.2f}, {self.home[1]:.2f}) — {self.landings} base(s) visitada(s)")
        if self.state == self.LANDED:
            self._after_takeoff = self.HOMING
            self._enter(self.ARMING)
        else:
            self._start_homing()

    def _start_homing(self):
        # home xy at the height already being flown, heading kept
        self._goto([self.home[0], self.home[1], self.sp[2], self.sp[3]], "HOME")
        self._enter(self.HOMING)

    # ── state machine ─────────────────────────────────────────────────────────

    def _tick(self):
        if self.stream and self.sp is not None:
            self._publish_sp()
        handler = {
            self.WAIT_FCU: self._do_wait_fcu,
            self.ARMING: self._do_arming,
            self.TAKEOFF: self._do_takeoff,
            self.APPROACH: self._do_approach,
            self.FACE: self._do_face,
            self.COMMAND: self._do_command,
            self.HOMING: self._do_homing,
            self.LAND: self._do_land,
            self.LANDED: self._do_landed,
        }.get(self.state)
        if handler is not None:
            handler()

    def _do_wait_fcu(self):
        if not (self.mav.connected and self.pose is not None):
            self._info_throttled("waiting for the MAVROS link and a local position...")
            return
        if not (self.cli_arm.service_is_ready() and self.cli_mode.service_is_ready()
                and self.cli_takeoff.service_is_ready()):
            self._info_throttled("waiting for the MAVROS command services...")
            return
        pp = self.pose.pose.position
        self.home = [pp.x, pp.y, pp.z, yaw_of(self.pose)]
        self.get_logger().info(
            f"takeoff base = ({pp.x:.2f}, {pp.y:.2f}, {pp.z:.2f}), "
            f"heading {math.degrees(self.home[3]):.0f} deg")
        self._after_takeoff = self.APPROACH
        self._enter(self.ARMING)

    def _do_arming(self):
        """GUIDED, then arm. /mavros/state decides, not the service replies:
        MAVROS acks a mode ArduPilot then refuses (EKF not ready yet)."""
        if self.mav.mode == "GUIDED" and self.mav.armed:
            self._enter(self.TAKEOFF)
            return
        if self._poll() == "pending" or self.now() - self._last_cmd < self.p["retry_period_s"]:
            return
        if self.mav.mode != "GUIDED":
            self._set_mode("GUIDED")
        else:
            self._start(self.cli_arm, CommandBool.Request(value=True))
        self._info_throttled(f"arming... (mode {self.mav.mode}, armed {self.mav.armed})")

    def _do_takeoff(self):
        # climbed, not absolute z: the second takeoff leaves from a base whose
        # top is not the plane the first one left from
        climbed = self.pose.pose.position.z - self._takeoff_z
        if climbed >= self.p["takeoff_alt"] - 0.15:
            pp = self.pose.pose.position
            # the height asked for, not the one the handover caught it at
            z = min(self._takeoff_z + self.p["takeoff_alt"], self.home[2] + self.p["max_alt"])
            self.sp = [pp.x, pp.y, z, yaw_of(self.pose)]
            self.stream = True
            self.moving = None
            self.get_logger().info(f"airborne — {climbed:.2f} m up, holding z {z:.2f}")
            if self._after_takeoff == self.APPROACH:
                yaw = self.sp[3]
                d = self.p["approach_forward_m"]
                self._goto([pp.x + d * math.cos(yaw), pp.y + d * math.sin(yaw), z, yaw],
                           "APPROACH")
                self.get_logger().info(f"[FASE 3] aproximação: {d:.1f} m à frente")
                self._enter(self.APPROACH)
            elif self._after_takeoff == self.HOMING:
                self._start_homing()
            else:
                self._enter(self.COMMAND)
            return
        if self.now() - self._since > self.p["takeoff_timeout_s"]:
            self.get_logger().warn("takeoff did not lift it; re-arming")
            self._enter(self.ARMING)
            return
        if self.mav.mode != "GUIDED" or not self.mav.armed:
            self._enter(self.ARMING)       # disarmed on the ground before the command landed
            return
        status = self._poll()
        if status == "ok":
            self._takeoff_acked = True
        # once accepted, give the climb until takeoff_timeout_s; resend only a
        # refused or lost command
        if (status == "pending" or self._takeoff_acked
                or self.now() - self._last_cmd < self.p["retry_period_s"]):
            return
        self._start(self.cli_takeoff, CommandTOL.Request(altitude=float(self.p["takeoff_alt"])))

    def _do_approach(self):
        if self._arrived() or self._move_timed_out():
            self.moving = None
            turn = math.radians(self.p["approach_turn_deg"])
            self._goto([*self.sp[:3], core.wrap_pi(self.sp[3] - turn)], "FACE")
            self.get_logger().info(
                f"[FASE 3] aproximação: girando {self.p['approach_turn_deg']:.0f} deg à direita")
            self._enter(self.FACE)

    def _do_face(self):
        if self._arrived() or self._move_timed_out():
            self.moving = None
            self.get_logger().info(
                "\n[FASE 3] aproximação concluída — procurando o operador "
                "(gesture_terminal: 'o' simula a câmera o vendo)")
            self._enter(self.WAIT_OPERATOR)

    def _do_command(self):
        if self.vel is not None:
            self._fly_velocity()
            return
        if self.moving is None:
            return
        if self._arrived():
            self.get_logger().info(f"[FASE 3] {self.moving} concluído")
            self.moving = None
            self.last_gesture = (self.last_gesture[0], self.now())
        elif self._move_timed_out():
            self.get_logger().warn(f"[FASE 3] {self.moving} não chegou — segurando onde está")
            self._hold_here()

    def _fly_velocity(self):
        """Hold-to-move: run the setpoint ahead of the drone along the held
        gesture, never more than vel_lead_m ahead, and brake if the camera
        stops sending."""
        if self.now() - self._stream_t > self.p["vel_timeout_s"]:
            self.get_logger().warn("[FASE 3] câmera sem quadros — freando")
            self.vel = None
            self._hold_here()
            return
        dt = 1.0 / max(self.p["setpoint_hz"], 1.0)
        vx, vy, vz = core.velocity_enu(self.vel, self.sp[3], self.p["vel_speed"], self.p["vel_speed_z"])
        x, y, z = self.sp[0] + vx * dt, self.sp[1] + vy * dt, self.sp[2] + vz * dt
        pp = self.pose.pose.position
        d = math.hypot(x - pp.x, y - pp.y)
        if d > self.p["vel_lead_m"]:
            k = self.p["vel_lead_m"] / d
            x, y = pp.x + (x - pp.x) * k, pp.y + (y - pp.y) * k
        z = min(max(z, self.home[2] + self.p["min_alt"]), self.home[2] + self.p["max_alt"])
        self.sp = [x, y, z, self.sp[3]]
        self.stream = True

    def _do_homing(self):
        if self._arrived(self.p["home_tol_m"]) or self._move_timed_out():
            self.moving = None
            self.get_logger().info("[FASE 3] sobre a base de decolagem — pousando")
            self._begin_landing(final=True)

    def _begin_landing(self, final):
        self.vel = None
        self.going_home = self.going_home or final
        self.stream = False            # LAND owns the vehicle from here
        self.moving = None
        self._land_z = self.pose.pose.position.z
        self._z_hist = []
        self._enter(self.LAND)
        self._set_mode("LAND")

    def _do_land(self):
        if (self.mav.mode != "LAND" and self._poll() != "pending"
                and self.now() - self._last_cmd >= self.p["retry_period_s"]):
            self._set_mode("LAND")
            return
        z = self.pose.pose.position.z
        descended = self._land_z - z >= self.p["min_descent_m"]
        landed = (not self.mav.armed) or (descended and self._z_still())
        if not landed and self.now() - self._since < self.p["land_timeout_s"]:
            return
        if self.going_home:
            self.get_logger().info(
                "\n" + "=" * 60 +
                f"\n[FASE 3] POUSO NA BASE DE DECOLAGEM — fim da tentativa"
                f"\n         bases visitadas: {self.landings}" +
                "\n" + "=" * 60)
            self._enter(self.DONE)
            return
        self.landings += 1
        self.get_logger().info(
            f"[FASE 3] POUSO #{self.landings} (z {z:.2f}) — "
            f"{self.landings}/{self.p['target_bases']} bases")
        self._enter(self.LANDED)
        if self.landings >= self.p["target_bases"]:
            self.get_logger().info("[FASE 3] todas as bases pousadas — voltando para casa")
            self._auto_home_at = self.now() + self.p["dwell_s"]

    def _do_landed(self):
        if self._auto_home_at is not None and self.now() >= self._auto_home_at:
            self._auto_home_at = None
            self._go_home()

    def _z_still(self):
        now = self.now()
        self._z_hist.append((now, self.pose.pose.position.z))
        while len(self._z_hist) > 1 and self._z_hist[1][0] <= now - self.p["land_settle_s"]:
            self._z_hist.pop(0)
        if now - self._z_hist[0][0] < self.p["land_settle_s"]:
            return False
        zs = [z for _, z in self._z_hist]
        return max(zs) - min(zs) <= self.p["land_still_tol_m"]

    # ── helpers ───────────────────────────────────────────────────────────────

    def _goto(self, sp, what):
        self.sp = list(sp)
        self.stream = True
        self.moving = what
        self.move_since = self.now()

    def _hold_here(self):
        pp = self.pose.pose.position
        self.sp = [pp.x, pp.y, self.sp[2] if self.sp else pp.z, yaw_of(self.pose)]
        self.stream = True
        self.moving = None

    def _arrived(self, tol=None):
        pp = self.pose.pose.position
        d = math.dist((pp.x, pp.y, pp.z), self.sp[:3])
        dyaw = abs(core.wrap_pi(yaw_of(self.pose) - self.sp[3]))
        return d <= (tol or self.p["arrive_tol_m"]) and dyaw <= math.radians(self.p["yaw_tol_deg"])

    def _move_timed_out(self):
        if self.now() - self.move_since > self.p["move_timeout_s"]:
            self.get_logger().warn(f"{self.moving}: timed out, carrying on")
            return True
        return False

    def _publish_sp(self):
        x, y, z, yaw = self.sp
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_sp.publish(m)

    def _publish_status(self):
        pose = ""
        if self.pose is not None:
            pp = self.pose.pose.position
            pose = f" pos ({pp.x:.2f}, {pp.y:.2f}, {pp.z:.2f}) yaw {math.degrees(yaw_of(self.pose)):.0f}"
        self.pub_status.publish(String(
            data=f"{self.state} bases {self.landings}/{self.p['target_bases']}"
                 f"{' moving ' + self.moving if self.moving else ''}"
                 f"{' gesto ' + self._stream_name if self.vel else ''}{pose}"))

    def _set_mode(self, mode):
        self._start(self.cli_mode, SetMode.Request(custom_mode=mode))

    def _start(self, client, req):
        if not client.service_is_ready():
            self._info_throttled(f"{client.srv_name} not up yet")
            return
        self._last_cmd = self.now()
        self._call = _Call(self, client, req, self.p["service_timeout_s"])

    def _poll(self):
        if self._call is None:
            return "idle"
        status = self._call.poll(self.now())
        if status != "pending":
            if status != "ok":
                self.get_logger().warn(f"{self._call.name} -> {status}")
            self._call = None
        return status

    def _enter(self, state):
        if state != self.state:
            self.get_logger().info(f"[{self.state} -> {state}]")
        if state == self.TAKEOFF and self.pose is not None:
            self._takeoff_z = self.pose.pose.position.z
            self._takeoff_acked = False
        self.state = state
        self._since = self.now()
        self._call = None
        self._last_cmd = 0.0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _info_throttled(self, text):
        self.get_logger().info(text, throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = Phase3HriNode()
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
