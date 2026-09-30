"""
Phase 3 mission core: take off, 1 m forward, 90 deg right, find the operator,
then fly what their arms say until they say land.

Pure Python, no ROS: the wrapper is phase3_gesture_node, the tests are
test/test_gesture_mission.py. Same contract as the maze core: the node feeds
`step()` the LIO pose, the latest gesture message and the lidar points near the
drone, and turns the returned Command into MAVROS calls.

Frames: `odom` (the takeoff point, x forward, y left, z up, yaw CCW+). Every
velocity out of here is in odom; the node rotates it into MAVROS' local ENU.
Gestures are in the drone's BODY: DIREITA is the drone's right, APROXIMAR its
nose — toward the operator, because FIND leaves the camera looking at them.

States:
  WAIT      on the ground until start()
  TAKEOFF   FCU takeoff to takeoff_alt over the ground
  FORWARD   forward_dist along the takeoff heading, on the LIO position
  TURN      turn_deg on the spot (-90 = right)
  FIND      hold, look for a person; after find_wait_s sweep +-search_span_deg
            around where it faced; nobody for find_timeout_s -> LAND
  GESTURE   debounced gestures -> velocities, inside the safety limits
  LAND      FCU land, wait for disarm
  LANDED    SUBIR held retakeoff_hold_s -> TAKEOFF again (then FIND)

Safety, every tick in GESTURE:
  - NENHUM (no person, bent arm, dead zone) or a stale camera is HOVER
  - HOVER/STOP hold the position where the motion stopped (LIO), not a drifting zero
  - altitude stays inside [min_alt, max_alt] over the takeoff ground
  - horizontal motion stays inside max_radius of where the gestures began
  - no horizontal motion toward anything the lidar sees closer than stop_dist
    along the path (the operator included), nor with a stale lidar
  - person lost for lost_search_s -> back to FIND, which lands on its own timeout
"""

import math
from dataclasses import dataclass, field

import numpy as np

# (lateral, forward, vertical): +lateral = drone's right, +forward = drone's nose
COMMANDS = {
    "HOVER": (0, 0, 0), "STOP": (0, 0, 0), "POUSAR": (0, 0, 0),
    "DIREITA": (1, 0, 0), "ESQUERDA": (-1, 0, 0),
    "APROXIMAR": (0, 1, 0), "AFASTAR": (0, -1, 0),
    "SUBIR": (0, 0, 1), "DESCER": (0, 0, -1),
}
DESCRIPTION = {
    "HOVER": "parado (bracos para baixo)",
    "STOP": "parar (bracos em T)",
    "SUBIR": "subir (bracos em Y)",
    "DESCER": "descer (bracos em A)",
    "DIREITA": "ir para a direita",
    "ESQUERDA": "ir para a esquerda",
    "APROXIMAR": "aproximar do operador",
    "AFASTAR": "afastar do operador",
    "POUSAR": "pousar",
}
# How long a gesture must be held before it is obeyed. Going back to neutral
# is the quickest (safety). AFASTAR/APROXIMAR are slower than the default:
# forming POUSAR (one arm up, then the other out) passes through APROXIMAR
# (DIAG_UP+DOWN) and sits in AFASTAR (UP+DOWN) while the second arm rises.
HOLD_S = {"HOVER": 0.25, "STOP": 0.25, "POUSAR": 1.2, "AFASTAR": 0.7, "APROXIMAR": 0.7}
DEFAULT_HOLD = 0.5


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clip(v, lim):
    return max(-lim, min(lim, v))


class Debouncer:
    """Confirms a gesture only after it is held for HOLD_S.

    A NENHUM shorter than `grace` does not reset a gesture being confirmed: one
    frame with a low-confidence wrist would otherwise restart the clock, and a
    noisy skeleton could never confirm anything. Longer than that it is HOVER.
    """

    def __init__(self, hold=None, default=DEFAULT_HOLD, grace=0.2):
        self.hold = dict(HOLD_S if hold is None else hold)
        self.default, self.grace = default, grace
        self.reset()

    def reset(self, active="HOVER", t=0.0):
        self.active, self.since = active, t
        self.cand, self.t0, self.t_seen = None, 0.0, 0.0

    def update(self, g, t):
        """(active command, changed?)."""
        if g == "NENHUM":
            if self.cand not in (None, "HOVER") and t - self.t_seen <= self.grace:
                return self.active, False
            g = "HOVER"
        if g == self.active:
            self.cand = None
            return self.active, False
        if g != self.cand:
            self.cand, self.t0 = g, t
        self.t_seen = t
        if t - self.t0 >= self.hold.get(g, self.default):
            self.active, self.since, self.cand = g, t, None
            return self.active, True
        return self.active, False


def clearance(pts_xy, direction, corridor):
    """Distance to the nearest point in a corridor along `direction` (body xy).

    pts_xy: (N,2) points in the body frame at flight level. The corridor is
    2*corridor wide: the drone's width plus margin, not a cone, so a wall
    alongside does not stop a move parallel to it.
    """
    n = math.hypot(direction[0], direction[1])
    if pts_xy is None or len(pts_xy) == 0 or n < 1e-9:
        return math.inf
    d = np.array([direction[0] / n, direction[1] / n])
    s = pts_xy @ d
    lat = np.abs(pts_xy @ np.array([-d[1], d[0]]))
    m = (s > 0.0) & (lat < corridor)
    return float(s[m].min()) if m.any() else math.inf


@dataclass
class MissionParams:
    takeoff_alt: float = 1.0        # m over the takeoff ground
    forward_dist: float = 1.0
    turn_deg: float = -90.0         # + left (CCW), - right
    leg_speed: float = 0.3          # m/s cap on the scripted legs
    pos_kp: float = 0.8
    pos_tol: float = 0.12
    z_kp: float = 1.0
    yaw_kp: float = 1.2
    yaw_rate_max: float = 0.5       # rad/s
    yaw_tol_deg: float = 5.0
    settle_s: float = 1.0
    obs_timeout: float = 0.5        # gesture message older than this: camera dead
    find_confirm_s: float = 0.8
    find_wait_s: float = 3.0
    search_span_deg: float = 45.0
    search_rate: float = 0.25       # rad/s
    find_timeout_s: float = 60.0
    lost_search_s: float = 4.0
    v_xy: float = 0.3               # gesture speeds
    v_z: float = 0.25
    min_alt: float = 0.5
    max_alt: float = 2.0
    max_radius: float = 3.0
    stop_dist: float = 1.2          # lidar: nothing closer than this along the path
    corridor: float = 0.45
    cloud_timeout: float = 1.0
    require_lidar: bool = True
    track_yaw: bool = True          # keep the operator centred in the image
    track_kp: float = 0.6
    track_deadband: float = 0.08
    hold_kp: float = 0.8
    hold_vmax: float = 0.3
    retakeoff: bool = True
    retakeoff_hold_s: float = 1.5
    takeoff_tol: float = 0.15


@dataclass
class Observation:
    t: float                        # when it arrived (node clock)
    gesture: str
    person: bool
    image_x: float = 0.0


@dataclass
class Command:
    kind: str                       # 'idle' | 'takeoff' | 'velocity' | 'land'
    velocity: tuple = (0.0, 0.0, 0.0)
    yaw_rate: float = 0.0
    z: float = 0.0                  # takeoff target, odom z
    takeoff_seq: int = 0            # a new takeoff each time this changes
    state: str = ""
    gesture: str = ""
    blocked: str = ""               # why a gesture's motion was cut, if it was
    debug: dict = field(default_factory=dict)


class Mission:

    def __init__(self, params=None, logger=print):
        self.p = params or MissionParams()
        self.log = logger
        self.state = "WAIT"
        self.t_state = None
        self.debounce = Debouncer()
        self.z_ground = None
        self.z_target = None
        self.takeoff_seq = 0
        self.leg_origin = self.leg_goal = self.yaw0 = None
        self.yaw_goal = None
        self.t_in_tol = None
        self.hold_xy = None
        self.facing = None
        self.sweep_dir = -1.0
        self.t_seen = None
        self.t_lost = None
        self.anchor = None
        self.land_sent = False
        self.t_subir = None
        self.blocked = ""
        self._started = False

    # ── public ──────────────────────────────────────────────────────────────
    def start(self):
        self._started = True

    def step(self, t, pose, obs=None, obstacles=None, obstacles_t=None, armed=True):
        """One tick. pose = (xyz in odom, yaw). obs = latest Observation or None.
        obstacles = (N,2) body-frame xy at flight level, stamped obstacles_t."""
        pos, yaw = np.asarray(pose[0], dtype=float), float(pose[1])
        if self.t_state is None:
            self.t_state = t
        fresh = obs is not None and t - obs.t <= self.p.obs_timeout
        person = fresh and obs.person
        handler = getattr(self, f"_{self.state.lower()}")
        cmd = handler(t, pos, yaw, obs if fresh else None, person, obstacles, obstacles_t, armed)
        cmd.state, cmd.takeoff_seq = self.state, self.takeoff_seq
        cmd.gesture = self.debounce.active
        return cmd

    # ── helpers ─────────────────────────────────────────────────────────────
    def _go(self, state, t, why=""):
        self.log(f"{self.state} -> {state}{' (' + why + ')' if why else ''}")
        self.state, self.t_state = state, t

    def _goto(self, pos, goal_xy, vmax, kp):
        e = np.asarray(goal_xy, dtype=float) - pos[:2]
        v = kp * e
        n = float(np.linalg.norm(v))
        if n > vmax:
            v *= vmax / n
        vz = clip(self.p.z_kp * (self.z_target - pos[2]), self.p.v_z)
        return (float(v[0]), float(v[1]), float(vz))

    def _yaw_to(self, yaw, goal):
        return clip(self.p.yaw_kp * wrap(goal - yaw), self.p.yaw_rate_max)

    def _track(self, obs, person):
        if not (self.p.track_yaw and person) or abs(obs.image_x) < self.p.track_deadband:
            return 0.0
        # operator right of centre -> turn right, which is yaw negative
        return clip(-self.p.track_kp * obs.image_x, self.p.yaw_rate_max)

    def _settled(self, t, ok):
        if not ok:
            self.t_in_tol = None
            return False
        if self.t_in_tol is None:
            self.t_in_tol = t
        return t - self.t_in_tol >= self.p.settle_s

    def _hold_cmd(self, pos, yaw_rate=0.0):
        return Command("velocity", self._goto(pos, self.hold_xy, self.p.hold_vmax, self.p.hold_kp),
                       yaw_rate)

    # ── states ──────────────────────────────────────────────────────────────
    def _wait(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if self._started:
            self._go("TAKEOFF", t)
        return Command("idle")

    def _takeoff(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if self.z_ground is None:
            self.z_ground = float(pos[2])
        if self.z_target is None:
            self.z_target = self.z_ground + self.p.takeoff_alt
        if pos[2] >= self.z_target - self.p.takeoff_tol:
            self.hold_xy = pos[:2].copy()
            if self.leg_origin is None:
                self._go("FORWARD", t, f"at {pos[2] - self.z_ground:.2f} m")
            else:
                self._enter_find(t, pos, yaw, "airborne again")
            return self._hold_cmd(pos)
        return Command("takeoff", z=self.z_target)

    def _forward(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if self.leg_origin is None:
            self.leg_origin, self.yaw0 = pos[:2].copy(), yaw
            self.leg_goal = self.leg_origin + self.p.forward_dist * np.array(
                [math.cos(yaw), math.sin(yaw)])
            self.log(f"forward {self.p.forward_dist:.2f} m to "
                     f"({self.leg_goal[0]:.2f}, {self.leg_goal[1]:.2f})")
        v = self._goto(pos, self.leg_goal, self.p.leg_speed, self.p.pos_kp)
        ok = np.linalg.norm(self.leg_goal - pos[:2]) < self.p.pos_tol
        if self._settled(t, ok):
            self.t_in_tol = None
            self.hold_xy = self.leg_goal.copy()
            self.yaw_goal = wrap(self.yaw0 + math.radians(self.p.turn_deg))
            self._go("TURN", t, f"turning {self.p.turn_deg:+.0f} deg")
        return Command("velocity", v, self._yaw_to(yaw, self.yaw0))

    def _turn(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        err = wrap(self.yaw_goal - yaw)
        if self._settled(t, abs(err) < math.radians(self.p.yaw_tol_deg)):
            self.t_in_tol = None
            self._enter_find(t, pos, yaw, "facing the operator's side")
        return self._hold_cmd(pos, self._yaw_to(yaw, self.yaw_goal))

    def _enter_find(self, t, pos, yaw, why):
        self.facing = yaw
        self.hold_xy = pos[:2].copy()
        self.t_seen = None
        self.sweep_dir = -1.0
        self.debounce.reset("HOVER", t)
        self._go("FIND", t, why)

    def _find(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if person:
            if self.t_seen is None:
                self.t_seen = t
                self.log("person in view")
            if t - self.t_seen >= self.p.find_confirm_s:
                if self.anchor is None:
                    self.anchor = pos[:2].copy()
                self.t_lost = None
                self.debounce.reset("HOVER", t)
                self._go("GESTURE", t, "operator found, obeying gestures")
            return self._hold_cmd(pos, self._track(obs, True))
        self.t_seen = None
        if t - self.t_state >= self.p.find_timeout_s:
            self._go("LAND", t, f"nobody in {self.p.find_timeout_s:.0f} s")
            return self._hold_cmd(pos)
        rate = 0.0
        if t - self.t_state >= self.p.find_wait_s:
            off = wrap(yaw - self.facing)
            span = math.radians(self.p.search_span_deg)
            if off <= -span:
                self.sweep_dir = 1.0
            elif off >= span:
                self.sweep_dir = -1.0
            rate = self.sweep_dir * self.p.search_rate
        return self._hold_cmd(pos, rate)

    def _gesture(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        raw = obs.gesture if person else "NENHUM"
        active, changed = self.debounce.update(raw, t)
        if changed:
            self.log(f"GESTO: {active} -> {DESCRIPTION.get(active, active)}")
        if person:
            self.t_lost = None
        else:
            if self.t_lost is None:
                self.t_lost = t
            if t - self.t_lost >= self.p.lost_search_s:
                self._enter_find(t, pos, yaw, f"operator lost for {self.p.lost_search_s:.0f} s")
                return self._hold_cmd(pos)
        if active == "POUSAR":
            self._go("LAND", t, "POUSAR")
            return self._hold_cmd(pos)

        lat, fwd, vert = COMMANDS.get(active, (0, 0, 0))
        blocked = []
        # vertical, inside [min_alt, max_alt]
        z_rel = pos[2] - self.z_ground
        if (vert > 0 and z_rel >= self.p.max_alt) or (vert < 0 and z_rel <= self.p.min_alt):
            blocked.append("max_alt" if vert > 0 else "min_alt")
            vert = 0
        if vert:
            vz = vert * self.p.v_z
            self.z_target = float(pos[2])
        else:
            vz = clip(self.p.z_kp * (self.z_target - pos[2]), self.p.v_z)

        # horizontal, body -> odom, then the fence and the lidar
        vb = np.array([fwd * self.p.v_xy, -lat * self.p.v_xy])
        if vb.any():
            if self.p.require_lidar and (obst_t is None or t - obst_t > self.p.cloud_timeout):
                vb[:] = 0.0
                blocked.append("lidar_stale")
            else:
                c = clearance(obst, vb, self.p.corridor)
                if c < self.p.stop_dist:
                    vb[:] = 0.0
                    blocked.append(f"obstacle {c:.2f} m")
        cy, sy = math.cos(yaw), math.sin(yaw)
        v = np.array([cy * vb[0] - sy * vb[1], sy * vb[0] + cy * vb[1]])
        if v.any():
            d = pos[:2] - self.anchor
            r = float(np.linalg.norm(d))
            if r >= self.p.max_radius and v @ d > 0:
                u = d / r
                v = v - (v @ u) * u
                blocked.append("max_radius")
        if np.linalg.norm(v) > 1e-6:
            self.hold_xy = pos[:2].copy()
            vxy = v
        else:
            vxy = np.array(self._goto(pos, self.hold_xy, self.p.hold_vmax, self.p.hold_kp)[:2])

        why = ",".join(blocked)
        if why and why != self.blocked:
            self.log(f"{active} held back: {why}")
        self.blocked = why
        return Command("velocity", (float(vxy[0]), float(vxy[1]), float(vz)),
                       self._track(obs, person), blocked=why)

    def _land(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if not self.land_sent:
            self.land_sent = True
            return Command("land")
        if not armed:
            self.debounce.reset("HOVER", t)
            self.t_subir = None
            self._go("LANDED", t, "disarmed")
            return Command("idle")
        return Command("land")

    def _landed(self, t, pos, yaw, obs, person, obst, obst_t, armed):
        if not self.p.retakeoff:
            return Command("idle")
        if person and obs.gesture == "SUBIR":
            if self.t_subir is None:
                self.t_subir = t
            if t - self.t_subir >= self.p.retakeoff_hold_s:
                self.log("GESTO: SUBIR mantido -> decolar")
                self.takeoff_seq += 1
                self.land_sent = False
                self.z_ground = float(pos[2])
                self.z_target = self.z_ground + self.p.takeoff_alt
                self._go("TAKEOFF", t, "SUBIR held on the ground")
                return Command("takeoff", z=self.z_target)
        else:
            self.t_subir = None
        return Command("idle")
