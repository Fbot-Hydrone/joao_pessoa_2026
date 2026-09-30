"""The known-map Phase 4 stack in one object, wired like the ROS graph.

    odometry (LIO) ──────────────┐
    body scan + attitude ─> MapLocalization ─> PoseMux ─> KnownMission ─> Command
                                  (beams, map pose)   ^         │
                                                      └─source──┘

In ROS these are three nodes talking over topics (beam_localizer_node,
pose_mux_node, phase4_maze_node). Here they're called in the same order per
sensor frame, which is what the kinematic sim and the viewer demo run.
"""
import numpy as np

from hydrone_confined.core.beams import level
from hydrone_confined.core.map_localization import MapLocalization, MapLocParams
from hydrone_confined.core.pose_mux import MuxParams, PoseMux, wrap
from hydrone_mission.phase4.explore_mission import Mission, MissionParams
from hydrone_mission.phase4.known_mission import KnownMission, KnownParams


def rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


class KnownStack:

    def __init__(self, known_map, mission=None, loc=None, mux=None, explore=True, logger=None):
        self.km = known_map
        self.logger = logger
        self.locp = loc or MapLocParams()
        self.loc = MapLocalization(known_map, self.locp)
        self.mux = PoseMux(mux or MuxParams(), logger=self._log_mux)
        mp = mission or KnownParams()
        ex = Mission(MissionParams(flight_z=mp.out_z), logger=None) if explore else None
        self.mission = KnownMission(known_map, mp, explore=ex, logger=logger)
        to = known_map.takeoff
        self.z0 = float(to.get('top', 0.54)) + 0.08      # base_link on the pad, map z
        self.prior = np.array([to['center'][0], to['center'][1], self.z0, to.get('yaw', 0.0)])
        self.yaw_est = self.prior[3]
        self._odom_prev = None
        self._cmd_prev = np.zeros(3)
        self.fix = None
        self.cmd = None

    def _log_mux(self, msg):
        self.mission.say(f'[mux] {msg}')

    # what the sim reads back
    @property
    def phase(self):
        return self.mission.phase

    @property
    def log(self):
        return self.mission.log

    @property
    def fallback_used(self):
        return self.mission.fallback_used

    @property
    def step_ms(self):
        return [a + b for a, b in zip(self.mission.step_ms, self.loc.ms)]

    def __call__(self, f):
        t = f.t
        odom = np.array([f.odom[0][0], f.odom[0][1], f.odom[0][2], f.odom[1]])
        if self._odom_prev is None:
            self.loc.start(self.prior[:2], self.prior[3], self.prior[2])
            self.mux.set_anchor(self.prior, odom)
            self.mux.track = True
        # LIO into the mux, velocity turned into the map frame by the anchor
        v_nav = np.r_[rot2(self.mux.T[3]) @ np.asarray(f.odom_vel)[:2], f.odom_vel[2]]
        self.mux.on_odom(t, odom, v_nav)
        # odometry step in the body frame, for the particle filter's prediction
        step = None
        if self._odom_prev is not None:
            d = odom - self._odom_prev
            step = np.r_[rot2(-self._odom_prev[3]) @ d[:2], d[2], wrap(d[3])]
        self._odom_prev = odom
        cmd_body = None
        if self.cmd is not None:
            cmd_body = np.r_[rot2(-self.yaw_est) @ self._cmd_prev[:2], self._cmd_prev[2]]
        if f.scan_body is not None:
            self.fix = self.loc.step(t, f.scan_body, f.attitude, step, cmd_body)
            self.mux.on_map(t, self.fix.pose, self.fix.healthy)
        pose = self.mux.pose()
        # the scan in the map frame, through the pose that flies (for the explorer)
        scan_nav = None
        if f.scan_body is not None:
            lvl = level(f.scan_body, *f.attitude)
            R = rot2(pose[3])
            s = np.array([self.locp.mount[0], self.locp.mount[1]])
            origin = np.r_[pose[:2] + R @ s, pose[2] + self.locp.mount[2]]
            pts = np.column_stack([(lvl[:, :2] @ R.T) + origin[:2], lvl[:, 2] + origin[2]])
            scan_nav = (pts, origin)
        self.cmd = self.mission.step(t, pose, self.fix, f.qr, scan_nav, self.mux.mode)
        self.mux.request(self.cmd.source, t)
        self.yaw_est = pose[3]
        self._cmd_prev = np.asarray(self.cmd.velocity, dtype=float) if self.cmd.kind == 'velocity' \
            else np.zeros(3)
        return self.cmd
