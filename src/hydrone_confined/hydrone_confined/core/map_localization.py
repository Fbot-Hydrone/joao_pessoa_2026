"""One scan in, a map pose out: the beam localizer with everything around it.

Per scan:
  level the body-frame scan with the FCU roll/pitch
  beams (localization) + sectors (guard) + wall direction + vertical clearances
  predict with an odometry step if it is believable, else the commanded
  velocity, else nothing (just more noise)
  z from the ceiling when the map has one overhead, else carried by odometry
  update, and say whether the result can be flown on

Shared by the ROS node (beam_localizer_node) and the kinematic sim, so both
run exactly this.
"""
from dataclasses import dataclass, field
import time

import numpy as np

from hydrone_confined.core.beam_localizer import BeamLocalizer, LocalizerParams
from hydrone_confined.core.beams import BeamParams, beams, level, sector_min, vertical_clearance, wall_yaw


@dataclass
class MapLocParams:
    beams: BeamParams = field(default_factory=BeamParams)
    sectors: int = 16
    mount: tuple = (0.0, 0.0, 0.1)      # sensor over base_link
    max_speed: float = 0.8              # m/s; a faster odometry step isn't believed (the
                                        # missions fly <= 0.5; a diverging LIO runs away at ~20)
    healthy_xy: float = 0.12            # m, particle spread still flyable
    lost_s: float = 2.0                 # s lost before re-spreading the particles locally
    global_after: int = 2               # local re-spreads that didn't help: spread over every room
    init_xy: float = 0.25
    init_yaw: float = np.deg2rad(20.0)
    yaw_band: tuple = (-0.3, 0.3)
    z_gate: float = 0.15                # m a ceiling reading may move z by
    loc: LocalizerParams = field(default_factory=LocalizerParams)


@dataclass
class MapFix:
    t: float
    pose: np.ndarray                    # (x, y, z, yaw) base_link in the map
    cov: np.ndarray                     # (3,3) x, y, yaw
    healthy: bool
    consistency: float                  # fraction of beams that agree with the map
    beams: object = None                # levelled body-frame beams (for the viewer)
    sectors: object = None              # guard sectors, body frame
    z_source: str = 'odom'
    note: str = ''


class MapLocalization:

    def __init__(self, known_map, params=None):
        self.map = known_map
        self.p = params or MapLocParams()
        self.loc = BeamLocalizer(known_map, self.p.loc)
        self.z = None
        self.t = None
        self._lost_since = None
        self._respreads = 0
        self.last = None
        self.ms = []

    def start(self, xy, yaw, z, sigma_xy=None, sigma_yaw=None):
        self.loc.init_gaussian(xy, yaw, sigma_xy or self.p.init_xy, sigma_yaw or self.p.init_yaw)
        self.z = float(z)

    def step(self, t, scan_body, attitude=(0.0, 0.0), odom_step=None, cmd_vel=None):
        """odom_step: (dx, dy, dz, dyaw) in the body frame since the last scan.
        cmd_vel: (vx, vy, vz) commanded, body frame, used when odometry isn't believable."""
        c0 = time.perf_counter()
        dt = 0.1 if self.t is None else max(t - self.t, 1e-3)
        self.t = t
        pts = level(scan_body, *attitude)
        bm = beams(pts, self.p.beams)
        sec = sector_min(pts, self.p.sectors, band=self.p.beams.band, min_range=self.p.beams.min_range)
        ym = wall_yaw(pts, band=self.p.yaw_band)
        up, down = vertical_clearance(pts)
        note = ''
        # predict
        dz = 0.0
        if odom_step is not None and np.hypot(odom_step[0], odom_step[1]) <= self.p.max_speed * dt:
            self.loc.predict(odom_step[:2], odom_step[3])
            dz = odom_step[2]
        elif cmd_vel is not None:
            self.loc.predict(np.asarray(cmd_vel[:2]) * dt, 0.0, scale=2.0)
            dz = cmd_vel[2] * dt
            note = 'odometry step not believed, predicted from the command'
        else:
            self.loc.predict(np.zeros(2), 0.0, scale=3.0)
        est = self.loc.estimate()
        xy = est[0]
        z_pred = (self.z or 0.0) + dz
        zc = self.loc.z_from(xy, up=up, down=None, sensor_up=self.p.mount[2])
        # the ceiling has to agree roughly with where we were, or it's something else
        if zc is not None and abs(zc - z_pred) < self.p.z_gate:
            self.z, zsrc = zc, 'ceiling'
        else:
            self.z, zsrc = z_pred, 'odom'
        s = self.z + self.p.mount[2]
        band = (s + self.p.beams.band[0], s + self.p.beams.band[1])
        self.loc.update(bm, band, yaw_meas=ym, mount=self.p.mount[:2])
        xy, yaw, cov = self.loc.estimate()
        spread = float(np.sqrt(cov[0, 0] + cov[1, 1]))
        lost = self.loc.lost
        if lost:
            self._lost_since = self._lost_since if self._lost_since is not None else t
            if t - self._lost_since > self.p.lost_s:
                self._respreads += 1
                if self._respreads > self.p.global_after and self.map.rooms:
                    # local tries failed: we may be rooms away, look everywhere
                    self.loc.init_rooms(yaw=yaw, sigma_yaw=np.deg2rad(25.0))
                    self._respreads = 0
                    note = 'lost: particles spread over every room'
                else:
                    # spread out around where we think we are and try again
                    self.loc.init_gaussian(xy, yaw, 0.5, np.deg2rad(30.0))
                    note = 'lost: particles re-spread'
                self._lost_since = t
        else:
            self._lost_since = None
            self._respreads = 0
        healthy = (not lost) and spread <= self.p.healthy_xy
        self.last = MapFix(t, np.array([xy[0], xy[1], self.z, yaw]), cov, healthy,
                           self.loc.last_consistency, bm, sec, zsrc, note)
        self.ms.append((time.perf_counter() - c0) * 1e3)
        return self.last
