"""surface_offset — remove the jumps a downward rangefinder takes over bases.

ROS-free. The EKF uses the rangefinder as its height source, so every base the
sensor passes over (0 to 1.5 m tall in the CBR arena) looks like the vehicle
dropping by the base's height. The EKF smooths that step over ~1.2 s and the
position controller climbs to undo it (log 46, 2026-11-17, 651 s: raw range
1.62 -> 1.12 in one 50 ms sample, the vehicle then rose to 2.3 m off the floor).

This module sits between the real sensor and the EKF. It keeps `s`, the height
of the surface under the sensor above the arena FLOOR, and hands the EKF a
reading as if the sensor were always looking at the floor:

    v      = r * cos(tilt)              vertical distance sensor -> surface
    s      : vertical, metres, >= 0 over a base
    r_out  = r + s / cos(tilt)          what the EKF receives (it re-projects:
                                        r_out * cos(tilt) = v + s)

`s` is ALWAYS stored vertical, measured from readings projected with the
attitude of the instant they were taken, and converted back with the attitude
of the instant it is sent. A jump detected while leaning is never carried as
a slant length.

Detection. Between two samples the vertical reading may change by what the
vehicle itself moved (vz * dt, from the EKF/IMU) plus noise. Anything left over
is the surface changing:

    innovation = (v_k - v_{k-1}) - vz * dt

The edge of a base takes ~50 ms to cross (one intermediate sample at 20 Hz:
1.615, 1.549, 1.121 in log 46), so innovations are accumulated while they keep
the same sign and closed when one falls back to noise. A closed step of
|size| >= jump_min_m changes s by -size at once (the EKF never sees it).

Keeping it. A step that raised s (sensor now over something taller) is held
while there is EVIDENCE — the caller says the sensor is inside a zone that may
hold a raised structure, or the belly camera sees a base under it. With no
evidence for `confirm_timeout_s` it is released, which makes the vehicle
climb by s: the safe direction. A step that brings s back to ~0 is the floor
again and needs nothing. A step that would put s below the floor (< 0) is a
glitch by definition — the arena has no holes — and is held only until the
reading recovers or the timeout passes.
"""

import math
from dataclasses import dataclass, field


@dataclass
class Params:
    jump_min_m: float = 0.10       # smallest step taken as a surface change
    noise_m: float = 0.03          # per-sample innovation that is just noise
    max_step_samples: int = 4      # an edge must be crossed within this many
    max_tilt_deg: float = 25.0     # above this the reading is not used
    min_range_m: float = 0.10
    max_range_m: float = 8.0
    confirm_timeout_s: float = 1.5
    floor_snap_m: float = 0.05     # |s| below this is the floor
    max_dt_s: float = 0.30         # gap after which no step is inferred


@dataclass
class Step:
    t: float
    size: float                    # change of v (negative = surface rose)
    s_before: float
    s_after: float
    tilt_deg: float
    kept: bool = True
    reason: str = ""


@dataclass
class SurfaceOffset:
    p: Params = field(default_factory=Params)
    s: float = 0.0                 # surface height above the floor, vertical
    steps: list = field(default_factory=list)

    _v_prev: float = None
    _t_prev: float = None
    _acc: float = 0.0              # open step: accumulated innovation
    _acc_n: int = 0
    _acc_t0: float = 0.0
    _acc_tilt: float = 0.0
    _pending: Step = None          # last kept step still awaiting evidence
    _pending_since: float = 0.0
    _evidence_seen: bool = False

    def reset(self, s: float = 0.0):
        """Start (or restart) with a known surface: the takeoff base's height
        when the vehicle stands on it."""
        self.s = float(s)
        self._v_prev = self._t_prev = None
        self._acc, self._acc_n = 0.0, 0
        self._pending = None

    @staticmethod
    def tilt_cos(roll, pitch):
        return math.cos(roll) * math.cos(pitch)

    def update(self, t, r, roll, pitch, vz, evidence=False, active=True):
        """One sample. Returns (r_out, h_floor) or (None, None) if unusable.

        t        seconds
        r        raw range along the beam, m
        roll, pitch  radians, at the sample's time
        vz       vertical velocity, m/s, UP positive
        evidence the caller believes a raised surface may be under the sensor
        active   False while disarmed / on the ground being handled: track the
                 reading but infer no steps
        """
        c = self.tilt_cos(roll, pitch)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        if (not (self.p.min_range_m <= r <= self.p.max_range_m)
                or tilt > self.p.max_tilt_deg or c <= 0.0):
            return None, None
        v = r * c

        if evidence:
            self._evidence_seen = True

        if (active and self._v_prev is not None
                and 0.0 < t - self._t_prev <= self.p.max_dt_s):
            innov = (v - self._v_prev) - vz * (t - self._t_prev)
            self._feed(t, innov, tilt)
        else:
            self._close(t, force_drop=True)

        self._v_prev, self._t_prev = v, t
        self._expire(t, evidence)
        return r + self.s / c, v + self.s

    # ── steps ────────────────────────────────────────────────────────────

    def _feed(self, t, innov, tilt):
        if abs(innov) > self.p.noise_m and (
                self._acc_n == 0 or math.copysign(1, innov)
                == math.copysign(1, self._acc)):
            if self._acc_n == 0:
                self._acc_t0, self._acc_tilt = t, tilt
            self._acc += innov
            self._acc_n += 1
            # Mid-step, s is not changed yet: the sample with the beam half on
            # the edge (log 46: 6.6 cm of a 50 cm step) reaches the EKF for
            # one sample, 50 ms, before the step closes and s takes all of it.
            # The EKF's own smoothing makes that blip negligible.
            if self._acc_n >= self.p.max_step_samples:
                self._close(t)
            return
        self._close(t)

    def _close(self, t, force_drop=False):
        if self._acc_n == 0:
            return
        size, self._acc, self._acc_n = self._acc, 0.0, 0
        if force_drop or abs(size) < self.p.jump_min_m:
            return
        s_before = self.s
        self.s = s_before - size
        st = Step(self._acc_t0, size, s_before, self.s, self._acc_tilt)
        if abs(self.s) < self.p.floor_snap_m:
            self.s = 0.0
            st.s_after, st.reason = 0.0, "back on the floor"
            self._pending = None
        else:
            st.reason = "raised surface" if self.s > 0 else "below floor?"
            self._pending, self._pending_since = st, t
            self._evidence_seen = False
        self.steps.append(st)

    def _expire(self, t, evidence):
        st = self._pending
        if st is None:
            return
        if self.s > 0 and (evidence or self._evidence_seen):
            st.reason = "raised surface, confirmed"
            self._pending = None       # kept for good
            return
        if t - self._pending_since < self.p.confirm_timeout_s:
            return
        # No evidence in time: release. For s > 0 the vehicle climbs (safe).
        st.kept = False
        st.reason += " -> RELEASED (no evidence)"
        self.s = 0.0 if self.s < 0 else st.s_before
        self._pending = None
