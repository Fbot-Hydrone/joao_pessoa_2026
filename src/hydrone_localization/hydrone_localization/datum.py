"""datum — pin the EKF's height datum to the arena floor, on the ground.

ROS-free. Why this exists (bench, 2026-11-18): with the rangefinder as its
height source, EKF3 reports z = range*cos(tilt) - T, where T is a terrain state
it learns while ON THE GROUND and then holds (it survived a 20 s gap in the
feed, and through every flight of 2026-11-17 z - range stayed -0.11..-0.14).
T is learned during whatever the EKF did before rangefinder data arrived. With
the DroneCAN sensor that is from boot, and T came out at ~0.12. With the
Orin's feed it cannot be: the FCU's EKF starts seconds after power-on, the
Orin and the Ethernet link a minute later, and meanwhile the EKF runs on the
barometer, whose drift ends up in T (measured: T = 1.14 and 1.29 on two boots,
so z = -0.46 standing on a 0.5 m base).

Since z follows the fed range 1:1 once T is fixed, the fix is a bias on the
feed: while DISARMED and STILL, compare the FCU's z with what it should be,

    z_target = h - ground_range          (h = surface + range*cos(tilt))

i.e. the height of the landing gear above the floor (0 on the floor, 0.5 on
the 0.5 m base), and move the bias by the error. Frozen at arming, so nothing
changes in flight. ground_range is the range the sensor reads standing on a
flat surface (0.17-0.18 m on the big drone).
"""

from collections import deque
from dataclasses import dataclass


@dataclass
class DatumParams:
    ground_range_m: float = 0.17
    # Leave errors under this alone. The EKF's z wanders +-4 cm around the
    # target after a correction (bench 2026-11-18: -0.04, then +0.05);
    # chasing that only hunts.
    tol_m: float = 0.05
    window_s: float = 3.0        # stillness window
    still_m: float = 0.02        # max spread of h and z inside the window
    trend_m: float = 0.01        # max |mean(last third) - mean(first third)|
    # On the ground the EKF takes SECONDS to follow the feed (bench: a +0.24 m
    # change was still moving z 4 s later), and after a gap in the feed it
    # walks back from the barometer just as slowly. Judging a moving z
    # overshoots, so wait for it to stop.
    settle_s: float = 8.0        # wait after a change
    min_feed_s: float = 10.0     # continuous feed before the first judgement
    max_bias_m: float = 3.0
    max_misses: int = 3          # the EKF did not follow -> give up


class DatumCal:
    def __init__(self, p: DatumParams = None):
        self.p = p or DatumParams()
        self.bias = 0.0
        self.adjustments = []    # (t, error, new_bias)
        self.disabled = None     # reason, once given up
        # True once a still window was judged within tol since the last gap
        # or change: z IS height above the floor. Not reset by arming.
        self.ok = False
        self._win = deque()
        self._last_adjust_t = None
        self._last_err = None
        self._misses = 0
        self._feed_start_t = None
        self._last_t = None

    def error(self):
        """Mean z - z_target over the current window, or None."""
        if not self._win:
            return None
        n = len(self._win)
        return sum(z - (h - self.p.ground_range_m)
                   for _, h, z in self._win) / n

    def update(self, t, h, z, armed) -> bool:
        """Feed one sample. Returns True when the bias changed."""
        if armed or self.disabled or h is None or z is None:
            self._win.clear()
            return False
        if self._last_t is None or t - self._last_t > 0.5:
            self._feed_start_t = t          # (re)started after a gap: the
            self._win.clear()               # EKF may have been on the baro
            self.ok = False
        self._last_t = t
        if t - self._feed_start_t < self.p.min_feed_s:
            return False
        self._win.append((t, h, z))
        while self._win and t - self._win[0][0] > self.p.window_s:
            self._win.popleft()
        if t - self._win[0][0] < 0.8 * self.p.window_s:
            return False
        hs = [w[1] for w in self._win]
        zs = [w[2] for w in self._win]
        if (max(hs) - min(hs) > self.p.still_m
                or max(zs) - min(zs) > self.p.still_m):
            return False                       # being carried / settling
        k = max(1, len(zs) // 3)
        if abs(sum(zs[-k:]) / k - sum(zs[:k]) / k) > self.p.trend_m:
            return False                       # EKF still converging
        if (self._last_adjust_t is not None
                and t - self._last_adjust_t < self.p.settle_s):
            return False
        e = self.error()
        if self._last_err is not None:
            # The last change should have taken most of the error away.
            # A miss is a large error the last change did not shrink — not
            # the few-cm hunting around the target.
            if abs(e) > 2 * self.p.tol_m and abs(e) > 0.5 * abs(
                    self._last_err):
                self._misses += 1
                if self._misses >= self.p.max_misses:
                    self.disabled = (f"EKF z does not follow the feed (error "
                                     f"{e:+.2f} m after {self._misses} "
                                     "corrections)")
                    return False
            else:
                self._misses = 0
            self._last_err = None
        if abs(e) <= self.p.tol_m:
            self.ok = True
            return False
        new = self.bias - e
        if abs(new) > self.p.max_bias_m:
            self.disabled = (f"datum bias would be {new:+.2f} m "
                             f"(> {self.p.max_bias_m} m)")
            return False
        self.bias = new
        self.ok = False
        self.adjustments.append((t, e, new))
        self._last_adjust_t = t
        self._last_err = e
        self._win.clear()
        return True
