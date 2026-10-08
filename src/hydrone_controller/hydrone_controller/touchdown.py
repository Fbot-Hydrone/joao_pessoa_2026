"""touchdown — has the vehicle stopped descending? ROS-free.

Ported unchanged from phase1_mission_node (_z_is_still + the descent check).

Landed = the FCU disarmed us, OR the altitude has STOPPED CHANGING after a real
descent. Stillness is relative, so it does not care what height the base is
at, and it cannot be satisfied on the way down: a descending vehicle moves far
more than `still_tol_m` across the window, a resting one moves only estimator
noise. The old absolute "z <= 0.5 m" test bailed out of LAND before touchdown
and never fired at all on a base higher than the one we left.
"""


class TouchdownDetector:
    def __init__(self, settle_s: float = 2.0, still_tol_m: float = 0.05,
                 min_descent_m: float = 0.30):
        self.settle_s = settle_s
        self.still_tol_m = still_tol_m
        self.min_descent_m = min_descent_m
        self.reset(None)

    def reset(self, entry_z):
        """Start a new descent from altitude `entry_z` (None if unknown)."""
        self.entry_z = entry_z
        self._hist = []

    def update(self, now: float, z: float) -> bool:
        """Feed one sample. True once still for a full window AND descended."""
        self._hist.append((now, z))
        # Keep the first sample at or before the cutoff — trimming to exactly
        # the window would leave a span one sample short of settle_s forever.
        cutoff = now - self.settle_s
        while len(self._hist) > 1 and self._hist[1][0] <= cutoff:
            self._hist.pop(0)
        if now - self._hist[0][0] < self.settle_s:
            return False
        zs = [h[1] for h in self._hist]
        still = (max(zs) - min(zs)) <= self.still_tol_m
        descended = (self.entry_z is not None
                     and self.entry_z - z >= self.min_descent_m)
        return still and descended
