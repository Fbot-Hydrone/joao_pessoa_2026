"""servo — centring the vehicle on what the belly camera sees, without knowing
how the belly camera is mounted.

Turning a pixel offset into a position nudge needs the mapping from image
pixels to body metres. That mapping is a rotation, a sign and a scale, and
NONE of the three can be assumed here:

* the scale changes with height, on every frame of a descent;
* the rotation depends on how the camera is bolted on, and the real airframe's
  is not the simulator's;
* the sign follows from the rotation, and getting it backwards does not centre
  slowly — it walks the vehicle AWAY from the pad, accelerating.

MEASURED 2026-08-28 on the simulator, regressing pixel motion against body
motion over 42 steps: `dv` tracked body x at +0.810 and `du` tracked nothing
at all (coefficients 0.00002 and 0.00014). Even here, with the mount known
from a config file, there is no clean constant to write down. On hardware
there is less.

So this does not use a constant. It ESTIMATES the mapping while it servoes,
by Broyden's rule: command a step, watch how the error moved, correct the
estimate. That is standard uncalibrated visual servoing, and it converges from
a wrong initial guess — including one with the signs inverted — in a handful of
steps. A camera mounted at any angle, mirrored, or rotated 90 degrees works
with no change and no calibration flight.

What it CANNOT learn is where the camera points when the vehicle is level. If
the lens is off-centre on the real airframe, centring the pad on the image
centre parks the vehicle off the pad by that misalignment times the height.
That offset is one constant, it is a property of the airframe, and it is
`target_uv` — measured once by hovering over a known pad and reading where it
sits in frame. Everything else is learned in flight.

ROS-free, like route.py and planner.py: pixels in, metres out.
"""

import math

# How much of the correction to apply per step. Below 1 because the estimate is
# being learned from the same steps it is driving: a full-gain step on a bad
# Jacobian is a large move in a direction nothing has verified yet.
GAIN = 0.5

# Never command more than this in one nudge, whatever the estimate says. The
# guard that turns a bad Jacobian into a slow correction instead of a flyaway.
MAX_STEP_M = 0.25

# Below this the pad is centred and moving again would only add drift.
DEADBAND_PX = 12.0

# A step smaller than this teaches the estimate nothing — the pixel change is
# mostly noise — so it is not used for an update.
MIN_LEARN_STEP_M = 0.02


class VisualServo:
    """Drives a pixel error to zero by nudging position, learning as it goes.

    Usage, once per detection:

        step = servo.update((u, v), height_m)   # (dx, dy) in the BODY frame
        if step is None:                        # centred, or nothing to do
            ...

    The caller rotates (dx, dy) into the world frame by the vehicle's yaw and
    adds it to the position setpoint.
    """

    def __init__(self, *, target_uv=(320.0, 240.0), gain=GAIN,
                 max_step_m=MAX_STEP_M, deadband_px=DEADBAND_PX,
                 min_learn_step_m=MIN_LEARN_STEP_M, fov_rad=math.pi / 2.0,
                 image_width=640.0):
        self.target_uv = target_uv
        self.gain = gain
        self.max_step_m = max_step_m
        self.deadband_px = deadband_px
        self.min_learn_step_m = min_learn_step_m
        self.fov_rad = fov_rad
        self.image_width = image_width
        self._J = None            # d(pixel) / d(body metre), 2x2
        self._cols = [(0.0, 0.0), (0.0, 0.0)]
        self._probe = 0           # 0, 1 = probing; 2 = servoing
        self._last_err = None
        self._last_step = None
        self.updates = 0

    # ── the estimate ────────────────────────────────────────────────────────

    def _probe_size(self, height_m):
        """How far to probe. Big enough that the pixel change is signal, small
        enough that being wrong about it costs nothing."""
        return max(self.min_learn_step_m * 2.0,
                   min(self.max_step_m, 0.08 * max(height_m, 0.5)))

    def _broyden(self, d_err, d_step):
        """J += ((d_err - J d_step) d_step^T) / (d_step^T d_step).

        Refines the estimate from the servo's own steps once the probes have
        established it. Alone it is not enough: from a diagonal guess against
        an anti-diagonal mount, every step is collinear and only one direction
        of J ever gets corrected — MEASURED in test, a 90 degree mount diverged
        to 1.66 m. That is what the probes exist to prevent.
        """
        n2 = d_step[0] ** 2 + d_step[1] ** 2
        if n2 < 1e-12:
            return
        pred = [self._J[i][0] * d_step[0] + self._J[i][1] * d_step[1]
                for i in range(2)]
        for i in range(2):
            r = d_err[i] - pred[i]
            self._J[i][0] += r * d_step[0] / n2
            self._J[i][1] += r * d_step[1] / n2
        self.updates += 1

    # ── the step ────────────────────────────────────────────────────────────

    def update(self, uv, height_m):
        """Pixel reading -> (dx, dy) to command in the body frame, or None.

        The first two calls that need a move are PROBES: one along body x, one
        along body y. Two orthogonal probes determine a 2x2 Jacobian exactly,
        for any invertible mount — no rotation, sign or scale is assumed, and
        nothing has to converge. Servoing starts on the third call, already
        knowing which way the vehicle has to go.
        """
        err = (uv[0] - self.target_uv[0], uv[1] - self.target_uv[1])
        moved_enough = (self._last_step is not None
                        and math.hypot(*self._last_step) >= self.min_learn_step_m)

        if self._last_err is not None and moved_enough:
            d_err = (err[0] - self._last_err[0], err[1] - self._last_err[1])
            if self._probe < 2:
                # A probe along one axis IS the corresponding column of J.
                d = self._last_step[self._probe]
                if abs(d) > 1e-9:
                    self._cols[self._probe] = (d_err[0] / d, d_err[1] / d)
                self._probe += 1
                if self._probe == 2:
                    self._J = [[self._cols[0][0], self._cols[1][0]],
                               [self._cols[0][1], self._cols[1][1]]]
            elif self._J is not None:
                self._broyden(d_err, self._last_step)

        if math.hypot(*err) <= self.deadband_px:
            self._last_err, self._last_step = err, None
            return None

        if self._probe < 2:
            d = self._probe_size(height_m)
            step = (d, 0.0) if self._probe == 0 else (0.0, d)
            self._last_err, self._last_step = err, step
            return step

        step = self._solve(err)
        if step is None:
            # The probes produced a singular estimate — the mount cannot be
            # inverted from what was seen. Probe again rather than invert noise.
            self._probe = 0
            self._J = None
            self._last_err, self._last_step = err, None
            return None

        step = (-self.gain * step[0], -self.gain * step[1])
        n = math.hypot(*step)
        if n > self.max_step_m:
            step = (step[0] * self.max_step_m / n, step[1] * self.max_step_m / n)

        self._last_err, self._last_step = err, step
        return step

    def _solve(self, err):
        """J^-1 err, or None when J is singular."""
        (a, b), (c, d) = self._J[0], self._J[1]
        det = a * d - b * c
        if abs(det) < 1e-9:
            return None
        return ((d * err[0] - b * err[1]) / det,
                (-c * err[0] + a * err[1]) / det)

    def reset(self):
        """Forget the run, keep the estimate. A new pad is the same camera, so
        the probes are not paid for twice."""
        self._last_err = None
        self._last_step = None
