"""visual_descent — centre on a base by camera, descending, then hand to LAND.

ROS-free. Modelled on Black Bee's CBR 2025 phase 1 (cbr-2025/mapping,
CenterOnDetection), which is what this stack's learned servo (servo.py) was
trying and failing to do:

* TRACK THE DETECTION NEAREST THE TARGET PIXEL. With two or three bases in
  frame, "the last detection published" chased the neighbour (2026-10-01,
  seed 100); the one nearest centre is the one under the vehicle.
* A FIXED, KNOWN MAPPING, not a learned one. Body frame is ROS FLU (x
  forward, y LEFT). For this stack's belly camera (down_cam_mimic_node TF:
  pitch +90 deg, standard optical frame) image +u is body RIGHT (-y) and
  image +v is body BACK (-x), so `axes` = (-1, -1); the scale is height / fx.
  MEASURED 2026-10-08, seed 2: with the y sign wrong the error grew from 78 to
  263 px while "correcting" until the base was lost.
  The learned Jacobian poisoned itself on neighbour detections and froze.
* TWO PHASES. Centre at cruise height until within `coarse_px`, then descend
  towards `descend_to_m` above the base top while still centring, with a
  deadband that tightens from `coarse_px` to `fine_px` near the bottom.
  READY = low enough AND within `fine_px`. Then the caller sends LAND.

Every call returns a Step: a body-frame position nudge (dx, dy), a vertical
nudge dz (negative = down), and the phase. The caller rotates (dx, dy) by yaw
and adds them to its setpoint — the controller holds position; nudging the
target IS a P velocity loop on a position controller.
"""

import math
from dataclasses import dataclass

CENTRING, DESCENDING, READY, LOST = "centring", "descending", "ready", "lost"


@dataclass
class Step:
    phase: str
    dx: float = 0.0          # body x (forward), m
    dy: float = 0.0          # body y (LEFT, ROS FLU), m
    dz: float = 0.0          # m, negative = descend
    err_px: float = 0.0
    picked: tuple | None = None


class VisualDescent:
    def __init__(self, *, target_uv=(320.0, 240.0), coarse_px=100.0,
                 fine_px=40.0, gain=0.6, max_step_m=0.20,
                 descend_to_m=1.2, descend_step_m=0.40,
                 lost_after_s=3.0, axes=(-1.0, -1.0)):
        self.target_uv = target_uv
        self.coarse_px = coarse_px
        self.fine_px = fine_px
        self.gain = gain
        self.max_step = max_step_m
        self.descend_to = descend_to_m
        self.descend_step = descend_step_m
        self.lost_after = lost_after_s
        # (sign of body y per +u, sign of body x per +v), FLU. A camera bolted
        # on rotated or mirrored flips one of these.
        self.axes = axes
        self.reset()

    def reset(self):
        self.phase = CENTRING
        self._last_seen = None

    def pick(self, detections_uv):
        """The detection nearest the target pixel, or None."""
        if not detections_uv:
            return None
        u0, v0 = self.target_uv
        return min(detections_uv,
                   key=lambda d: math.hypot(d[0] - u0, d[1] - v0))

    def update(self, detections_uv, height_m, fx, now) -> Step:
        """One control tick.

        detections_uv  [(u, v), ...] from the latest frame (may be empty)
        height_m       camera height above the base TOP
        fx             focal length in px (belly CameraInfo)
        now            seconds, any monotonic clock
        """
        if self.phase == READY:
            return Step(READY)
        d = self.pick(detections_uv)
        if d is None:
            if self._last_seen is None:
                self._last_seen = now
            if now - self._last_seen > self.lost_after:
                self.phase = LOST
            return Step(self.phase if self.phase == LOST else self.phase)
        self._last_seen = now

        eu = d[0] - self.target_uv[0]
        ev = d[1] - self.target_uv[1]
        err = math.hypot(eu, ev)
        low = height_m <= self.descend_to + 0.05
        # coarse_px only gates the START of the descent; once descending the
        # vehicle keeps correcting down to fine_px (MEASURED 2026-10-08, seed
        # 2: a coarse deadband held a 50 px error all the way down).
        deadband = self.fine_px

        if self.phase == CENTRING and err <= self.coarse_px:
            self.phase = DESCENDING
        if self.phase == DESCENDING and low and err <= self.fine_px:
            self.phase = READY
            return Step(READY, err_px=err, picked=d)

        dx = dy = 0.0
        if err > deadband and fx and fx > 0:
            m_per_px = max(height_m, 0.2) / fx
            dy = self.axes[0] * self.gain * eu * m_per_px
            dx = self.axes[1] * self.gain * ev * m_per_px
            n = math.hypot(dx, dy)
            if n > self.max_step:
                dx, dy = dx * self.max_step / n, dy * self.max_step / n
        dz = 0.0
        if self.phase == DESCENDING and not low:
            dz = -min(self.descend_step, height_m - self.descend_to)
        return Step(self.phase, dx, dy, dz, err, d)
