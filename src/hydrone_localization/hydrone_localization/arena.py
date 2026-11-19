"""arena — where the arena is, and which parts of it may hold raised things.

ROS-free. The geometry is stated in the ARENA's own frame, not the FCU's: the
takeoff point moves a few centimetres every time a person sets the drone down,
the arena does not.

Arena frame: origin at one corner, x along the side the zones split, y across,
both in metres; z up from the floor.

Zones are strips laid side by side along x, each spanning the full width
(they touch both long walls, and the first and last touch the end walls):

    zone_edges_x_m = [0.0, 2.0, 8.0]
    zone_raised    = [true, true]      # may hold something taller than floor

    y=Ly +-----------+-----------------------------+
         | takeoff + |                             |
         |  house    |        landing bases        |
    y=0  +-----------+-----------------------------+
        x=0         x=2                           x=8

`zone_raised` says whether a step the rangefinder takes inside that zone is
believed on its own (a base or the house can be there). A zone marked false
needs the belly camera to vouch for each raised surface.

FCU frame. The FCU's local frame has its origin where the EKF started and its
own yaw. It is tied to the arena by ONE known fact: the drone is set down on
the takeoff base, whose centre and heading in the arena are parameters
(`takeoff_base_arena = [x, y, yaw_deg]`). Latching the FCU pose at the first
arming gives the transform; a person's placement error shifts every zone edge
by that error and nothing worse.
"""

import math
from dataclasses import dataclass, field


@dataclass
class Arena:
    size_xy: tuple = (0.0, 0.0)            # (Lx, Ly), 0 = unknown
    takeoff_base: tuple = (0.0, 0.0, 0.0)  # centre x, y and heading (deg)
    zone_edges_x: list = field(default_factory=list)
    zone_raised: list = field(default_factory=list)
    margin_m: float = 0.15                 # zone edges count this much wider

    # FCU <- arena, set by latch()
    _o: tuple = None                       # FCU xy of the arena origin
    _c: float = 1.0
    _s: float = 0.0

    def validate(self):
        """Raise on inconsistent geometry. Empty zones are allowed (= none)."""
        e = list(self.zone_edges_x)
        if not e:
            return
        if len(e) != len(self.zone_raised) + 1:
            raise ValueError("zone_edges_x needs one more entry than "
                             "zone_raised (edges bound the zones)")
        if any(b <= a for a, b in zip(e, e[1:])):
            raise ValueError("zone_edges_x must increase")
        if self.size_xy[0] > 0 and (e[0] > 1e-6
                                    or abs(e[-1] - self.size_xy[0]) > 1e-6):
            raise ValueError("zones must cover the arena length exactly, "
                             "from 0 to Lx — no gap at either end")

    @property
    def latched(self) -> bool:
        return self._o is not None

    def latch(self, fcu_x, fcu_y, fcu_yaw):
        """The vehicle is on the takeoff base, at FCU pose (x, y, yaw)."""
        bx, by, byaw = self.takeoff_base
        rot = fcu_yaw - math.radians(byaw)    # arena -> FCU rotation
        self._c, self._s = math.cos(rot), math.sin(rot)
        # FCU position of the arena origin: base_fcu - R * base_arena
        self._o = (fcu_x - (self._c * bx - self._s * by),
                   fcu_y - (self._s * bx + self._c * by))

    def to_arena(self, x, y):
        dx, dy = x - self._o[0], y - self._o[1]
        return (self._c * dx + self._s * dy, -self._s * dx + self._c * dy)

    def to_fcu(self, ax, ay):
        return (self._o[0] + self._c * ax - self._s * ay,
                self._o[1] + self._s * ax + self._c * ay)

    def zone_at(self, ax, ay):
        """Index of the zone holding arena point (ax, ay), else None. Points
        within margin_m of an edge belong to BOTH neighbours: the raised flag
        is OR-ed by raised_at, so a step on a boundary is not lost."""
        hits = self.zones_at(ax, ay)
        return hits[0] if hits else None

    def zones_at(self, ax, ay):
        e, m = self.zone_edges_x, self.margin_m
        ly = self.size_xy[1]
        if ly > 0 and not (-m <= ay <= ly + m):
            return []
        return [i for i in range(len(e) - 1)
                if e[i] - m <= ax <= e[i + 1] + m]

    def raised_at(self, ax, ay) -> bool:
        """With NO zones configured every point may be raised: a step is then
        held on its own, never released. That is the safe default — releasing
        a step over the 1.5 m house (no blue for the camera to vouch) would
        make the vehicle climb 1.5 m, up against the 2.5 m plan ceiling."""
        if not self.zone_edges_x:
            return True
        return any(self.zone_raised[i] for i in self.zones_at(ax, ay))


def sensor_ground_point(x, y, yaw, offset_xy):
    """Where a body-mounted downward sensor points, in the FCU frame (level
    approximation: at the tilts flown, < 6 deg, the beam lands within
    h*tan(6deg) = 15 cm at 1.5 m of this point, inside the zone margin)."""
    ox, oy = offset_xy
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * ox - s * oy, y + s * ox + c * oy


def rangefinder_in_image(rng_body_xy, cam_body_xy, depth_m, fx, cx, cy):
    """Pixel where the rangefinder spot appears in the belly camera.

    Body frame ROS FLU (x forward, y LEFT). The belly camera's image +u is
    body RIGHT and +v body BACK (visual_descent's measured mapping), so a spot
    AHEAD of the lens is UP in the image and one to its LEFT is LEFT.
    """
    dx = rng_body_xy[0] - cam_body_xy[0]
    dy = rng_body_xy[1] - cam_body_xy[1]
    k = fx / max(depth_m, 0.1)
    return cx - dy * k, cy - dx * k


def camera_vouches(dets, spot_uv, margin_px=20.0):
    """True if any belly detection's disc covers the rangefinder spot.

    dets: [(u, v, radius_px), ...]. radius_px is the detector's equivalent
    radius of the blue region; the base's top is at least that big.
    """
    su, sv = spot_uv
    return any(math.hypot(u - su, v - sv) <= r + margin_px
               for u, v, r in dets)
