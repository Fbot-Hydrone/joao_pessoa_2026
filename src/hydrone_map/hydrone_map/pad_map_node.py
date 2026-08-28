#!/usr/bin/env python3
"""
pad_map_node — the drone's memory of where the landing pads are.

Detections arrive as a stream of independent, noisy, per-frame guesses from two
cameras. This node turns that stream into a small persistent map: one entry per
physical pad, with a fused position, a height, and — critically — a flag saying
whether the drone has already landed on it.

That flag is what makes "land, take off, keep going" a mission rather than a
loop: without it the drone re-detects the pad it is standing on and lands on it
again forever.

    /hydrone/pads/detections  (PadDetection, many per second, from N cameras)
              |
              |  gate -> associate -> fuse -> prune
              v
    /hydrone/pads/map         (PadMap, the whole map on every change)
    /hydrone/pads/markers     (MarkerArray, the same thing for RViz)

Fusion
------
Nearest-neighbour association inside `merge_radius`, then a weighted running
mean of x and y. The weight is `confidence / max(range, 1)`, so a close-up look
from the down camera at 2 m outvotes a speck seen 20 m ahead — which is the
right ordering, because projection error grows with range in both routes
(depth noise linearly, ground-plane error with the ray's obliqueness).

Heights
-------
Pads sit at WHATEVER HEIGHT THEY SIT AT. There is no gate on a detection's z:
the competition's bases are at different elevations, and the drone may take off
from one of them, which puts every other pad at a negative z in the FCU's local
frame. Rejecting on height threw those away — the detector saw them and the map
refused them. A bad depth sample now costs a provisional entry that fails to
reach min_observations and is pruned, which is what that gate already does.

A detection's z comes from a flat-floor assumption, so every pad initially maps
at floor level. The arena has an elevated base (~0.5 m), and landing on it needs
its real height. The fix costs nothing extra: whenever the drone hovers within
`overhead_radius` of a mapped pad, the downward rangefinder is measuring the top
of THAT pad, so `height = drone_z - range` and the entry is corrected in place.
Until then `height_measured` stays false and the mission descends cautiously.

Motion
------
Detections are mapped ONLY while the vehicle is holding still. A projection is
composed with the vehicle pose, and while translating or slewing that pose is
stale by however long the estimator lags, so the pad lands in the map metres
from where it really is — the "strange detections while moving" that then cost
the search a leg to fly out and rule out. The search already works in
rotate/settle/look steps, so the still moments are the ones worth believing and
nothing is lost by ignoring the rest.

Pruning
-------
A pad seen once and never again was a false positive. Entries below
`min_observations` that have not been re-seen within `provisional_ttl_s` are
dropped. Confirmed, visited and takeoff-base entries are never pruned —
forgetting where you landed would undo the whole point.

The takeoff base
----------------
The drone always starts standing ON a base, and it is not one of the sites it is
meant to land on. Two things follow, and both are handled here rather than in
the mission:

  * **Nothing is mapped before the first arm.** On the ground the belly camera
    is a few centimetres above that base and the forward camera is looking along
    it at a grazing angle, which is exactly the geometry that produces a
    detection of the start base. Mapping it then would seed a candidate the
    mission has to fly to and rule out. `require_armed` (default true) drops
    every detection until /mavros/state first reports armed.
  * **It is declared, not detected.** `RegisterTakeoffBase` is called the
    instant the drone arms, with the position it is standing at. That entry
    carries `is_takeoff_base`, is never pruned, is never offered as a landing
    candidate, and claims later detections within `takeoff_base_radius` so a
    glancing view of it from the air cannot spawn a phantom pad beside it.

Registering costs nothing and is certain. The alternative — letting the detector
find the start base later and having the mission hover over it to decide — spends
a travel leg and a confirmation hover, and every second of both is accumulated
visual-odometry drift, on a question that was already answered.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from mavros_msgs.msg import State

from hydrone_msgs.msg import Pad, PadDetection, PadMap
from hydrone_msgs.srv import MarkPadVisited, RegisterTakeoffBase


class _Entry:
    """Mutable map entry. Converted to a Pad message on publish."""

    __slots__ = ("id", "x", "y", "z", "weight", "confidence", "observations",
                 "last_seen", "height", "height_measured", "visited",
                 "is_takeoff_base")

    def __init__(self, pad_id, x, y, z, weight, confidence, stamp):
        self.id = pad_id
        self.x = x
        self.y = y
        self.z = z
        self.weight = weight
        self.confidence = confidence
        self.observations = 1
        self.last_seen = stamp
        self.height = z
        self.height_measured = False
        self.visited = False
        self.is_takeoff_base = False

    def fuse(self, x, y, z, weight, confidence, stamp):
        total = self.weight + weight
        self.x = (self.x * self.weight + x * weight) / total
        self.y = (self.y * self.weight + y * weight) / total
        if not self.height_measured:
            # A measured height is ground truth; never let a projected z, which
            # assumes a flat floor, wash it back out.
            self.z = (self.z * self.weight + z * weight) / total
            self.height = self.z
        self.weight = total
        self.confidence = max(self.confidence, confidence)
        self.observations += 1
        self.last_seen = stamp


class PadMapNode(Node):

    def __init__(self, **kwargs):
        # **kwargs reaches rclpy's Node so tests can pass
        # parameter_overrides; the parameters here are read once, in __init__.
        super().__init__("pad_map", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("detections_topic", "/hydrone/pads/detections")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        # ONE topic in both worlds. REAL: ArduPilot reads the VL53L1X natively
        # over I2C and MAVROS publishes it here. SIM: rangefinder_bridge mimics
        # that publication (it also feeds MAVROS on /mavros/rangefinder, which
        # is where this MAVROS build's distance_sensor plugin SUBSCRIBES — that
        # topic is plumbing INTO the FCU, not a sensor bus to read).
        self.declare_parameter("range_topic",
                               "/mavros/distance_sensor/rangefinder")
        self.declare_parameter("map_topic", "/hydrone/pads/map")
        self.declare_parameter("marker_topic", "/hydrone/pads/markers")

        # Two detections farther apart than this are two different pads.
        # Comfortably larger than the projection error, comfortably smaller than
        # the spacing between the arena's bases.
        self.declare_parameter("merge_radius", 1.2)
        self.declare_parameter("min_confidence", 0.50)
        # Beyond this range a projection is too uncertain to seed the map with.
        self.declare_parameter("max_range_m", 30.0)
        # The arena, as [min_x, min_y, max_x, max_y] in the world frame. A
        # detection projected OUTSIDE it is refused.
        #
        # This is not a tidy-up, it is a wall. The ground-plane projection
        # intersects a camera ray with the floor, and a ray aimed at the top of
        # a wall crosses that plane on the FAR SIDE of it — so a base gets
        # mapped inside the masonry, the mission flies at it, and in a confined
        # arena that is a collision. MEASURED 2026-08-28 on an 8x8 m arena
        # (regions run -4..+4): candidates at (4.99, 3.57) and (4.97, 0.05),
        # and on earlier runs (10.23, -2.25) and (10.36, -2.26) — two and a
        # half metres past the wall.
        #
        # Nothing downstream can recover from this: pad_map has no idea where
        # the walls are, route just flies to the nearest candidate, and the
        # confirmation hover happens after the trip. The cheapest place to
        # refuse it is here, the moment the position is computed.
        self.declare_parameter("arena_bounds", [-4.5, -4.5, 4.5, 4.5])
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("provisional_ttl_s", 20.0)
        # How close overhead the drone must be for the rangefinder to be
        # measuring the pad rather than the floor beside it.
        self.declare_parameter("overhead_radius", 0.5)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("world_frame", "map")

        # Map nothing until the vehicle has armed at least once. On the ground
        # the cameras are looking at the base the drone is standing on, from the
        # grazing angles that detect it best; without this the map opens with a
        # candidate the mission then has to fly to and rule out. See the module
        # docstring.
        self.declare_parameter("require_armed", True)
        self.declare_parameter("state_topic", "/mavros/state")
        # Map nothing while the vehicle is MOVING. A projection is only as good
        # as the pose it is composed with, and while translating or slewing that
        # pose is stale by however long the estimator lags: the pad lands in the
        # map metres from where it is, and those phantoms are what the search
        # then flies out to rule out. Holding still costs nothing here — the
        # search is already rotate, settle, look.
        #
        # The EKF's own velocity, not a pose difference: at 30 Hz a centimetre
        # of pose noise differentiates to 0.3 m/s and would gate out everything.
        self.declare_parameter("velocity_topic",
                               "/mavros/local_position/velocity_local")
        self.declare_parameter("max_map_speed", 0.15)
        self.declare_parameter("max_map_yaw_rate_deg", 10.0)
        # How far from the registered takeoff base a detection is still THAT
        # base. Wider than merge_radius on purpose: the start base is seen from
        # the air at an angle, where the projection error is largest, and a
        # phantom pad 1.3 m from the real one would be indistinguishable from a
        # genuine second site.
        self.declare_parameter("takeoff_base_radius", 1.5)
        # Seconds between repeats of the same (camera, gate) rejection warning.
        # 0 logs every one — useful when counting how often a gate closes, noisy
        # at 10 Hz per camera.
        self.declare_parameter("reject_log_period_s", 5.0)

        p = lambda name: self.get_parameter(name).value
        self.merge_radius = float(p("merge_radius"))
        self.min_conf = float(p("min_confidence"))
        self.max_range = float(p("max_range_m"))
        self.min_obs = int(p("min_observations"))
        b = [float(v) for v in p("arena_bounds")]
        self.arena_bounds = (b[0], b[1], b[2], b[3])
        self.ttl = float(p("provisional_ttl_s"))
        self.overhead_radius = float(p("overhead_radius"))
        self.world_frame = p("world_frame")
        self.require_armed = bool(p("require_armed"))
        self.takeoff_base_radius = float(p("takeoff_base_radius"))
        self.max_map_speed = float(p("max_map_speed"))
        self.max_map_yaw_rate = math.radians(float(p("max_map_yaw_rate_deg")))
        self.reject_log_period = float(p("reject_log_period_s"))

        # ── State ───────────────────────────────────────────────────────────
        self.pads: dict[int, _Entry] = {}
        self._next_id = 0
        self.pose: PoseStamped | None = None
        self.range_m: float | None = None
        # Latched, never cleared: the gate is "has this vehicle ever been
        # armed", not "is it armed now". It is disarmed on every pad it lands
        # on, and detections must keep flowing then.
        self.armed_once = not self.require_armed
        self.takeoff_base_id: int | None = None
        # None until the first velocity message. The gate FAILS OPEN while it
        # is None: a topic that never arrives must not silently empty the map.
        self.twist: TwistStamped | None = None
        self._warned_no_twist = False
        # (camera, gate) -> when that combination last logged. See _reject.
        self._last_reject: dict[tuple[str, str], float] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── I/O ─────────────────────────────────────────────────────────────
        self.pub_map = self.create_publisher(PadMap, p("map_topic"), 10)
        self.pub_markers = self.create_publisher(MarkerArray,
                                                 p("marker_topic"), 10)
        self.create_subscription(PadDetection, p("detections_topic"),
                                 self._cb_detection, 20)
        self.create_subscription(PoseStamped, p("pose_topic"),
                                 self._cb_pose, sensor_qos)
        self.create_subscription(Range, p("range_topic"),
                                 self._cb_range, sensor_qos)
        self.create_subscription(TwistStamped, p("velocity_topic"),
                                 self._cb_twist, sensor_qos)
        # MAVROS publishes /mavros/state RELIABLE, unlike the sensor buses.
        self.create_subscription(State, p("state_topic"), self._cb_state, 10)

        self.srv_visited = self.create_service(
            MarkPadVisited, "/hydrone/pads/mark_visited", self._svc_visited)
        self.srv_takeoff_base = self.create_service(
            RegisterTakeoffBase, "/hydrone/pads/register_takeoff_base",
            self._svc_register_takeoff_base)

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._tick)

        gate = ("holding all detections until the first arm"
                if self.require_armed else "mapping from the first frame")
        self.get_logger().info(
            f"pad_map ready — fusing landing-pad detections, {gate}.")

    # ────────────────────────────────────────────────────────────────────────
    # Inputs
    # ────────────────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_state(self, msg: State):
        """Latch the first arm. Everything before it is discarded."""
        if msg.armed and not self.armed_once:
            self.armed_once = True
            self.get_logger().info(
                "vehicle armed — accepting pad detections from now on.")

    def _cb_twist(self, msg: TwistStamped):
        self.twist = msg

    def _moving(self) -> tuple[bool, str]:
        """Is the vehicle translating or slewing? (moving, why).

        Fails OPEN — reports "not moving" — until a velocity message has been
        seen at all, so a missing topic degrades to the old behaviour instead of
        silently refusing to map anything.
        """
        if self.twist is None:
            if not self._warned_no_twist:
                self._warned_no_twist = True
                # info, not warn: in this node a warn means a detection was
                # DROPPED, and this is the opposite — the gate standing aside.
                self.get_logger().info(
                    "no vehicle velocity yet; mapping detections regardless of "
                    "motion until one arrives.")
            return False, ""
        v = self.twist.twist.linear
        speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        yaw_rate = abs(self.twist.twist.angular.z)
        if speed > self.max_map_speed:
            return True, (f"vehicle is translating at {speed:.2f} m/s > "
                          f"max_map_speed {self.max_map_speed:.2f}")
        if yaw_rate > self.max_map_yaw_rate:
            return True, (f"vehicle is slewing at {math.degrees(yaw_rate):.0f} "
                          f"deg/s > max_map_yaw_rate_deg "
                          f"{math.degrees(self.max_map_yaw_rate):.0f}")
        return False, ""

    def _cb_range(self, msg: Range):
        """Cache the last plausible downward range reading."""
        if msg.min_range <= msg.range <= msg.max_range and math.isfinite(msg.range):
            self.range_m = float(msg.range)
        else:
            self.range_m = None

    def _cb_detection(self, msg: PadDetection):
        if not self.armed_once:
            # Not "ignore the start base" — ignore EVERYTHING. On the ground
            # there is no useful geometry to map from: the belly camera is
            # centimetres off a surface and the forward camera is looking at the
            # horizon, and both project through a pose the EKF has not settled.
            self._throttle_pregate()
            return
        # Every rejection below is SILENT unless it is logged: the detector goes
        # on drawing a confident box on its debug image while the map stays
        # empty, and from the outside that is indistinguishable from "the
        # detector never saw it". Say which gate closed, throttled so a
        # persistent false positive cannot flood the log.
        moving, why = self._moving()
        if moving:
            # Not a bad detection — a bad MOMENT to believe one. The pixel is
            # probably a real pad; the pose it would be projected through is
            # the part that is wrong.
            self._reject(msg, "motion", why)
            return
        if not msg.position_valid:
            # The pixel could not be placed in the world: no camera_info, no
            # vehicle pose, a stale pose, or a ray that never meets the ground
            # plane. Nothing to do with distance or confidence.
            self._reject(msg, "projection",
                         "position_valid is false — the detector could not "
                         "project it (no camera_info, no/stale pose, or a ray "
                         "at or above the horizon)")
            return
        if msg.confidence < self.min_conf:
            self._reject(msg, "confidence",
                         f"confidence {msg.confidence:.2f} < min_confidence "
                         f"{self.min_conf:.2f}")
            return
        if msg.range_m > self.max_range:
            self._reject(msg, "range",
                         f"range {msg.range_m:.1f} m > max_range_m "
                         f"{self.max_range:.1f} m")
            return
        min_x, min_y, max_x, max_y = self.arena_bounds
        if not (min_x <= msg.position.x <= max_x
                and min_y <= msg.position.y <= max_y):
            # Outside the arena means BEHIND A WALL, and a base cannot be
            # there. See arena_bounds: a ray aimed at the top of a wall crosses
            # the ground plane on the far side of it, so the projection lands
            # in the masonry — and the mission then flies at it.
            self._reject(msg, "outside the arena",
                         f"({msg.position.x:.2f}, {msg.position.y:.2f}) is "
                         f"outside [{min_x:.1f}, {min_y:.1f}] .. "
                         f"[{max_x:.1f}, {max_y:.1f}] — the ray crossed the "
                         f"ground plane past a wall")
            return
        # Weight close, confident looks far above distant ones — projection
        # error grows with range on both projection routes.
        weight = float(msg.confidence) / max(float(msg.range_m), 1.0)
        stamp = self.get_clock().now().nanoseconds * 1e-9

        entry = self._nearest(msg.position.x, msg.position.y)
        if entry is None:
            entry = _Entry(self._next_id, msg.position.x, msg.position.y,
                           msg.position.z, weight, msg.confidence, stamp)
            self.pads[self._next_id] = entry
            self._next_id += 1
            self.get_logger().info(
                f"pad {entry.id}: new candidate at "
                f"({entry.x:.2f}, {entry.y:.2f}) from {msg.camera} "
                f"@ {msg.range_m:.1f} m, conf {msg.confidence:.2f}")
        else:
            was_confirmed = entry.observations >= self.min_obs
            entry.fuse(msg.position.x, msg.position.y, msg.position.z,
                       weight, msg.confidence, stamp)
            if not was_confirmed and entry.observations >= self.min_obs:
                self.get_logger().info(
                    f"pad {entry.id}: CONFIRMED at "
                    f"({entry.x:.2f}, {entry.y:.2f}) after "
                    f"{entry.observations} looks, conf {entry.confidence:.2f}")

    def _nearest(self, x: float, y: float) -> _Entry | None:
        """Closest map entry whose claim radius covers (x, y), or None.

        Every entry claims `merge_radius`; the takeoff base claims
        `takeoff_base_radius`, which is wider. The comparison is on the ratio
        d/radius rather than on d, so an entry that only just reaches the point
        never outbids one that covers it comfortably — otherwise the takeoff
        base's wider reach would let it steal detections from a genuine pad
        sitting closer to them.
        """
        best, best_score = None, 1.0
        for entry in self.pads.values():
            radius = (self.takeoff_base_radius if entry.is_takeoff_base
                      else self.merge_radius)
            score = math.hypot(entry.x - x, entry.y - y) / radius
            if score < best_score:
                best, best_score = entry, score
        return best

    # ────────────────────────────────────────────────────────────────────────
    # Services
    # ────────────────────────────────────────────────────────────────────────

    def _svc_register_takeoff_base(self, request, response):
        """Declare the pad the drone is standing on as the takeoff base.

        Called once, at the first arm. Two cases:

          * an entry already covers that position — it IS this base, seen from
            the air on some earlier run or (with require_armed off) from the
            ground. Flag it in place rather than creating a duplicate 20 cm
            away.
          * nothing there — create the entry outright. It is not a detection and
            it does not pretend to be one: observations is left at 1 and
            confidence at 1.0, because its identity comes from the vehicle
            having been standing on it, which is better evidence than any
            camera produces.

        Idempotent: calling it again moves the flag to whatever is under the
        drone now and clears it from the previous holder, so a re-armed mission
        cannot end up with two takeoff bases.
        """
        x = float(request.position.x)
        y = float(request.position.y)
        z = float(request.position.z)

        entry = self._closest_within(x, y, self.takeoff_base_radius)
        if entry is None:
            entry = _Entry(self._next_id, x, y, z, weight=1.0, confidence=1.0,
                           stamp=self._now())
            self.pads[self._next_id] = entry
            self._next_id += 1
            created = "registered"
        else:
            created = f"claimed existing pad {entry.id} as"

        # The drone is standing on it, so its top surface is exactly the
        # altitude the drone reports. That is a measurement, not a projection.
        entry.height = z
        entry.z = z
        entry.height_measured = True

        for other in self.pads.values():
            other.is_takeoff_base = (other is entry)
        self.takeoff_base_id = entry.id

        # Registering the takeoff base OPENS THE GATE, if the arm has not
        # already. These are the same event: the mission calls this exactly
        # once, in the moment between arming and leaving the ground, so on a
        # real flight `armed_once` is already true here and this line does
        # nothing. It earns its place in a DRY RUN, where the vehicle never
        # arms and /mavros/state therefore never reports it — without this the
        # map would stay empty for the whole rehearsal and the thing being
        # rehearsed would have no input. Note this is strictly better than
        # running the rehearsal with `require_armed:=false`, which would map
        # the pre-arm frames the gate exists to throw away.
        if not self.armed_once:
            self.armed_once = True
            self.get_logger().info(
                "takeoff base registered without an arm (dry run) — accepting "
                "pad detections from now on.")

        response.success = True
        response.id = int(entry.id)
        response.message = f"pad {entry.id} is the takeoff base"
        self.get_logger().info(
            f"{created} TAKEOFF BASE pad {entry.id} at "
            f"({entry.x:.2f}, {entry.y:.2f}, {entry.height:.2f}) — it will "
            "never be offered as a landing candidate.")
        self._publish()
        return response

    def _svc_visited(self, request, response):
        entry = self.pads.get(int(request.id))
        if entry is None:
            response.success = False
            response.message = f"no pad {request.id} in the map"
            return response

        entry.visited = True
        if request.height_valid:
            entry.height = float(request.height)
            entry.z = float(request.height)
            entry.height_measured = True

        response.success = True
        response.message = f"pad {entry.id} marked visited"
        self.get_logger().info(
            f"pad {entry.id}: VISITED at ({entry.x:.2f}, {entry.y:.2f}, "
            f"{entry.height:.2f}) — it will not be targeted again")
        self._publish()
        return response

    # ────────────────────────────────────────────────────────────────────────
    # Periodic work
    # ────────────────────────────────────────────────────────────────────────

    def _tick(self):
        self._refine_height_from_rangefinder()
        self._prune()
        self._publish()

    def _refine_height_from_rangefinder(self):
        """Correct the height of whichever pad we are currently hovering over.

        This is what discovers that the second base is elevated. The projected z
        assumed a flat floor; the rangefinder measures the surface actually below
        the drone, and while we are inside `overhead_radius` of a mapped pad,
        that surface IS the pad.

        Visited pads are excluded. Their height was measured by standing on
        them, which beats any airborne reading, and letting a later fly-over
        rewrite it would let a glancing pass over the floor beside the pad undo
        the one measurement we are sure of.
        """
        if self.pose is None or self.range_m is None:
            return
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y
        pz = self.pose.pose.position.z

        entry = self._closest_within(px, py, self.overhead_radius,
                                     skip_visited=True,
                                     skip_takeoff_base=True)
        if entry is None:
            return

        height = pz - self.range_m
        # Only meaningful while genuinely airborne above it; on the ground the
        # rangefinder reads its own minimum and the subtraction is noise.
        if self.range_m < 0.25:
            return

        if not entry.height_measured or abs(height - entry.height) > 0.05:
            first = not entry.height_measured
            entry.height = height
            entry.z = height
            entry.height_measured = True
            if first:
                self.get_logger().info(
                    f"pad {entry.id}: height measured {height:.2f} m "
                    f"({'ELEVATED' if height > 0.15 else 'ground level'})")

    def _closest_within(self, x: float, y: float, radius: float,
                        skip_visited: bool = False,
                        skip_takeoff_base: bool = False) -> _Entry | None:
        best, best_d = None, radius
        for entry in self.pads.values():
            if skip_visited and entry.visited:
                continue
            if skip_takeoff_base and entry.is_takeoff_base:
                continue
            d = math.hypot(entry.x - x, entry.y - y)
            if d < best_d:
                best, best_d = entry, d
        return best

    def _prune(self):
        """Drop TRUE one-offs. Confirmed, visited and takeoff-base pads are kept.

        A single sighting is what noise produces. TWO sightings, from different
        frames, is a thing that was there both times — and dropping it because
        a third never came is a misreading of the evidence. The third never
        came because the drone TURNED AWAY, not because the pad stopped being
        there: not re-seeing something nobody looked at again is an absence of
        evidence, not evidence of absence. It is the same free/unknown
        distinction the occupancy map is built around.

        MEASURED 2026-08-27, a --ground-truth run over 6 bases:

            pad 1: dropped — 2 sighting(s) in 120 s
            pad 2: dropped — 2 sighting(s) in 120 s

        Both were real bases, seen at 6.9 m and 5.7 m. Losing them here is
        losing them for the whole attempt, because nothing downstream can
        target a pad that is no longer in the map. The mission has a far better
        judge than this timer anyway — a confirmation hover puts the belly
        camera a metre above the thing, and a candidate that fails it is
        blacklisted. Spending one hover on a doubtful lead costs ~25 s;
        dropping a real base costs the base.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        doomed = [pid for pid, e in self.pads.items()
                  if not e.visited
                  and not e.is_takeoff_base
                  and e.observations < 2
                  and now - e.last_seen > self.ttl]
        for pid in doomed:
            entry = self.pads.pop(pid)
            self.get_logger().info(
                f"pad {pid}: dropped — seen ONCE in {self.ttl:.0f} s, "
                f"treating it as a false positive")

    # ────────────────────────────────────────────────────────────────────────
    # Output
    # ────────────────────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _reject(self, msg: PadDetection, gate: str, why: str):
        """Say why a detection did not reach the map.

        WARN rather than INFO: a detection that the detector was confident
        enough to publish and the map then threw away is the one event where the
        two halves of the pipeline disagree, and it is invisible everywhere else.

        Throttled PER (camera, gate), by hand. rclpy's own throttle_duration_sec
        keys its state on the CALL SITE — (function, file, line) in
        rcutils_logger.py — and every gate here funnels through this one line, so
        the built-in version would give all four reasons and both cameras a
        single shared bucket. The belly camera rejecting something at 10 Hz would
        then hide the forward camera's reason for 5 s at a time, which is exactly
        the case this logging exists to diagnose.
        """
        key = (msg.camera, gate)
        now = self._now()
        if now - self._last_reject.get(key, -1e9) < self.reject_log_period:
            return
        self._last_reject[key] = now
        self.get_logger().warn(
            f"detection from {msg.camera} at ({msg.position.x:.2f}, "
            f"{msg.position.y:.2f}, {msg.position.z:.2f}) conf "
            f"{msg.confidence:.2f} range {msg.range_m:.1f} m REJECTED: {why}")

    def _throttle_pregate(self):
        self.get_logger().info(
            "detection ignored: nothing is mapped before the first arm "
            "(require_armed).", throttle_duration_sec=10.0)

    def _publish(self):
        now = self.get_clock().now().to_msg()

        msg = PadMap()
        msg.header.stamp = now
        msg.header.frame_id = self.world_frame
        msg.pads = [self._to_pad_msg(e, now) for e in
                    sorted(self.pads.values(), key=lambda e: e.id)]
        self.pub_map.publish(msg)
        self.pub_markers.publish(self._to_markers(msg))

    def _to_pad_msg(self, entry: _Entry, now) -> Pad:
        pad = Pad()
        pad.header.stamp = now
        pad.header.frame_id = self.world_frame
        pad.id = entry.id
        pad.position.x = float(entry.x)
        pad.position.y = float(entry.y)
        pad.position.z = float(entry.z)
        pad.confidence = float(entry.confidence)
        pad.observations = int(entry.observations)
        pad.last_seen.sec = int(entry.last_seen)
        pad.last_seen.nanosec = int((entry.last_seen % 1.0) * 1e9)
        pad.height = float(entry.height)
        pad.height_measured = bool(entry.height_measured)
        pad.visited = bool(entry.visited)
        pad.is_takeoff_base = bool(entry.is_takeoff_base)
        return pad

    def _to_markers(self, map_msg: PadMap) -> MarkerArray:
        """Disc + id label per pad.

        Grey = candidate, cyan = confirmed, green = landed on,
        ORANGE = the takeoff base. The takeoff base is checked first because it
        is the one state that says "never fly here"; it would otherwise render
        as an ordinary candidate and look exactly like the thing the mission is
        hunting for.
        """
        markers = MarkerArray()
        # A leading DELETEALL keeps pruned pads from lingering in RViz.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for pad in map_msg.pads:
            if pad.is_takeoff_base:
                colour = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.85)
            elif pad.visited:
                colour = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.85)
            elif pad.observations >= self.min_obs:
                colour = ColorRGBA(r=0.1, g=0.8, b=0.9, a=0.75)
            else:
                colour = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.45)

            disc = Marker()
            disc.header = map_msg.header
            disc.ns = "pads"
            disc.id = int(pad.id)
            disc.type = Marker.CYLINDER
            disc.action = Marker.ADD
            disc.pose.position = pad.position
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 1.0
            disc.scale.z = 0.05
            disc.color = colour
            markers.markers.append(disc)

            text = Marker()
            text.header = map_msg.header
            text.ns = "pad_labels"
            text.id = int(pad.id)
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pad.position.x
            text.pose.position.y = pad.position.y
            text.pose.position.z = pad.position.z + 0.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color = colour
            state = ("takeoff-base" if pad.is_takeoff_base
                     else "landed" if pad.visited
                     else "confirmed" if pad.observations >= self.min_obs
                     else "candidate")
            text.text = (f"pad {pad.id} [{state}] "
                         f"h={pad.height:.2f}{'' if pad.height_measured else '?'}")
            markers.markers.append(text)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = PadMapNode()
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
