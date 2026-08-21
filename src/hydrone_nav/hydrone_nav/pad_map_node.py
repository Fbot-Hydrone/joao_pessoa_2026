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
A detection's z comes from a flat-floor assumption, so every pad initially maps
at floor level. The arena has an elevated base (~0.5 m), and landing on it needs
its real height. The fix costs nothing extra: whenever the drone hovers within
`overhead_radius` of a mapped pad, the downward rangefinder is measuring the top
of THAT pad, so `height = drone_z - range` and the entry is corrected in place.
Until then `height_measured` stays false and the mission descends cautiously.

Pruning
-------
A pad seen once and never again was a false positive. Entries below
`min_observations` that have not been re-seen within `provisional_ttl_s` are
dropped. Confirmed and visited entries are never pruned — forgetting where you
landed would undo the whole point.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from hydrone_msgs.msg import Pad, PadDetection, PadMap
from hydrone_msgs.srv import MarkPadVisited


class _Entry:
    """Mutable map entry. Converted to a Pad message on publish."""

    __slots__ = ("id", "x", "y", "z", "weight", "confidence", "observations",
                 "last_seen", "height", "height_measured", "visited")

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
        # A "pad" floating 2 m up is a detector artefact, not a landing site.
        self.declare_parameter("max_pad_height", 2.0)
        self.declare_parameter("min_pad_height", -0.5)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("provisional_ttl_s", 20.0)
        # How close overhead the drone must be for the rangefinder to be
        # measuring the pad rather than the floor beside it.
        self.declare_parameter("overhead_radius", 0.5)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("world_frame", "map")

        p = lambda name: self.get_parameter(name).value
        self.merge_radius = float(p("merge_radius"))
        self.min_conf = float(p("min_confidence"))
        self.max_range = float(p("max_range_m"))
        self.max_height = float(p("max_pad_height"))
        self.min_height = float(p("min_pad_height"))
        self.min_obs = int(p("min_observations"))
        self.ttl = float(p("provisional_ttl_s"))
        self.overhead_radius = float(p("overhead_radius"))
        self.world_frame = p("world_frame")

        # ── State ───────────────────────────────────────────────────────────
        self.pads: dict[int, _Entry] = {}
        self._next_id = 0
        self.pose: PoseStamped | None = None
        self.range_m: float | None = None

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

        self.srv_visited = self.create_service(
            MarkPadVisited, "/hydrone/pads/mark_visited", self._svc_visited)

        self.create_timer(1.0 / max(float(p("publish_hz")), 0.1), self._tick)

        self.get_logger().info("pad_map ready — fusing landing-pad detections.")

    # ────────────────────────────────────────────────────────────────────────
    # Inputs
    # ────────────────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_range(self, msg: Range):
        """Cache the last plausible downward range reading."""
        if msg.min_range <= msg.range <= msg.max_range and math.isfinite(msg.range):
            self.range_m = float(msg.range)
        else:
            self.range_m = None

    def _cb_detection(self, msg: PadDetection):
        if not msg.position_valid or msg.confidence < self.min_conf:
            return
        if msg.range_m > self.max_range:
            return
        if not (self.min_height <= msg.position.z <= self.max_height):
            # Projected well off the floor: a reflection, or a bad depth sample.
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
        """Closest map entry within merge_radius, or None."""
        best, best_d = None, self.merge_radius
        for entry in self.pads.values():
            d = math.hypot(entry.x - x, entry.y - y)
            if d < best_d:
                best, best_d = entry, d
        return best

    # ────────────────────────────────────────────────────────────────────────
    # Services
    # ────────────────────────────────────────────────────────────────────────

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
                                     skip_visited=True)
        if entry is None:
            return

        height = pz - self.range_m
        if not (self.min_height <= height <= self.max_height):
            return
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
                        skip_visited: bool = False) -> _Entry | None:
        best, best_d = None, radius
        for entry in self.pads.values():
            if skip_visited and entry.visited:
                continue
            d = math.hypot(entry.x - x, entry.y - y)
            if d < best_d:
                best, best_d = entry, d
        return best

    def _prune(self):
        """Drop stale one-off candidates. Confirmed and visited pads are kept."""
        now = self.get_clock().now().nanoseconds * 1e-9
        doomed = [pid for pid, e in self.pads.items()
                  if not e.visited
                  and e.observations < self.min_obs
                  and now - e.last_seen > self.ttl]
        for pid in doomed:
            entry = self.pads.pop(pid)
            self.get_logger().info(
                f"pad {pid}: dropped — {entry.observations} sighting(s) in "
                f"{self.ttl:.0f} s, treating it as a false positive")

    # ────────────────────────────────────────────────────────────────────────
    # Output
    # ────────────────────────────────────────────────────────────────────────

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
        return pad

    def _to_markers(self, map_msg: PadMap) -> MarkerArray:
        """Disc + id label per pad. Grey = candidate, cyan = confirmed,
        green = landed on."""
        markers = MarkerArray()
        # A leading DELETEALL keeps pruned pads from lingering in RViz.
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for pad in map_msg.pads:
            if pad.visited:
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
            state = ("landed" if pad.visited
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
