#!/usr/bin/env python3
"""landmark_correction_node — measure the pose error from the landing bases.

The arithmetic lives in hydrone_localization.landmark, which knows nothing
about ROS. This node is the plumbing: it watches the pad map, the detections
and the vehicle pose, and turns them into one number — how far the estimate has
walked away from the world.

    /hydrone/pads/map          PadMap        what the map remembers
    /hydrone/pads/detections   PadDetection  what the camera sees right now
    /mavros/local_position/pose              where the vehicle thinks it is
        |
        v
    /hydrone/localization/correction    PoseWithCovarianceStamped

Two measurements, in the order they become trustworthy:

**The anchor.** The takeoff base was registered from the drone's own position
at the moment it armed — no camera, no projection, no accumulated pose. It is
the only absolute fix in the map, so the gap between where it was registered
and where the map now holds it is pure drift. This needs nothing to happen: it
is available from the first map update.

**Re-observation.** Fly back over a settled pad and the detector projects it
somewhere. The pad has not moved, so the offset is the pose error. This needs
association and settling, both of which live in the library.

An OBSERVER, deliberately
-------------------------
This node publishes a measurement and steers nothing. /zed/zed_node/odom is
what the EKF flies on with GPS off, so a bad correction injected there is not a
worse map, it is an aircraft flying to the wrong place. The correction has to
be shown to track the true error before anything consumes it, and with the
simulator's ground truth that is a measurement, not an opinion:
`err_norm` in the odom_error CSV is the same quantity this topic estimates, so
the two can simply be plotted against each other.

`apply` exists for when that measurement says yes. It defaults to false.

MEASURED 2026-08-27, and the answer is NO
-----------------------------------------
Flown on the VO (odom_source:=vo), a 5.5 minute Phase 1 run:

    true position error (odom_error CSV)   7.83 m
    what the anchor reported               0.01 m
    what the one re-observation reported    0.11 m

The correction does not see the drift, and the reason is structural rather
than a tuning problem — which is exactly why this node was built as an
observer and measured before anything consumed it.

**Both sides of the subtraction live in the same drifting frame.** The map
entry was built by projecting a detection through the vehicle pose. The fresh
detection is projected through the same pose, a few minutes later. When that
pose has walked 7 m, BOTH the recorded position and the new projection have
walked with it, and the difference between them is not the drift — it is the
noise between two projections. The 0.11 m above is that noise.

The anchor is worse, and blind by construction. `map` is the EKF's own frame,
the takeoff base was registered at the vehicle's position in that frame, and
`home` is the same number. Their difference is zero no matter how far `map` has
drifted from the world. Drift of a frame is not observable from inside it.

What WOULD see it, in increasing order of work:

1. **The landing.** When the drone physically touches the takeoff base, its
   `map` position must read the armed position. Any discrepancy IS the drift,
   measured against the world by contact rather than by projection. One
   measurement per attempt, no association, no ambiguity — and Phase 1 already
   flies that return leg.
2. **An independent range.** The rangefinder and the depth image give a
   pad's distance without going through the accumulated pose. A re-observation
   that constrains range and bearing, from a vantage point far from the one
   that built the entry, does carry information the map does not already have.
3. **A real pose graph** over those constraints. Which is the point at which
   this stops being "landmark correction" and becomes SLAM, and should only be
   started once 1 and 2 have been measured and found insufficient.

So this node stays an observer, and `apply` stays false. What it publishes is
honest — it is a measurement of the residual between two projections, it is
just not a measurement of the drift, and the difference matters enough to be
written down here rather than discovered again later.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hydrone_msgs.msg import PadDetection, PadMap

from hydrone_localization.landmark import (LandmarkTracker, anchor_drift,
                                           associate, drift_from_observations,
                                           is_trustworthy, weight_of)


class LandmarkCorrectionNode(Node):

    def __init__(self, **kwargs):
        super().__init__("landmark_correction", **kwargs)

        self.declare_parameter("in_map", "/hydrone/pads/map")
        self.declare_parameter("in_detection", "/hydrone/pads/detections")
        self.declare_parameter("in_pose", "/mavros/local_position/pose")
        self.declare_parameter("out_correction",
                               "/hydrone/localization/correction")
        # Where the drone armed. Left unset it is captured from the first pose
        # seen, which is the arming pose when this node starts with the stack.
        self.declare_parameter("home_x", float("nan"))
        self.declare_parameter("home_y", float("nan"))
        # How stale a detection may be before it stops describing the pose that
        # produced it. The detector runs at camera rate, so this is generous.
        self.declare_parameter("detection_max_age_s", 1.0)
        self.declare_parameter("settle_tolerance_m", 0.10)
        self.declare_parameter("settle_updates", 3)
        self.declare_parameter("association_gate_m", 1.5)
        self.declare_parameter("max_correction_m", 2.0)
        # Steering, not observing. See the module docstring.
        self.declare_parameter("apply", False)

        p = lambda n: self.get_parameter(n).value

        self.tracker = LandmarkTracker(
            settle_tolerance_m=float(p("settle_tolerance_m")),
            settle_updates=int(p("settle_updates")))
        self.gate_m = float(p("association_gate_m"))
        self.max_correction_m = float(p("max_correction_m"))
        self.detection_max_age_s = float(p("detection_max_age_s"))
        self.apply = bool(p("apply"))

        hx, hy = float(p("home_x")), float(p("home_y"))
        self.home = None if (math.isnan(hx) or math.isnan(hy)) else (hx, hy)

        self.pads = []
        self.settled = set()
        self.pose = None
        self.n_anchor = 0
        self.n_reobs = 0

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, p("out_correction"), 10)
        self.create_subscription(PadMap, p("in_map"), self._cb_map, 10)
        self.create_subscription(PadDetection, p("in_detection"),
                                 self._cb_detection, 20)
        # MAVROS publishes local_position/pose BEST_EFFORT. A default
        # (RELIABLE) subscription does not merely drop messages — it never
        # MATCHES the publisher, so this node would sit silent forever with
        # only a QoS warning to say why.
        self.create_subscription(
            PoseStamped, p("in_pose"), self._cb_pose,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        if self.apply:
            self.get_logger().warn(
                "apply:=true — this node is STEERING the estimate, not "
                "observing it. Nothing has measured that its correction "
                "tracks the true error.")
        self.get_logger().info(
            f"landmark correction ready (observer) -> {p('out_correction')}")

    # ── inputs ──────────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg
        if self.home is None:
            # First pose seen is the arming pose: the one position in this
            # whole problem that no drift has touched.
            self.home = (msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info(
                f"home anchored at ({self.home[0]:.2f}, {self.home[1]:.2f}) "
                f"from the first pose")

    def _cb_map(self, msg: PadMap):
        self.pads = list(msg.pads)
        self.settled = self.tracker.update(self.pads)
        self._publish_anchor()

    def _cb_detection(self, msg: PadDetection):
        """A re-observation, if it can be tied to a settled landmark."""
        if not msg.position_valid or self.pose is None:
            return
        age = self._age_s(msg.header)
        if age is None or age > self.detection_max_age_s:
            return

        observed = (msg.position.x, msg.position.y)
        eligible = {int(pad.id) for pad in self.pads
                    if int(pad.id) in self.settled and is_trustworthy(pad)}
        pad = associate(self.pads, observed, gate_m=self.gate_m,
                        eligible=eligible)
        if pad is None:
            return

        d = drift_from_observations(
            [((pad.position.x, pad.position.y), observed, weight_of(pad))],
            max_correction_m=self.max_correction_m)
        if d is None:
            return
        self.n_reobs += 1
        self.get_logger().info(
            f"re-observed pad {int(pad.id)}: map says "
            f"({pad.position.x:.2f}, {pad.position.y:.2f}), detector says "
            f"({observed[0]:.2f}, {observed[1]:.2f}) — drift "
            f"{math.hypot(*d):.2f} m [{self.n_reobs}]",
            throttle_duration_sec=5.0)
        self._publish(d, source="reobservation", pad_id=int(pad.id))

    # ── outputs ─────────────────────────────────────────────────────────────

    def _publish_anchor(self):
        if self.home is None:
            return
        d = anchor_drift(self.pads, self.home,
                         max_correction_m=self.max_correction_m)
        if d is None:
            return
        self.n_anchor += 1
        self.get_logger().info(
            f"anchor: the takeoff base has walked {math.hypot(*d):.2f} m "
            f"from where the drone armed [{self.n_anchor}]",
            throttle_duration_sec=10.0)
        self._publish(d, source="anchor", pad_id=None)

    def _publish(self, d, *, source, pad_id):
        """The correction to APPLY to the pose: the negative of the drift.

        drift is where the world appears to have moved to; the pose moved the
        other way by the same amount.
        """
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = (self.pose.header.frame_id
                               if self.pose is not None else "map")
        msg.pose.pose.position.x = -d[0]
        msg.pose.pose.position.y = -d[1]
        msg.pose.pose.orientation.w = 1.0      # translation only, on purpose
        # Not a real covariance — a marker of which measurement this was, so a
        # consumer and a log can tell the anchor from a re-observation without
        # a second topic. The anchor is the stronger evidence and says so.
        var = 0.05 if source == "anchor" else 0.25
        msg.pose.covariance[0] = var
        msg.pose.covariance[7] = var
        self.pub.publish(msg)

    def _age_s(self, header):
        t = header.stamp.sec + header.stamp.nanosec * 1e-9
        if t == 0.0:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - t


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkCorrectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
