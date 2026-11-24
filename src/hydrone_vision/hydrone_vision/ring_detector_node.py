"""ring_detector_node — publishes the belly camera's ring fit as PadDetection.

Fallback base detector for the Phase 2 delivery descent (see ring_detector.py
for why). Same message and topic family as pad_detector_node, so the mission
reads it with the same `Frames` helper; one message per frame with a ring.
"""

import math

import rclpy
from cv_bridge import CvBridge
from hydrone_msgs.msg import PadDetection
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String

from hydrone_vision.ring_detector import fit_ring


class RingDetectorNode(Node):
    def __init__(self):
        super().__init__("ring_detector")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.ring_k = p("ring_k", 150.0)
        self.k_tol = p("k_tol", 0.25)
        self.min_conf = p("min_confidence", 0.5)
        self.range_m = None
        # Only while a kit is carried: the fit costs CPU the kit YOLO needs on
        # the pickup descent.
        self.carrying = False
        self.create_subscription(
            String, "/hydrone/mission/status",
            lambda m: setattr(self, "carrying", "carrying=True" in m.data), 10)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            PadDetection, p("out_topic", "/hydrone/pads/down/ring"), 10)
        self.create_subscription(
            Range, p("range_topic", "/mavros/distance_sensor/rangefinder"),
            self._cb_range, qos_profile_sensor_data)
        self.create_subscription(
            Image, p("image_topic", "/down_cam/image_raw"), self._cb_image,
            qos_profile_sensor_data)

    def _cb_range(self, m):
        self.range_m = float(m.range) if math.isfinite(m.range) else None

    def _cb_image(self, msg):
        if not self.carrying:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ring = fit_ring(bgr, self.range_m, ring_k=self.ring_k, k_tol=self.k_tol)
        if ring is None or ring[3] < self.min_conf:
            return
        out = PadDetection(header=msg.header, camera="down")
        out.u, out.v, out.radius_px, out.confidence = ring
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(RingDetectorNode())
