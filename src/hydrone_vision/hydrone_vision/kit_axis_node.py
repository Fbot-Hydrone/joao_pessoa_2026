"""kit_axis_node — the angle of every kit the detector finds (kit_axis.py).

Pairs each PadDetection with the image it came from (same stamp) and
publishes a KitAxis per detection. Works on any camera + any detector that
publishes PadDetection, so it runs off a webcam as well as the drone:

    ros2 run hydrone_vision kit_axis_node --ros-args \
        -p image_topic:=/image_raw -p detections_topic:=/hydrone/kits/detections

Debug image (publish_debug): the frame with each axis drawn, yellow when
coherent enough to turn for, grey when not.
"""

from collections import OrderedDict

import rclpy
from cv_bridge import CvBridge
from hydrone_msgs.msg import KitAxis, PadDetection
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hydrone_vision.kit_axis import draw, kit_axis


class KitAxisNode(Node):
    def __init__(self):
        super().__init__("kit_axis")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.min_coh = p("min_coherence", 0.3)
        self.debug = p("publish_debug", True)
        self.bridge = CvBridge()
        self.frames = OrderedDict()      # stamp -> Image, the last few
        self.keep = p("keep_frames", 15)
        self.pub = self.create_publisher(
            KitAxis, p("out_topic", "/hydrone/kits/axis"), 10)
        self.pub_dbg = (self.create_publisher(
            Image, p("debug_topic", "/hydrone/kits/axis_debug"), 1)
            if self.debug else None)
        self.create_subscription(
            Image, p("image_topic", "/down_cam/image_raw"), self._cb_image,
            qos_profile_sensor_data)
        self.create_subscription(
            PadDetection, p("detections_topic", "/hydrone/kits/detections"),
            self._cb_det, 20)

    @staticmethod
    def _key(header):
        return (header.stamp.sec, header.stamp.nanosec)

    def _cb_image(self, msg):
        self.frames[self._key(msg.header)] = msg
        while len(self.frames) > self.keep:
            self.frames.popitem(last=False)

    def _cb_det(self, det):
        img = self.frames.get(self._key(det.header))
        if img is None:
            return        # the frame already left the buffer: YOLO too slow
        bgr = self.bridge.imgmsg_to_cv2(img, "bgr8")
        r = kit_axis(bgr, det.u, det.v, det.radius_px)
        if r is None:
            return
        out = KitAxis(header=det.header, u=det.u, v=det.v)
        out.angle, out.coherence = float(r[0]), float(r[1])
        self.pub.publish(out)
        if self.pub_dbg is not None:
            ok = r[1] >= self.min_coh
            draw(bgr, det.u, det.v, r[0], 1.5 * det.radius_px,
                 (0, 255, 255) if ok else (128, 128, 128))
            dbg = self.bridge.cv2_to_imgmsg(bgr, "bgr8")
            dbg.header = det.header
            self.pub_dbg.publish(dbg)


def main():
    rclpy.init()
    node = KitAxisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
