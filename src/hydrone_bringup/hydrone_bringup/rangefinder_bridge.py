#!/usr/bin/env python3
"""
rangefinder_bridge — BiguaSim down-facing RangeFinderSensor -> ArduPilot rangefinder.

BiguaSim publishes the RangeFinderSensor as a sensor_msgs/LaserScan. ArduPilot's
MAVLink rangefinder backend (RNGFND1_TYPE=10) expects DISTANCE_SENSOR messages,
which the MAVROS distance_sensor plugin emits from a sensor_msgs/Range topic. This
node is the small adapter in between:

  /biguasim/<agent>/RangeFinderSensor (LaserScan)
      --> this node -->
  /mavros/rangefinder (Range)  --(MAVROS distance_sensor, subscriber mode)-->
  DISTANCE_SENSOR --> ArduPilot RNGFND1

  (This MAVROS build's distance_sensor subscriber listens on ~/<name> from the
  /mavros namespace, i.e. /mavros/rangefinder — not /mavros/distance_sensor/*.)

The SAME Range also goes out on /mavros/distance_sensor/rangefinder, which is
where MAVROS PUBLISHES it on the real drone (the VL53L1X, read natively over
I2C). Nothing in the sim needs that copy — MAVROS is fed by the topic above —
but the autonomy layer does: without it, pad_map_node has to be told a different
topic in sim than on the drone, and that was the last thing separating
landing_sites.launch.py's two worlds. So this node mimics MAVROS's real output
the way zed_mimic_node mimics the ZED's, and the autonomy reads ONE name
everywhere. On the real drone this node does not run at all and MAVROS itself
publishes that topic.

The rangefinder is a nadir altimeter for landing/flare only (see holybro_sitl.parm:
it is NOT a global EKF height source). We publish the single downward beam; if the
sim sensor is configured with LaserCount>1 we take the nearest finite return.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Range


class RangefinderBridge(Node):

    def __init__(self):
        super().__init__("rangefinder_bridge")

        # in_scan and max_range are derived from biguasim's config.yaml by
        # sources_sim.launch.py (agent name and LaserMaxDistance); the defaults
        # here mirror it and only apply to a standalone `ros2 run`.
        self.declare_parameter("in_scan", "/biguasim/uav0_id0/RangeFinderSensor")
        # Into MAVROS (sim plumbing): the topic its distance_sensor subscriber
        # listens on, so the reading reaches ArduPilot.
        self.declare_parameter("out_range", "/mavros/rangefinder")
        # The contract copy: what MAVROS publishes on the drone. Consumers read
        # this one, in both worlds. Empty disables it.
        self.declare_parameter("out_range_mimic",
                               "/mavros/distance_sensor/rangefinder")
        self.declare_parameter("min_range", 0.20)   # m; matches RNGFND1_MIN_CM
        self.declare_parameter("max_range", 40.0)    # m; = LaserMaxDistance / RNGFND1_MAX_CM
        self.declare_parameter("field_of_view", 0.1) # rad; narrow single beam
        self.declare_parameter("frame_id", "rangefinder_link")

        p = lambda n: self.get_parameter(n).value
        self.min_range = float(p("min_range"))
        self.max_range = float(p("max_range"))
        self.fov = float(p("field_of_view"))
        self.frame_id = p("frame_id")

        self.pub = self.create_publisher(Range, p("out_range"), 10)
        mimic_topic = p("out_range_mimic")
        self.pub_mimic = self.create_publisher(Range, mimic_topic, 10) \
            if mimic_topic else None
        self.create_subscription(LaserScan, p("in_scan"), self._cb, 10)

        self.get_logger().info(
            f"Rangefinder bridge ready — {p('in_scan')} -> {p('out_range')}"
            + (f" (+ {mimic_topic}, mimicking real MAVROS)" if mimic_topic
               else ""))

    def _cb(self, scan: LaserScan):
        # Nearest finite return within the valid band (single beam for LaserCount=1).
        vals = [r for r in scan.ranges
                if math.isfinite(r) and self.min_range <= r <= self.max_range]
        if not vals:
            return  # nothing valid this frame (out of range / sky); skip
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED   # laser-like; ULTRASOUND also accepted
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = min(vals)
        self.pub.publish(msg)
        if self.pub_mimic is not None:
            self.pub_mimic.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangefinderBridge()
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
