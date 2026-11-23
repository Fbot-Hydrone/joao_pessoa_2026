"""ROS 2 wrapper: sets `color` at start, then follows /hydrone/leds/color.

Params:
  color     start color (default white), any `leds` CLI spelling
  backend   sysfs (the Jetson) or fake (no hardware)
  gpios     sysfs numbers for red, green, blue (default Nano pins 29, 31, 33)
  off_on_exit  switch the strip off when the node stops (default true)

    ros2 topic pub --once /hydrone/leds/color std_msgs/String "{data: 'red blue'}"
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from hydrone_leds.core import JETSON_NANO_GPIOS, RgbLeds, color_name, make_backend


class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        color = self.declare_parameter('color', 'white').value
        backend = self.declare_parameter('backend', 'sysfs').value
        gpios = self.declare_parameter('gpios', list(JETSON_NANO_GPIOS)).value
        self.off_on_exit = self.declare_parameter('off_on_exit', True).value

        self.leds = RgbLeds(make_backend(backend, gpios))
        self.pub = self.create_publisher(String, '/hydrone/leds/state', 1)
        self.create_subscription(String, '/hydrone/leds/color', self._on_color, 1)
        self._apply(color)

    def _apply(self, color):
        try:
            state = self.leds.set(color)
        except (ValueError, OSError) as e:
            self.get_logger().error('leds %r: %s' % (color, e))
            return
        self.get_logger().info('leds: %s' % color_name(state))
        self.pub.publish(String(data=color_name(state)))

    def _on_color(self, msg):
        self._apply(msg.data)

    def shutdown(self):
        if self.off_on_exit:
            try:
                self.leds.off()
            except OSError as e:
                self.get_logger().error('leds off: %s' % e)


def main():
    rclpy.init()
    node = LedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
