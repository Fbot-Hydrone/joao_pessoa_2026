#!/usr/bin/env python3
"""
stream_rate_node — ask the REAL ArduPilot for its telemetry, and ask again.

ArduPilot 4.7 removed the SRx_ parameters: a Pixhawk on 4.7 sends a companion
computer nothing but HEARTBEAT until somebody asks. Measured on the CubeBlack
(ArduCopter 4.7.1): MAVROS connects, /mavros/state says connected, and
/mavros/imu/data, /mavros/local_position/pose and /mavros/home_position/home
never publish. Everything above waits on those — mavros_glue does not start
without a home, phase3_hri_node flies on the local pose — so without this node
the mission sits there forever without an error.

SITL in BiguaSim does not need it (its defaults stream), which is why the sim
launches never had it.

  in   /mavros/state
  call /mavros/set_stream_rate (stream_id 0 = all, message_rate Hz), on every
       connection, again every `repeat_s` while connected: an FCU reboot or a
       USB replug resets the rates and MAVROS reconnects without telling us
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate


class StreamRateNode(Node):

    def __init__(self):
        super().__init__('stream_rate')
        p = self.declare_parameters('', [
            ('message_rate', 20),       # Hz, every stream
            ('repeat_s', 10.0),
        ])
        self.rate, self.repeat = int(p[0].value), float(p[1].value)
        self.connected = False
        self.cli = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_timer(self.repeat, self._ask)

    def _state(self, m):
        if m.connected and not self.connected:
            self.get_logger().info('FCU connected: requesting the streams')
            self.connected = True
            self._ask()
        self.connected = m.connected

    def _ask(self):
        if not self.connected or not self.cli.service_is_ready():
            return
        fut = self.cli.call_async(StreamRate.Request(stream_id=0, message_rate=self.rate, on_off=True))
        fut.add_done_callback(self._done)

    def _done(self, fut):
        if fut.result() is None:
            self.get_logger().warn(f'set_stream_rate failed: {fut.exception()}')


def main():
    rclpy.init()
    node = StreamRateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
