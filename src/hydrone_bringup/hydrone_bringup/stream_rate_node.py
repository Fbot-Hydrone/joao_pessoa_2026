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

With `messages` (comma-separated MAVLink ids) it asks for only those, each at
message_rate, as MAV_CMD_SET_MESSAGE_INTERVAL through /mavros/cmd/command
instead (MAVROS's own set_message_interval times out on Humble: it calls its
command service from inside a service callback): everything else the
FCU would stream is bytes MAVROS decodes for nobody. On each connection it
first turns "all" off once (a previous run may have asked for it); it never
re-sends "all", because ArduPilot resets the per-message intervals when it gets
a stream request.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, StreamRate


class StreamRateNode(Node):

    def __init__(self):
        super().__init__('stream_rate')
        p = self.declare_parameters('', [
            ('message_rate', 20),       # Hz, every stream
            ('repeat_s', 10.0),
            ('messages', ''),           # e.g. '1,30,32': only these, '' = every stream
        ])
        self.rate, self.repeat = int(p[0].value), float(p[1].value)
        self.messages = [int(v) for v in str(p[2].value).replace(' ', '').split(',') if v]
        self.connected = False
        self.fresh = False              # connected, "all" not yet turned off
        self.busy = False               # a round of intervals is in flight
        self.cli = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.icli = self.create_client(CommandLong, '/mavros/cmd/command')
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_timer(self.repeat, self._ask)

    def _state(self, m):
        if m.connected and not self.connected:
            self.get_logger().info('FCU connected: requesting the streams')
            self.connected = True
            self.fresh = True
            self._ask()
        self.connected = m.connected

    def _ask(self):
        if not self.connected or not self.cli.service_is_ready():
            return
        if not self.messages:
            fut = self.cli.call_async(StreamRate.Request(stream_id=0, message_rate=self.rate, on_off=True))
            fut.add_done_callback(self._done)
        elif not self.icli.service_is_ready():
            return
        elif self.fresh:
            self.fresh = False
            fut = self.cli.call_async(StreamRate.Request(stream_id=0, message_rate=0, on_off=False))
            fut.add_done_callback(lambda f: (self._done(f), self._intervals()))
        else:
            self._intervals()

    def _intervals(self, queue=None):
        # One at a time: they are all the same MAVLink command, and MAVROS
        # matches the FCU's ACKs by command, so parallel ones time out.
        if queue is None:
            if self.busy:
                return
            self.busy, queue = True, list(self.messages)
        if not queue:
            self.busy = False
            return
        mid = queue.pop(0)
        fut = self.icli.call_async(CommandLong.Request(
            command=511, param1=float(mid), param2=1e6 / self.rate))  # MAV_CMD_SET_MESSAGE_INTERVAL, us
        fut.add_done_callback(lambda f: (self._done(f, mid), self._intervals(queue)))

    def _done(self, fut, mid=None):
        r = fut.result()
        if r is None:
            self.get_logger().warn(f'stream request failed: {fut.exception()}')
        elif getattr(r, 'result', 0) != 0:   # a refusal; a timeout (result 0) is retried next round
            self.get_logger().warn(f'FCU refused the interval for MAVLink message {mid}'
                                   f' (result {getattr(r, "result", "?")})')


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
