#!/usr/bin/env python3
"""
gripper_dynamixel_node — the payload gripper on the REAL drone (Dynamixel AX-12A).

Serves the same action the simulator does, so a mission cannot tell them apart:

  /hydrone/gripper/command   hydrone_msgs/action/Gripper   close / open

AX-12A facts this is built on (Protocol 1.0, e-Manual):
  * half-duplex TTL serial — needs an adapter (U2D2 or equivalent) on the
    Jetson; default 1 Mbps, ID 1
  * Goal Position (addr 30, 2 bytes): 0..1023 over 0..300 deg
  * Torque Enable (24, 1), Torque Limit (34, 2, 0..1023), Moving (46, 1),
    Present Position (36, 2), Present Load (40, 2; bit 10 = direction)
  * 9-12 V supply

HOLDING, the thing the simulator cannot report: a close that STOPS short of
`closed_position` with load above `holding_load` has something between the
fingers. A close that reaches `closed_position` closed on nothing.

All numbers are parameters with PROVISIONAL defaults — calibrate them on the
hardware (open/closed positions especially) before the first flight.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from hydrone_msgs.action import Gripper

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40
ADDR_MOVING = 46


class GripperDynamixelNode(Node):

    def __init__(self):
        super().__init__("gripper_dynamixel")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.port = p("port", "/dev/ttyUSB0")
        self.baud = p("baudrate", 1000000)
        self.dxl_id = p("id", 1)
        self.open_pos = p("open_position", 512)        # PROVISIONAL
        self.closed_pos = p("closed_position", 800)    # PROVISIONAL
        self.torque_limit = p("torque_limit", 600)     # of 1023; spare the kit
        self.speed = p("moving_speed", 200)            # of 1023
        self.reach_tol = p("reach_tolerance", 20)      # ticks
        self.holding_load = p("holding_load", 150)     # of 1023
        self.timeout_s = p("timeout_s", 3.0)

        # Imported here so the package builds and the sim never needs it.
        from dynamixel_sdk import PacketHandler, PortHandler
        self.port_h = PortHandler(self.port)
        self.pkt = PacketHandler(1.0)
        if not (self.port_h.openPort() and self.port_h.setBaudRate(self.baud)):
            raise RuntimeError(f"cannot open {self.port} at {self.baud}")
        self._w2(ADDR_TORQUE_LIMIT, self.torque_limit)
        self._w2(ADDR_MOVING_SPEED, self.speed)
        self._w1(ADDR_TORQUE_ENABLE, 1)

        ActionServer(self, Gripper, "/hydrone/gripper/command", self._exec,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.REJECT)
        self.get_logger().info(
            f"AX-12A id {self.dxl_id} on {self.port} @ {self.baud} — "
            f"open {self.open_pos}, closed {self.closed_pos}")

    # ── raw register access ─────────────────────────────────────────────────
    def _w1(self, addr, v):
        self.pkt.write1ByteTxRx(self.port_h, self.dxl_id, addr, int(v))

    def _w2(self, addr, v):
        self.pkt.write2ByteTxRx(self.port_h, self.dxl_id, addr, int(v))

    def _r1(self, addr):
        return self.pkt.read1ByteTxRx(self.port_h, self.dxl_id, addr)[0]

    def _r2(self, addr):
        return self.pkt.read2ByteTxRx(self.port_h, self.dxl_id, addr)[0]

    def _load(self):
        return self._r2(ADDR_PRESENT_LOAD) & 0x3FF   # magnitude, drop direction

    # ── action ──────────────────────────────────────────────────────────────
    def _exec(self, gh):
        close = bool(gh.request.close)
        target = self.closed_pos if close else self.open_pos
        self._w2(ADDR_GOAL_POSITION, target)
        t0 = time.monotonic()
        time.sleep(0.1)                       # let Moving rise
        while self._r1(ADDR_MOVING) and time.monotonic() - t0 < self.timeout_s:
            pos = self._r2(ADDR_PRESENT_POSITION)
            span = max(abs(self.closed_pos - self.open_pos), 1)
            gh.publish_feedback(Gripper.Feedback(
                position=abs(pos - self.open_pos) / span))
            time.sleep(0.05)
        pos, load = self._r2(ADDR_PRESENT_POSITION), self._load()
        res = Gripper.Result(success=True)
        if close:
            stopped_short = abs(pos - self.closed_pos) > self.reach_tol
            res.holding = stopped_short and load >= self.holding_load
            res.message = (f"closed at {pos} (target {target}), load {load} — "
                           + ("HOLDING" if res.holding else "EMPTY"))
        else:
            res.message = f"opened at {pos}"
        self.get_logger().info(res.message)
        gh.succeed()
        return res


def main(args=None):
    rclpy.init(args=args)
    node = GripperDynamixelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
