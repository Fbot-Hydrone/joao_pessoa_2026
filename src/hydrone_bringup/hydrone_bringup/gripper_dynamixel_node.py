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

OVERLOAD: an AX-12A that sits stalled at its Torque Limit raises the Overload
alarm, and its alarm shutdown ZEROES the Torque Limit -- the grip goes limp
and the next open cannot move (bench, 2026-11-17: closed on a payload at load
600, then "opened" stuck at 607 with torque_limit 0). So a close that holds
backs its goal off to just past where it stopped (`hold_squeeze` ticks, inside
the compliance slope: a firm grip below the limit), and every goal rewrites
the Torque Limit first, which also recovers from a shutdown that did happen.
Moves run at `torque_limit`; only the hold gets `hold_torque_limit` (bench:
1023 while moving lost packets through the ArbotiX; squeeze 30 at 1023 held
at load ~830 for over a minute; squeeze 60 tripped Overload at once).

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
        self.hold_squeeze = p("hold_squeeze", 30)      # ticks past the stop
        # Moving at full torque browns the ArbotiX bridge out (packets lost
        # mid-move on the bench); only the HOLD gets the full limit.
        self.hold_torque_limit = p("hold_torque_limit", 1023)
        self.timeout_s = p("timeout_s", 3.0)

        # Imported here so the package builds and the sim never needs it.
        from dynamixel_sdk import PacketHandler, PortHandler
        self.port_h = PortHandler(self.port)
        self.pkt = PacketHandler(1.0)
        if not (self.port_h.openPort() and self.port_h.setBaudRate(self.baud)):
            raise RuntimeError(f"cannot open {self.port} at {self.baud}")
        # Opening the port toggles DTR, which RESETS an ArbotiX-M bridge. Stay
        # SILENT through its ~2 s bootloader window -- bytes arriving then keep
        # it in the bootloader and the firmware never starts -- and then wait
        # for the servo to answer before the setup writes below.
        time.sleep(3.0)
        deadline = time.monotonic() + 10.0
        while self.pkt.read2ByteTxRx(
                self.port_h, self.dxl_id, ADDR_PRESENT_POSITION)[1] != 0:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"AX-12A id {self.dxl_id} not answering on {self.port}")
            time.sleep(0.2)
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
    # Retried: through an ArbotiX-M bridge an occasional packet goes missing
    # (its parser falls out of step when polled hard), and an unchecked
    # failure reads as position 0 or a goal that was never sent. Returns None
    # when the servo never answered -- never a made-up 0.
    def _txrx(self, fn, *args):
        for _ in range(3):
            out = fn(self.port_h, self.dxl_id, *args)
            if out[-2 if len(out) == 3 else 0] == 0:   # COMM_SUCCESS
                return out
            self.port_h.clearPort()
            time.sleep(0.05)
        self.get_logger().warn(
            f"AX-12A id {self.dxl_id}: no answer after 3 tries",
            throttle_duration_sec=2.0)
        return None

    def _w1(self, addr, v):
        self._txrx(self.pkt.write1ByteTxRx, addr, int(v))

    def _w2(self, addr, v):
        self._txrx(self.pkt.write2ByteTxRx, addr, int(v))

    def _r1(self, addr):
        out = self._txrx(self.pkt.read1ByteTxRx, addr)
        return None if out is None else out[0]

    def _r2(self, addr):
        out = self._txrx(self.pkt.read2ByteTxRx, addr)
        return None if out is None else out[0]

    def _arm(self, limit=None):
        """Torque Limit + Torque Enable: undoes an overload shutdown."""
        self._w2(ADDR_TORQUE_LIMIT,
                 self.torque_limit if limit is None else limit)
        self._w1(ADDR_TORQUE_ENABLE, 1)

    def _load(self):
        v = self._r2(ADDR_PRESENT_LOAD)
        return None if v is None else v & 0x3FF   # magnitude, drop direction

    # ── action ──────────────────────────────────────────────────────────────
    def _exec(self, gh):
        close = bool(gh.request.close)
        target = self.closed_pos if close else self.open_pos
        self._arm()
        self._w2(ADDR_GOAL_POSITION, target)
        t0 = time.monotonic()
        time.sleep(0.1)                       # let Moving rise
        span = max(abs(self.closed_pos - self.open_pos), 1)
        last, still = None, 0
        while self._r1(ADDR_MOVING) and time.monotonic() - t0 < self.timeout_s:
            pos = self._r2(ADDR_PRESENT_POSITION)
            if pos is not None:
                gh.publish_feedback(Gripper.Feedback(
                    position=abs(pos - self.open_pos) / span))
                # Stalled on the payload: stop polling at the torque limit,
                # every extra moment there is a step toward Overload.
                still = still + 1 if last is not None and \
                    abs(pos - last) <= 2 else 0
                last = pos
                if close and still >= 2:
                    break
            time.sleep(0.2)   # an ArbotiX bridge drops packets polled faster
        pos, load = self._r2(ADDR_PRESENT_POSITION), self._load()
        res = Gripper.Result(success=True)
        if pos is None or load is None:
            res.success = False
            res.message = (f"{'close' if close else 'open'}: AX-12A id "
                           f"{self.dxl_id} not answering — state unknown")
        elif close:
            stopped_short = abs(pos - self.closed_pos) > self.reach_tol
            res.holding = stopped_short and load >= self.holding_load
            res.message = (f"closed at {pos} (target {target}), load {load} — "
                           + ("HOLDING" if res.holding else "EMPTY"))
            if stopped_short:
                # Don't sit stalled at the limit (Overload -> limp grip).
                sign = 1 if self.closed_pos > self.open_pos else -1
                lo, hi = sorted((self.open_pos, self.closed_pos))
                hold = min(max(pos + sign * self.hold_squeeze, lo), hi)
                self._w2(ADDR_GOAL_POSITION, hold)
                self._arm(self.hold_torque_limit)
                time.sleep(1.0)
                out = self._txrx(self.pkt.read2ByteTxRx, ADDR_PRESENT_LOAD)
                if out is not None:
                    res.message += (f"; hold load {out[0] & 0x3FF}"
                                    + (" OVERLOAD" if out[2] & 0x20 else ""))
        else:
            reached = abs(pos - self.open_pos) <= self.reach_tol
            res.success = reached
            res.message = (f"opened at {pos} (target {target})"
                           + ("" if reached else " — STUCK, did not open"))
        self.get_logger().info(res.message)
        if res.success:
            gh.succeed()
        else:
            gh.abort()
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
