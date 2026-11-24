#!/usr/bin/env python3
"""
gripper_dynamixel_node — the payload gripper on the REAL drone (Dynamixel AX-12A).

Interfaces:
  /hydrone/gripper/command     hydrone_msgs/action/Gripper   close / open (mission)
  /hydrone/gripper/grab        std_srvs/srv/Trigger          close + dynamic torque ramp -> hold
  /hydrone/gripper/release     std_srvs/srv/Trigger          open & release object
  /hydrone/gripper/calibrate   std_srvs/srv/Trigger          dynamic torque-limited homing
  /hydrone/gripper/state       sensor_msgs/msg/JointState    live position, speed, load

AX-12A facts this is built on (Protocol 1.0, via ArbotiX-M `ros.ino` bridge):
  * FTDI USB (115200 baud) -> ArbotiX-M -> 1 Mbps half-duplex TTL bus, ID 1.
  * Opening the FTDI port toggles DTR and resets the ArbotiX-M; we stay silent
    for `boot_wait_s` (7.5 s default, covering both ~2 s and ~6.5 s bootloaders)
    and probe with a register read (ArbotiX `ros.ino` ignores PING).
  * Registers 36..46 (Present Position, Speed, Load, Voltage, Temp, Moving) are
    read in ONE atomic 11-byte packet, cutting bus traffic by 3x and giving live
    torque feedback (`Present Load`, bits 0..9) + hardware error byte (`0x20` =
    Overload) on every poll.

3D-PRINTED GRIPPER — DYNAMIC TORQUE RAMP & MOVEMENT/STIFFNESS ANALYSIS:
  * Closing starts at a gentle `close_start_torque` (350). When resistance slows
    the jaws short of `closed_position`, we do NOT immediately trust it as a
    grasp (3D-print friction or a partially-seated object can fool a static check).
  * Instead, we step torque up by `torque_ramp_step` (80) up to a 3D-print-safe
    ceiling `close_max_torque` (680) and measure `(moved_ticks, delta_load)`:
      - If jaws KEEP MOVING without significant torque increase (`moved >
        firm_move_tol` and `dload < min_torque_rise`), the object is NOT firmly
        grabbed -> resume closing at the higher torque.
      - If jaws STOP (`moved <= firm_move_tol`) AND load rises cleanly with the
        added torque (`dload >= min_torque_rise`, or solid load at ceiling),
        the object IS firmly gripped -> latch safe compliance-slope hold capped
        at `hold_torque_limit` (700) with active relief above `max_hold_load` (650).
"""

from dataclasses import dataclass
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from hydrone_msgs.action import Gripper

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36   # 36..46 = 11 bytes contiguous telemetry block
ADDR_PRESENT_LOAD = 40
ADDR_MOVING = 46

ERR_OVERLOAD = 0x20
SAFE_MIN_TICK = 15
SAFE_MAX_TICK = 1008


@dataclass
class ServoState:
    pos: int
    speed: int
    load: int
    voltage: float
    temp: int
    moving: bool
    hw_err: int


def clamp_tick(val: int, lo: int = SAFE_MIN_TICK, hi: int = SAFE_MAX_TICK) -> int:
    return max(lo, min(hi, int(val)))


def parse_telemetry(data, hw_err: int) -> ServoState:
    """Decode an 11-byte block starting at ADDR_PRESENT_POSITION (36..46)."""
    pos = data[0] | (data[1] << 8)
    spd_raw = data[2] | (data[3] << 8)
    load_raw = data[4] | (data[5] << 8)
    return ServoState(
        pos=pos,
        speed=spd_raw & 0x3FF,
        load=load_raw & 0x3FF,
        voltage=data[6] / 10.0,
        temp=data[7],
        moving=bool(data[10]),
        hw_err=hw_err,
    )


class GripperDynamixelNode(Node):

    def __init__(self, port_h=None, pkt=None):
        super().__init__("gripper_dynamixel")
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.port = p("port", "/dev/ttyUSB0")
        self.baud = int(p("baudrate", 115200))
        self.dxl_id = int(p("id", 1))
        self.boot_wait_s = float(p("boot_wait_s", 7.5))

        # Initial/fallback mechanical limits (refined dynamically if auto_calibrate=True)
        self.open_pos = int(p("open_position", 264))
        self.closed_pos = int(p("closed_position", 755))

        # Moving & holding torque / speed parameters (3D-print-safe defaults)
        self.torque_limit = int(p("torque_limit", 550))             # open / general move torque
        self.close_start_torque = int(p("close_start_torque", 350)) # initial gentle close torque
        self.torque_ramp_step = int(p("torque_ramp_step", 80))      # dynamic torque escalation step
        self.close_max_torque = int(p("close_max_torque", 720))     # hard 3D-print closing ceiling
        self.hold_torque_limit = int(p("hold_torque_limit", 780))   # hard 3D-print holding ceiling
        self.hold_torque_margin = int(p("hold_torque_margin", 110)) # margin above confirmed ramp torque
        self.speed = int(p("moving_speed", 200))                    # 0..1023
        self.reach_tol = int(p("reach_tolerance", 20))              # ticks

        # Dynamic torque sensing, movement/stiffness verification & stall thresholds
        self.holding_load = int(p("holding_load", 180))
        self.close_torque_threshold = int(p("close_torque_threshold", 260))
        self.open_torque_threshold = int(p("open_torque_threshold", 350))
        self.hard_torque_threshold = int(p("hard_torque_threshold", 560))
        self.max_hold_load = int(p("max_hold_load", 730))
        self.torque_samples = int(p("torque_samples", 2))
        self.slow_delta = int(p("slow_delta", 8))              # ticks/poll; above = moving freely
        self.stall_delta = int(p("stall_delta", 3))
        self.stall_samples = int(p("stall_samples", 2))
        self.inrush_ignore_s = float(p("inrush_ignore_s", 0.30))
        self.poll_interval_s = float(p("poll_interval_s", 0.18))

        # Movement-vs-torque grasp confirmation parameters
        self.firm_move_tol = int(p("firm_move_tol", 8))        # max ticks moved on torque step for firm grip
        self.min_torque_rise = int(p("min_torque_rise", 40))   # min load rise confirming solid object
        self.ramp_step_s = float(p("ramp_step_s", 0.30))       # probe window per torque increment
        self.max_ramp_cycles = int(p("max_ramp_cycles", 8))

        # Post-contact backoff / squeeze parameters
        self.hold_squeeze = int(p("hold_squeeze", 24))         # ticks past object contact
        self.relief_step = int(p("relief_step", 8))            # ticks eased on high hold load
        self.open_backoff = int(p("open_backoff", 10))         # ticks off open hard stop
        self.timeout_s = float(p("timeout_s", 5.0))

        # Dynamic startup homing / calibration parameters
        self.auto_calibrate = bool(p("auto_calibrate", True))
        self.calib_torque_limit = int(p("calib_torque_limit", 400))
        self.calib_speed = int(p("calib_speed", 150))
        self.calib_load_threshold = int(p("calib_load_threshold", 280))
        self.calib_stall_samples = int(p("calib_stall_samples", 3))
        self.calib_inrush_s = float(p("calib_inrush_s", 0.40))
        self.calib_timeout_s = float(p("calib_timeout_s", 6.0))
        self.limit_margin = int(p("limit_margin", 10))
        self.min_calib_travel = int(p("min_calib_travel", 100))

        self.pub_state = self.create_publisher(JointState, "/hydrone/gripper/state", 10)

        if port_h is not None and pkt is not None:
            # Dependency injection hook for unit tests
            self.port_h, self.pkt = port_h, pkt
        else:
            # Imported here so the package builds and the sim never needs it.
            from dynamixel_sdk import PacketHandler, PortHandler
            self.port_h = PortHandler(self.port)
            self.pkt = PacketHandler(1.0)
            if not (self.port_h.openPort() and self.port_h.setBaudRate(self.baud)):
                raise RuntimeError(f"cannot open {self.port} at {self.baud}")
            # Opening the port toggles DTR, resetting the ArbotiX-M. Stay silent
            # through the bootloader window (up to ~6.5 s on older bootloaders).
            if self.boot_wait_s > 0:
                time.sleep(self.boot_wait_s)

        deadline = time.monotonic() + 10.0
        first_state = None
        while first_state is None:
            first_state = self._read_state(retries=1)
            if first_state is not None:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"AX-12A id {self.dxl_id} not answering on {self.port}")
            time.sleep(0.2)

        self._publish_state(first_state)
        if not self._arm(self.torque_limit, self.speed):
            raise RuntimeError(
                f"AX-12A id {self.dxl_id}: initial torque/speed setup failed")

        if self.auto_calibrate:
            ok, msg = self._calibrate_limits()
            if ok:
                self.get_logger().info(f"auto_calibrate: {msg}")
            else:
                self.get_logger().warn(
                    f"auto_calibrate fell back to defaults "
                    f"(open={self.open_pos}, closed={self.closed_pos}): {msg}")

        ActionServer(self, Gripper, "/hydrone/gripper/command", self._exec,
                     goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.REJECT)
        self.create_service(Trigger, "/hydrone/gripper/calibrate",
                            self._srv_calibrate)
        self.create_service(Trigger, "/hydrone/gripper/grab",
                            self._srv_grab)
        self.create_service(Trigger, "/hydrone/gripper/release",
                            self._srv_release)
        self.get_logger().info(
            f"AX-12A id {self.dxl_id} on {self.port} @ {self.baud} "
            f"(V={first_state.voltage:.1f}V, T={first_state.temp}C) — "
            f"open={self.open_pos}, closed={self.closed_pos}, "
            f"close_torque={self.close_start_torque}..{self.close_max_torque}, "
            f"hold_cap={self.hold_torque_limit}")

    # ── helpers & raw register access ───────────────────────────────────────
    @property
    def _close_sign(self) -> int:
        return 1 if self.closed_pos >= self.open_pos else -1

    def _publish_state(self, st: ServoState):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper"]
        msg.position = [float(st.pos)]
        msg.velocity = [float(st.speed)]
        msg.effort = [float(st.load)]
        self.pub_state.publish(msg)

    def _publish_feedback(self, gh, pos: int):
        if gh is None:
            return
        span = max(abs(self.closed_pos - self.open_pos), 1)
        norm = min(max(abs(pos - self.open_pos) / span, 0.0), 1.0)
        gh.publish_feedback(Gripper.Feedback(position=norm))

    # Retried: through an ArbotiX-M bridge an occasional packet goes missing,
    # and an unchecked failure reads as position 0 or drops a command.
    def _txrx(self, fn, *args, retries: int = 3):
        for _ in range(retries):
            out = fn(self.port_h, self.dxl_id, *args)
            comm_idx = -2 if len(out) == 3 else 0
            if out[comm_idx] == 0:   # COMM_SUCCESS
                return out
            self.port_h.clearPort()
            time.sleep(0.05)
        self.get_logger().warn(
            f"AX-12A id {self.dxl_id}: no answer after {retries} tries",
            throttle_duration_sec=2.0)
        return None

    def _w1(self, addr: int, v: int) -> bool:
        return self._txrx(self.pkt.write1ByteTxRx, addr, int(v)) is not None

    def _w2(self, addr: int, v: int) -> bool:
        return self._txrx(self.pkt.write2ByteTxRx, addr, int(v)) is not None

    def _read_state(self, retries: int = 3):
        """Atomic 11-byte read of registers 36..46 (pos, speed, load, V, T, moving)."""
        out = self._txrx(
            self.pkt.readTxRx, ADDR_PRESENT_POSITION, 11, retries=retries)
        if out is None:
            return None
        data, _, hw_err = out
        if len(data) != 11:
            return None
        return parse_telemetry(data, hw_err)

    def _arm(self, limit=None, speed=None) -> bool:
        """Rewrite Torque Limit (+ optional speed) & Torque Enable.

        Recovers from an AX-12A Overload alarm shutdown (which zeroes Torque Limit).
        """
        ok = self._w2(ADDR_TORQUE_LIMIT,
                      self.torque_limit if limit is None else limit)
        if speed is not None:
            ok = self._w2(ADDR_MOVING_SPEED, speed) and ok
        ok = self._w1(ADDR_TORQUE_ENABLE, 1) and ok
        return ok

    # ── torque-guarded motion primitive ─────────────────────────────────────
    def _move_with_torque_guard(
        self,
        target: int,
        *,
        torque_limit: int,
        speed: int,
        load_threshold: int,
        timeout_s: float,
        inrush_ignore_s: float = None,
        stall_samples: int = None,
        hard_torque_threshold: int = None,
        gh=None,
    ):
        """Command `target` while watching live load, motion delta, and Overload.

        To avoid false positives from normal linkage friction while moving,
        `load_threshold` requires BOTH high load AND slowed motion
        (`delta <= slow_delta`). `hard_torque_threshold` acts as an upper
        backstop regardless of speed.

        Returns (final_ServoState_or_None, stop_reason: str).
        stop_reason is one of:
          "reached", "torque", "stall", "overload", "timeout", "comm_error"
        """
        inrush_s = self.inrush_ignore_s if inrush_ignore_s is None else inrush_ignore_s
        min_stalls = self.stall_samples if stall_samples is None else stall_samples
        hard_thresh = (self.hard_torque_threshold if hard_torque_threshold is None
                       else hard_torque_threshold)

        target = clamp_tick(target)
        if not self._arm(torque_limit, speed):
            return None, "comm_error"
        if not self._w2(ADDR_GOAL_POSITION, target):
            return None, "comm_error"

        t0 = time.monotonic()
        time.sleep(0.1)   # allow motion & Moving flag to rise

        last_pos = None
        still_hits = 0
        torque_hits = 0
        hard_hits = 0
        consecutive_misses = 0
        last_state = None
        reason = "timeout"

        while time.monotonic() - t0 < timeout_s:
            st = self._read_state(retries=2)
            if st is None:
                consecutive_misses += 1
                if consecutive_misses >= 3:
                    return None, "comm_error"
                time.sleep(self.poll_interval_s)
                continue

            consecutive_misses = 0
            last_state = st
            self._publish_state(st)
            self._publish_feedback(gh, st.pos)

            if st.hw_err & ERR_OVERLOAD:
                reason = "overload"
                break

            elapsed = time.monotonic() - t0
            delta = abs(st.pos - last_pos) if last_pos is not None else None
            if elapsed >= inrush_s and delta is not None:
                slowed = delta <= self.slow_delta
                stalled = delta <= self.stall_delta

                if st.load >= load_threshold and slowed:
                    torque_hits += 1
                else:
                    torque_hits = 0

                if st.load >= hard_thresh:
                    hard_hits += 1
                else:
                    hard_hits = 0

                if stalled:
                    still_hits += 1
                else:
                    still_hits = 0

                if torque_hits >= self.torque_samples:
                    reason = "torque"
                    break
                if (st.load >= hard_thresh and slowed) or hard_hits >= 2:
                    reason = "torque"
                    break
                if still_hits >= min_stalls:
                    reason = "stall"
                    break
                if not st.moving:
                    reason = "reached"
                    break

            last_pos = st.pos
            time.sleep(self.poll_interval_s)

        final_st = self._read_state(retries=3) or last_state
        if final_st is not None:
            self._publish_state(final_st)
        return final_st, reason

    # ── dynamic torque-limited calibration ──────────────────────────────────
    def _calibrate_limits(self):
        """Seek open & closed stops at controlled torque regardless of start pose."""
        close_sign = self._close_sign
        open_target = SAFE_MIN_TICK if close_sign > 0 else SAFE_MAX_TICK
        closed_target = SAFE_MAX_TICK if close_sign > 0 else SAFE_MIN_TICK
        calib_hard = max(self.calib_load_threshold + 80,
                         int(self.calib_torque_limit * 0.92))

        # 1. Seek open stop
        st_open, r_open = self._move_with_torque_guard(
            open_target,
            torque_limit=self.calib_torque_limit,
            speed=self.calib_speed,
            load_threshold=self.calib_load_threshold,
            timeout_s=self.calib_timeout_s,
            inrush_ignore_s=self.calib_inrush_s,
            stall_samples=self.calib_stall_samples,
            hard_torque_threshold=calib_hard,
        )
        if st_open is None:
            return False, f"open-stop probe failed ({r_open})"
        new_open = clamp_tick(st_open.pos + close_sign * self.limit_margin)
        if r_open == "overload":
            self._arm(self.calib_torque_limit, self.calib_speed)
        self._w2(ADDR_GOAL_POSITION, new_open)
        time.sleep(0.35)

        # 2. Seek closed stop
        st_closed, r_closed = self._move_with_torque_guard(
            closed_target,
            torque_limit=self.calib_torque_limit,
            speed=self.calib_speed,
            load_threshold=self.calib_load_threshold,
            timeout_s=self.calib_timeout_s,
            inrush_ignore_s=self.calib_inrush_s,
            stall_samples=self.calib_stall_samples,
            hard_torque_threshold=calib_hard,
        )
        if st_closed is None:
            return False, f"closed-stop probe failed ({r_closed})"
        new_closed = clamp_tick(st_closed.pos - close_sign * self.limit_margin)
        if r_closed == "overload":
            self._arm(self.calib_torque_limit, self.calib_speed)

        travel = (new_closed - new_open) * close_sign
        if travel < self.min_calib_travel:
            # Likely obstructed or jammed; open back up and keep previous limits.
            self._w2(ADDR_GOAL_POSITION, new_open)
            self._arm(self.torque_limit, self.speed)
            return False, (
                f"measured travel {travel} ticks < min {self.min_calib_travel} "
                f"(raw open={st_open.pos} [{r_open}], closed={st_closed.pos} [{r_closed}]) "
                f"— jaws obstructed?"
            )

        self.open_pos = new_open
        self.closed_pos = new_closed

        # 3. Return cleanly to calibrated open_pos and restore normal speed/torque
        self._move_with_torque_guard(
            self.open_pos,
            torque_limit=self.torque_limit,
            speed=self.speed,
            load_threshold=self.open_torque_threshold,
            timeout_s=self.timeout_s,
        )
        return True, (
            f"open={self.open_pos} (raw {st_open.pos}, {r_open}), "
            f"closed={self.closed_pos} (raw {st_closed.pos}, {r_closed}), "
            f"travel={travel} ticks"
        )

    # ── core close-and-hold with dynamic torque ramp & movement analysis ────
    def _close_and_hold(self, gh=None):
        """Close while ramping torque dynamically and analyzing movement vs load.

        If raising torque causes the jaws to keep moving without a matching
        torque rise, the object isn't solidly grabbed yet — keep closing.
        Only confirm HOLDING once jaws stop moving (`moved <= firm_move_tol`)
        and load rises cleanly (or hits solid load at `close_max_torque`),
        all bounded strictly for a 3D-printed gripper.

        Returns (comm_ok: bool, holding: bool, message: str).
        """
        target = self.closed_pos
        close_sign = self._close_sign
        lo, hi = sorted((self.open_pos, self.closed_pos))
        deadline = time.monotonic() + self.timeout_s

        cur_torque = min(self.close_start_torque, self.close_max_torque)
        confirmed_holding = False
        st = None
        reason = "timeout"

        for cycle in range(self.max_ramp_cycles):
            rem = deadline - time.monotonic()
            if rem <= 0.15:
                break

            step_thresh = min(self.close_torque_threshold, int(cur_torque * 0.80))
            step_hard = min(self.hard_torque_threshold, int(cur_torque * 0.94))
            st, reason = self._move_with_torque_guard(
                target,
                torque_limit=cur_torque,
                speed=self.speed,
                load_threshold=step_thresh,
                timeout_s=rem,
                inrush_ignore_s=self.inrush_ignore_s if cycle == 0 else 0.15,
                hard_torque_threshold=step_hard,
                gh=gh,
            )
            if st is None:
                return False, False, (
                    f"close: AX-12A id {self.dxl_id} not answering ({reason})")

            # Reached full empty close position
            if abs(st.pos - self.closed_pos) <= self.reach_tol:
                confirmed_holding = False
                break

            # Overload already tripped while short of closed_pos -> solid contact
            if reason == "overload":
                confirmed_holding = True
                break

            # Stopped short of closed_pos: probe with a controlled torque step
            # and analyze movement vs. load change to distinguish a true grab
            # from 3D-print friction or an object that isn't fully seated yet.
            prev_pos, prev_load = st.pos, st.load
            next_torque = min(cur_torque + self.torque_ramp_step, self.close_max_torque)
            at_ceiling = (next_torque == cur_torque)

            if not self._arm(next_torque, self.speed):
                return False, False, (
                    f"close: AX-12A id {self.dxl_id} failed torque ramp write")
            if not self._w2(ADDR_GOAL_POSITION, clamp_tick(target)):
                return False, False, (
                    f"close: AX-12A id {self.dxl_id} failed goal write")

            time.sleep(self.ramp_step_s)
            probe_st = self._read_state(retries=2)
            if probe_st is None:
                return False, False, (
                    f"close: AX-12A id {self.dxl_id} lost comm during torque probe")
            self._publish_state(probe_st)
            self._publish_feedback(gh, probe_st.pos)

            moved = (probe_st.pos - prev_pos) * close_sign
            dload = probe_st.load - prev_load
            st = probe_st
            cur_torque = next_torque

            # Reached full empty close during the ramp probe
            if abs(st.pos - self.closed_pos) <= self.reach_tol:
                confirmed_holding = False
                reason = "reached"
                break

            if st.hw_err & ERR_OVERLOAD:
                confirmed_holding = True
                reason = "overload"
                break

            stopped_firmly = abs(moved) <= self.firm_move_tol
            torque_rose = dload >= self.min_torque_rise
            solid_at_ceiling = (
                at_ceiling and stopped_firmly and st.load >= self.close_torque_threshold
            )

            self.get_logger().info(
                f"grab ramp: torque={cur_torque} (cap {self.close_max_torque}) | "
                f"pos {prev_pos}->{st.pos} (moved={moved:+d}) | "
                f"load {prev_load}->{st.load} (dload={dload:+d})")

            if stopped_firmly and (torque_rose or solid_at_ceiling):
                # Jaws held still AND load climbed cleanly -> object solidly grabbed!
                confirmed_holding = True
                reason = "torque_confirmed"
                break

            if moved > self.firm_move_tol and dload < self.min_torque_rise:
                # Moving without much torque increase -> object not quite grabbed yet;
                # keep closing at the raised torque (or abort if already at ceiling).
                self.get_logger().info(
                    "grab ramp: jaws still advancing without torque rise — "
                    "continuing close")
                if at_ceiling and not st.moving:
                    break
                continue

            if at_ceiling:
                # Hit 3D-print torque ceiling while jaws were still creeping/low load
                confirmed_holding = (
                    stopped_firmly and st.load >= self.holding_load
                )
                reason = "torque_ceiling"
                break

        if st is None:
            return False, False, (
                f"close: AX-12A id {self.dxl_id} did not produce telemetry")

        pos, load = st.pos, st.load
        holding = confirmed_holding and (abs(pos - self.closed_pos) > self.reach_tol)
        message = (
            f"closed at {pos} (target {target}, stop={reason}, torque={cur_torque}), "
            f"load {load}" + (" — HOLDING" if holding else " — EMPTY"))

        if holding:
            # Tailored 3D-print-safe hold torque (capped at hold_torque_limit)
            hold_torque = min(cur_torque + self.hold_torque_margin,
                              self.hold_torque_limit)
            hold = clamp_tick(pos + close_sign * self.hold_squeeze, lo, hi)
            self._w2(ADDR_GOAL_POSITION, hold)
            self._arm(hold_torque)
            hold_st = None
            for _ in range(3):
                time.sleep(0.35)
                hold_st = self._read_state(retries=2)
                if hold_st is None:
                    break
                self._publish_state(hold_st)
                overloaded = bool(hold_st.hw_err & ERR_OVERLOAD)
                if not overloaded and hold_st.load <= self.max_hold_load:
                    break
                hold = clamp_tick(hold - close_sign * self.relief_step, lo, hi)
                self._w2(ADDR_GOAL_POSITION, hold)
                self._arm(hold_torque)
            if hold_st is not None:
                message += (
                    f"; hold={hold} torque={hold_torque} load={hold_st.load}"
                    + (" OVERLOAD" if hold_st.hw_err & ERR_OVERLOAD else ""))
        else:
            # Empty close or unconfirmed slip: relax goal so the 3D-printed jaws aren't forced.
            torque_stopped = reason in ("torque", "overload", "stall", "torque_ceiling")
            relax = (clamp_tick(pos - close_sign * 5, lo, hi)
                     if torque_stopped else pos)
            self._w2(ADDR_GOAL_POSITION, relax)

        return True, holding, message

    def _open_gripper(self, gh=None):
        """Open with live torque guard and backoff off mechanical stops.

        Returns (success: bool, message: str).
        """
        target = self.open_pos
        st, reason = self._move_with_torque_guard(
            target,
            torque_limit=self.torque_limit,
            speed=self.speed,
            load_threshold=self.open_torque_threshold,
            timeout_s=self.timeout_s,
            gh=gh,
        )
        if st is None:
            return False, (
                f"open: AX-12A id {self.dxl_id} not answering ({reason})")

        pos, load = st.pos, st.load
        close_sign = self._close_sign
        lo, hi = sorted((self.open_pos, self.closed_pos))
        span = max(abs(self.closed_pos - self.open_pos), 1)
        early_stop = reason in ("torque", "overload", "stall")
        if early_stop:
            # Never lean against the open mechanical stop
            backed = clamp_tick(pos + close_sign * self.open_backoff, lo, hi)
            self._w2(ADDR_GOAL_POSITION, backed)
            self._arm(self.torque_limit)
        elif load >= self.holding_load:
            self._w2(ADDR_GOAL_POSITION, pos)

        reached = abs(pos - self.open_pos) <= self.reach_tol
        near_open_stop = early_stop and (abs(pos - self.open_pos) <= 0.35 * span)
        ok = reached or near_open_stop
        message = (
            f"opened at {pos} (target {target}, stop={reason}), load {load}"
            + ("" if ok else " — STUCK, did not open"))
        return ok, message

    # ── ROS 2 Trigger services ──────────────────────────────────────────────
    def _srv_calibrate(self, _req, res):
        ok, msg = self._calibrate_limits()
        res.success = ok
        res.message = msg
        if ok:
            self.get_logger().info(f"calibrate service: {msg}")
        else:
            self.get_logger().warn(f"calibrate service failed: {msg}")
        return res

    def _srv_grab(self, _req, res):
        """Close with dynamic torque ramp & movement verification, then hold."""
        comm_ok, holding, msg = self._close_and_hold()
        res.success = bool(comm_ok and holding)
        res.message = msg
        self.get_logger().info(f"grab service: {msg}")
        return res

    def _srv_release(self, _req, res):
        """Open gripper and release any held object."""
        ok, msg = self._open_gripper()
        res.success = bool(ok)
        res.message = msg
        self.get_logger().info(f"release service: {msg}")
        return res

    # ── ROS 2 Action server ─────────────────────────────────────────────────
    def _exec(self, gh):
        close = bool(gh.request.close)
        res = Gripper.Result()
        if close:
            ok, holding, msg = self._close_and_hold(gh=gh)
            res.success = ok
            res.holding = holding
            res.message = msg
        else:
            ok, msg = self._open_gripper(gh=gh)
            res.success = ok
            res.holding = False
            res.message = msg

        self.get_logger().info(res.message)
        if res.success:
            gh.succeed()
        else:
            gh.abort()
        return res

    def destroy_node(self):
        try:
            if getattr(self, "port_h", None) is not None:
                self.port_h.closePort()
        except Exception:
            pass
        super().destroy_node()


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
