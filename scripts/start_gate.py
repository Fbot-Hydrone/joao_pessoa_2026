#!/usr/bin/env python3
"""start_gate — the only thing that calls a mission's start service on the mini.

Run inside the stack's container by nano_flight/run_phase3.sh and run_phase4.sh:

    start_gate.py --service /hydrone/phase4/start --ready-only     # wait for READY, exit 0
    start_gate.py --service /hydrone/phase4/start --countdown 10   # READY, countdown, start
      --status /hydrone/phase3/status --status-ready WAIT_START    # phase 3 also waits for its state
      --nogo                                                       # every check, no start call

READY: MAVROS connected, the FCU has a home, disarmed, the EKF pose and the LIO
odometry fresh, the start service up (and the mission status, if given), and
the drone still and level for 3 s. During the countdown and right before the
call, all of it again plus "has not moved": the stack started with the drone
on its spot (the LIO origin is that spot), so anything else is someone
carrying it. Any failure prints ABORT and the start is never called.

A LIO that latched "LIO LOST" publishes nothing more, so it shows up here as
stale odometry.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

STALE_S = 0.5
STILL_S, STILL_M, LEVEL_DEG = 3.0, 0.10, 10.0
MOVED_M = 0.15      # from where it was at READY


def say(s):
    print(f"[{time.strftime('%H:%M:%S')}] {s}", flush=True)


class Gate(Node):
    def __init__(self, a):
        super().__init__("start_gate")
        self.state = None
        self.home = False
        self.status = ""
        self.lio_t = 0.0
        self.hist = []          # (t, x, y, z, tilt_deg)
        sd = qos_profile_sensor_data
        self.create_subscription(State, "/mavros/state", self._state, 10)
        # MAVROS latches the home (SITL sends it once): a volatile subscriber that starts later never sees it
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(HomePosition, "/mavros/home_position/home", self._home, latched)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose, sd)
        self.create_subscription(Odometry, "/hydrone/lio/odom", self._lio, sd)
        if a.status:
            self.create_subscription(String, a.status, self._status, 10)
        self.status_ready = a.status_ready if a.status else ""
        self.cli = self.create_client(Trigger, a.service)

    def _state(self, m):
        self.state = m

    def _home(self, _m):
        self.home = True

    def _status(self, m):
        self.status = m.data

    def _lio(self, _m):
        self.lio_t = time.time()

    def _pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        now = time.time()
        self.hist.append((now, p.x, p.y, p.z, math.degrees(max(abs(roll), abs(pitch)))))
        self.hist = [h for h in self.hist if h[0] > now - STILL_S - 1.0]

    def here(self):
        return self.hist[-1][1:4]

    def still_level(self):
        now = time.time()
        w = [h for h in self.hist if h[0] >= now - STILL_S]
        if len(w) < 5 or now - w[0][0] < STILL_S - 0.5:
            return False
        span = max(math.dist(a[1:4], b[1:4]) for a in (w[0], w[-1]) for b in w)
        return span < STILL_M and max(h[4] for h in w) < LEVEL_DEG

    def why_not(self, spot=None):
        """None when it can start, else the first reason it can't."""
        now = time.time()
        if self.state is None or not self.state.connected:
            return "no FCU"
        if self.state.armed:
            return "ARMED"
        if not self.home:
            return "FCU has no home (EKF not on the LIO yet)"
        if not self.hist or now - self.hist[-1][0] > STALE_S:
            return "EKF pose stale"
        if now - self.lio_t > STALE_S:
            return "LIO odometry stale (LIO LOST?)"
        if not self.cli.service_is_ready():
            return "start service not up"
        if self.status_ready and not self.status.startswith(self.status_ready):
            return f"mission is '{self.status or 'silent'}', not {self.status_ready}"
        if spot is not None and math.dist(self.here(), spot) > MOVED_M:
            return f"moved {math.dist(self.here(), spot):.2f} m since READY"
        if not self.still_level():
            return "moving or tilted"
        return None


def spin_for(n, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(n, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--service", required=True)
    ap.add_argument("--status", default="")
    ap.add_argument("--status-ready", default="")
    ap.add_argument("--countdown", type=float, default=0.0)
    ap.add_argument("--ready-wait", type=float, default=300.0)
    ap.add_argument("--ready-only", action="store_true")
    ap.add_argument("--nogo", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    n = Gate(a)
    code = 1
    try:
        t, last = time.time(), ""
        while (w := n.why_not()) is not None:
            if w == "ARMED":
                raise RuntimeError("FCU is ARMED")
            if time.time() - t > a.ready_wait:
                raise RuntimeError(f"not ready after {a.ready_wait:.0f} s: {w}")
            if w != last:
                say(f"waiting: {w}")
                last = w
            spin_for(n, 0.2)
        spot = n.here()
        say(f"READY (EKF {spot[0]:+.2f} {spot[1]:+.2f} {spot[2]:+.2f})")
        if a.ready_only:
            code = 0
            return
        say(f"COUNTDOWN {a.countdown:.0f} s")
        t = time.time()
        while time.time() - t < a.countdown:
            spin_for(n, 0.2)
            if (w := n.why_not(spot)) is not None:
                raise RuntimeError(f"during the countdown: {w}")
        if (w := n.why_not(spot)) is not None:
            raise RuntimeError(f"after the countdown: {w}")
        if a.nogo:
            say("NOGO: all checks passed, not starting")
            code = 0
            return
        fut = n.cli.call_async(Trigger.Request())
        t = time.time()
        while not fut.done() and time.time() - t < 10.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        r = fut.result() if fut.done() else None
        if r is None or not r.success:
            raise RuntimeError(f"start call: {r.message if r else 'no reply in 10 s'}")
        say(f"STARTED: {r.message}")
        code = 0
    except RuntimeError as e:
        say(f"ABORT: {e} — not starting")
    finally:
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(code)


if __name__ == "__main__":
    main()
