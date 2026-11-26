#!/usr/bin/env python3
"""floor_plain — FLOOR_Z for NOSURFACE=1 (FCU in PLAIN mode, no surface_offset).

In PLAIN mode the Pixhawk's own DroneCAN rangefinder drives the EKF height
directly, so the FCU z is height above WHATEVER IS UNDER THE DRONE (plus a
constant picked at boot). Standing still on the takeoff base, z is that
constant: the "floor" of every surface the drone sits on. run_phase1.sh adds
ALT to it, so the drone flies ALT metres above what is under it.

Checks: MAVROS pose + rangefinder arriving, disarmed, rangefinder reading the
ground (< MAX_GROUND_RANGE), z still over 5 s. Prints `FLOOR_Z=<m>` as the
last line, exit 1 on any failure. Needs sources_real (surface_offset:=false).
"""

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

MAX_GROUND_RANGE = 0.6   # raw sensor on a flat surface reads ~0.17
MAX_ABS = 4.0
WAIT_S = 90.0
WINDOW_S = 5.0
TRIES = 4


def say(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class L(Node):
    def __init__(self):
        super().__init__("floor_plain")
        self.z = self.rng = self.armed = None
        self.n_z = self.n_rng = 0
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose, qos_profile_sensor_data)
        self.create_subscription(Range, "/mavros/distance_sensor/rangefinder",
                                 self._rng, qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self._state, 10)

    def _pose(self, m):
        self.z = m.pose.position.z
        self.n_z += 1

    def _rng(self, m):
        self.rng = m.range
        self.n_rng += 1

    def _state(self, m):
        self.armed = m.armed


def spin_for(node, s):
    t = time.time()
    while time.time() - t < s:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    n = L()
    try:
        say("waiting for MAVROS pose, rangefinder and state (PLAIN mode)")
        t = time.time()
        while not (n.n_z > 10 and n.n_rng > 5 and n.armed is not None):
            if time.time() - t > WAIT_S:
                raise RuntimeError(
                    f"no data after {WAIT_S:.0f} s (pose {n.n_z}, rangefinder "
                    f"{n.n_rng}, state {n.armed}) — FCU really in PLAIN mode?")
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.armed:
            raise RuntimeError("FCU is ARMED")
        for i in range(1, TRIES + 1):
            zs, rs = [], []
            t = time.time()
            while time.time() - t < WINDOW_S:
                rclpy.spin_once(n, timeout_sec=0.05)
                zs.append(n.z)
                rs.append(n.rng)
            f = sum(zs) / len(zs)
            r = sum(rs) / len(rs)
            bad = []
            if max(zs) - min(zs) > 0.05:
                bad.append(f"z moving {min(zs):+.2f}..{max(zs):+.2f}")
            if r > MAX_GROUND_RANGE:
                bad.append(f"rangefinder {r:.2f} m: not on the ground?")
            if abs(f) > MAX_ABS:
                bad.append(f"z {f:+.2f} m out of range")
            if not bad:
                break
            if i == TRIES:
                raise RuntimeError("; ".join(bad))
            say(f"ground check {i}/{TRIES} failed ({'; '.join(bad)}), again")
            spin_for(n, 3.0)
        say(f"PLAIN: on the ground z {f:+.3f}, rangefinder {r:.3f} m")
        print(f"FLOOR_Z={f:.3f}", flush=True)
        code = 0
    except RuntimeError as e:
        say(f"ABORT: {e}")
        code = 1
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
