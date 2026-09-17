#!/usr/bin/env python3
"""
phase4_lio_check.py — fly the Kopis on the LIO and report the drift.

Run inside the phase 4 container once the stack is up:

    docker exec -u hydrone -it joao_pessoa_2026-hydrone-1 bash -lc \
        'python3 /ws/src/hydrone_bringup/../../scripts/phase4_lio_check.py'
    (or copy it in; it only needs rclpy + mavros_msgs)

GUIDED, arm, take off to `--alt`, hover, fly a `--side` m square, come back,
land. Everything is flown on the EKF, which only knows the LIO. Ground truth is
read only to print the drift at the end.

Exit code 0 = flew the whole thing and landed; the numbers are printed either way.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry


class Check(Node):

    def __init__(self, args):
        super().__init__('phase4_lio_check')
        self.args = args
        self.state = State()
        self.local = None
        self.lio = None
        self.gt = None
        self.gt0 = None
        self.lio0 = None
        self.errors = []
        self.gate_warn = 0
        self.windows = 0
        self.create_subscription(State, '/mavros/state', self._state, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/hydrone/lio/odom_raw', self._lio, 10)
        self.create_subscription(Odometry, args.gt_topic, self._gt, 50)
        self.create_subscription(DiagnosticStatus, '/hydrone/lio/consistency', self._diag, 10)
        self.sp = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, '/mavros/cmd/land')

    # ── inputs ────────────────────────────────────────────────────────────
    def _state(self, m):
        self.state = m

    def _local(self, m):
        self.local = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    @staticmethod
    def _pose(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return np.array([p.x, p.y, p.z]), yaw

    def _gt(self, m):
        self.gt = self._pose(m)

    def _lio(self, m):
        self.lio = self._pose(m)
        if self.gt is None:
            return
        if self.lio0 is None:
            self.lio0, self.gt0 = self.lio, self.gt
        # motion since start, GT rotated into the LIO's start heading
        dy = self.lio0[1] - self.gt0[1]
        c, s = math.cos(dy), math.sin(dy)
        d_gt = self.gt[0] - self.gt0[0]
        d_gt = np.array([c * d_gt[0] - s * d_gt[1], s * d_gt[0] + c * d_gt[1], d_gt[2]])
        self.errors.append(float(np.linalg.norm((self.lio[0] - self.lio0[0]) - d_gt)))

    def _diag(self, m):
        self.windows += 1
        if m.level != DiagnosticStatus.OK:
            self.gate_warn += 1

    # ── helpers ───────────────────────────────────────────────────────────
    def spin_for(self, seconds, publish=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if publish is not None:
                self.sp.publish(publish)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
        self.get_logger().error(f'timed out waiting for {what}')
        return False

    def call(self, cli, req, what):
        if not cli.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f'{what}: service missing')
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + 60.0
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.result()

    def setpoint(self, xyz):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
        m.pose.orientation.w = 1.0
        return m

    def goto(self, xyz, timeout):
        sp = self.setpoint(xyz)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.sp.publish(sp)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.local is not None and np.linalg.norm(self.local - np.array(xyz)) < 0.2:
                self.spin_for(2.0, sp)
                return True
        self.get_logger().warn(f'did not reach {xyz} within {timeout}s')
        return False

    # ── the flight ────────────────────────────────────────────────────────
    def run(self):
        a = self.args
        log = self.get_logger().info
        if not self.wait(lambda: self.state.connected, 300, 'FCU connection'):
            return False
        if not self.wait(lambda: self.lio is not None, 300, 'LIO odometry'):
            return False
        if not self.wait(lambda: self.local is not None, 600, 'EKF local position'):
            return False
        log('EKF has a position; switching to GUIDED')

        for _ in range(60):
            r = self.call(self.cli_mode, SetMode.Request(custom_mode='GUIDED'), 'mode')
            self.spin_for(1.0)
            if self.state.mode == 'GUIDED':
                break
        for _ in range(60):
            r = self.call(self.cli_arm, CommandBool.Request(value=True), 'arm')
            self.spin_for(2.0)
            if self.state.armed:
                break
        if not self.state.armed:
            self.get_logger().error('never armed (prearm checks? EKF not happy with the LIO?)')
            return False
        start = self.local.copy()
        log(f'armed at {np.round(start, 2)}; taking off to {a.alt} m')
        r = self.call(self.cli_takeoff, CommandTOL.Request(altitude=float(a.alt)), 'takeoff')
        ok = self.wait(lambda: self.local[2] - start[2] > a.alt * 0.85, a.leg_timeout, 'climb')
        if not ok:
            return False
        home = start + np.array([0.0, 0.0, a.alt])
        log('hovering')
        self.spin_for(a.hover, self.setpoint(home))

        s = a.side
        legs = [(s, 0, 0), (s, s, 0), (0, s, 0), (0, 0, 0)]
        if a.waypoints:
            # odom frame (x forward, y left at takeoff, z from the takeoff
            # point). vision_odom_bridge turns odom into MAVROS ENU as (-y, x, z).
            legs = []
            for wp in a.waypoints.split(';'):
                x, y, z = (float(v) for v in wp.split(','))
                legs.append((-y, x, z - a.alt))
        reached = 0
        for leg in legs:
            target = home + np.array(leg, dtype=float)
            log(f'-> {np.round(target, 2)}')
            reached += self.goto(target, a.leg_timeout)
        self.spin_for(a.hover, self.setpoint(home))

        log('landing')
        self.call(self.cli_land, CommandTOL.Request(), 'land')
        self.wait(lambda: not self.state.armed, a.leg_timeout * 2, 'disarm after landing')
        return reached == len(legs) and not self.state.armed

    def report(self, ok):
        e = np.array(self.errors) if self.errors else np.array([float('nan')])
        print('\n==== phase 4 LIO check ====')
        print(f'flight completed: {ok}')
        print(f'LIO vs ground truth: final {e[-1]:.3f} m, max {np.nanmax(e):.3f} m, '
              f'mean {np.nanmean(e):.3f} m over {len(self.errors)} poses')
        print(f'motion prior: {self.gate_warn} of {self.windows} windows flagged')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', type=float, default=1.0)
    ap.add_argument('--side', type=float, default=2.0)
    ap.add_argument('--hover', type=float, default=20.0)
    ap.add_argument('--leg-timeout', type=float, default=240.0)
    ap.add_argument('--waypoints', default='',
                    help='"x,y,z;x,y,z" in the odom frame instead of the square')
    ap.add_argument('--gt-topic', default='/biguasim/uav0_id0/DynamicsSensor/Odom')
    args = ap.parse_args()
    rclpy.init()
    node = Check(args)
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.report(ok)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
