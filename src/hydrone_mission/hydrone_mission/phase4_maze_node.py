#!/usr/bin/env python3
"""
phase4_maze_node — thin ROS wrapper around the maze core (hydrone_mission/maze).

All the thinking lives in `maze.mission.Mission`, which is pure Python and
tested headless (docs/Phase 4 Maze Explorer.md). This node only:

  in   /cloud_registered   (PointCloud2 in camera_init; odom = camera_init +
                            lidar_mount, the Mid-360's height over base_link)
       /hydrone/lio/odom_raw (odom -> base_link)
       /mavros/state, /mavros/local_position/pose
  out  /mavros/setpoint_velocity/cmd_vel_unstamped at cmd_hz (ArduPilot drops
       GUIDED velocity control after ~1 s of silence, so this never stops)
       arm / GUIDED / takeoff / land over the MAVROS services
  dbg  /hydrone/maze/{grid,path,markers,phase}

Frames: the core plans in `odom` (base_link at takeoff, x forward, y left).
vision_odom_bridge feeds the EKF that pose rotated +90 deg about z, so MAVROS'
local ENU is (-y, x, z) and a velocity goes out the same way. `check_frame`
logs the measured rotation against that assumption on the first metre flown.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from hydrone_mission.maze.mission import Mission, MissionParams


def cloud_xyz(msg, stride=1):
    """(N,3) float32 from a PointCloud2, every `stride`-th point."""
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    raw = raw[::stride]
    return np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0]
                     for c in 'xyz'], axis=1)


def odom_to_enu(v):
    """odom (x forward, y left) -> MAVROS local ENU, as vision_odom_bridge feeds it."""
    return np.array([-v[1], v[0], v[2]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MazeNode(Node):

    def __init__(self):
        super().__init__('phase4_maze_node')
        p = self.declare_parameters('', [
            ('cmd_hz', 20.0),
            ('debug_hz', 2.0),
            ('lidar_mount', [0.0, 0.0, 0.1]),   # Mid-360 over base_link = camera_init offset
            ('max_points', 25000),              # per scan, after striding
            ('flight_z', 0.5),
            ('inflate', 0.22),
            ('v_in', 0.25),
            ('v_out', 0.5),
            ('v_crawl', 0.15),
            ('start_delay', 5.0),               # s of odometry before arming
            ('odom_timeout', 2.0),
            ('auto_start', True),
        ])
        self.par = {q.name: q.value for q in p}
        self.mount = np.array(self.par['lidar_mount'], dtype=float)

        self.mission = Mission(MissionParams(
            flight_z=self.par['flight_z'], inflate=self.par['inflate'],
            v_in=self.par['v_in'], v_out=self.par['v_out'], v_crawl=self.par['v_crawl']),
            logger=lambda m: self.get_logger().info(f'[maze] {m}'))

        self.state = State()
        self.pose = None            # (xyz, yaw) in odom
        self.t_pose = None
        self.local = None           # MAVROS local ENU position
        self.local0 = None
        self.odom0 = None
        self.cmd = None
        self.armed_z = None
        self.started = False
        self.t_first = None
        self.takeoff_sent = False
        self.land_sent = False
        self.finished = False
        self._frame_checked = False
        self._last_scan_t = None

        self.create_subscription(State, '/mavros/state', self._state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._local_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/hydrone/lio/odom_raw', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_registered', self._cloud_cb,
                                 qos_profile_sensor_data)

        self.pub_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pub_phase = self.create_publisher(String, '/hydrone/maze/phase', 10)
        self.pub_grid = self.create_publisher(OccupancyGrid, '/hydrone/maze/grid', 1)
        self.pub_path = self.create_publisher(Path, '/hydrone/maze/path', 1)
        self.pub_mark = self.create_publisher(MarkerArray, '/hydrone/maze/markers', 1)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, '/mavros/cmd/land')
        self._pending = []

        self.create_timer(1.0 / max(self.par['cmd_hz'], 1.0), self._control)
        self.create_timer(1.0 / max(self.par['debug_hz'], 0.1), self._publish_debug)
        self.get_logger().info('phase4_maze_node up: waiting for LIO odometry')

    # ── inputs ──────────────────────────────────────────────────────────────
    def _state_cb(self, m):
        self.state = m

    def _local_cb(self, m):
        self.local = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose = (np.array([p.x, p.y, p.z]), yaw_of(q))
        self.t_pose = self._stamp(m.header)
        if self.t_first is None:
            self.t_first = self.t_pose
        self._check_frame()

    def _cloud_cb(self, m):
        if self.pose is None or self.finished:
            return
        n = m.width * m.height
        stride = max(1, int(math.ceil(n / max(self.par['max_points'], 1000))))
        pts = cloud_xyz(m, stride) + self.mount
        origin = self.pose[0] + self.mount
        t = self._stamp(m.header)
        self._last_scan_t = t
        self.cmd = self.mission.step(t, self.pose, (pts, origin))
        self._act(self.cmd)

    @staticmethod
    def _stamp(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _check_frame(self):
        """Measure the odom -> local ENU rotation once the drone has moved."""
        if self._frame_checked or self.local is None or self.pose is None:
            return
        if self.local0 is None:
            self.local0, self.odom0 = self.local.copy(), self.pose[0].copy()
            return
        d_odom = self.pose[0] - self.odom0
        d_local = self.local - self.local0
        if np.linalg.norm(d_odom[:2]) < 0.5:
            return
        self._frame_checked = True
        want = odom_to_enu(d_odom)
        err = float(np.linalg.norm(want[:2] - d_local[:2]))
        msg = (f'frame check over {np.linalg.norm(d_odom[:2]):.2f} m: odom {np.round(d_odom, 2)} '
               f'-> local {np.round(d_local, 2)}, expected {np.round(want, 2)} (err {err:.2f} m)')
        if err > 0.3:
            self.get_logger().error('FRAME MISMATCH — ' + msg)
        else:
            self.get_logger().info(msg)

    # ── commands ────────────────────────────────────────────────────────────
    def _call(self, cli, req, what):
        if not cli.service_is_ready():
            self.get_logger().warn(f'{what}: service not ready')
            return
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f, w=what: self.get_logger().info(f'{w}: {f.result()}'))
        self._pending.append(fut)

    def _act(self, cmd):
        """Turn one core command into MAVROS calls (the velocity itself is sent
        by the control timer, which must never stop)."""
        if cmd.kind == 'takeoff':
            if not self.started:
                return
            if self.state.mode != 'GUIDED':
                self._call(self.cli_mode, SetMode.Request(custom_mode='GUIDED'), 'GUIDED')
                return
            if not self.state.armed:
                self._call(self.cli_arm, CommandBool.Request(value=True), 'arm')
                return
            if self.armed_z is None:
                self.armed_z = float(self.pose[0][2])
            if not self.takeoff_sent:
                alt = float(cmd.z) - self.armed_z
                self.takeoff_sent = True
                self.get_logger().info(f'takeoff to odom z {cmd.z:.2f} ({alt:.2f} m above here)')
                self._call(self.cli_takeoff, CommandTOL.Request(altitude=alt), 'takeoff')
        elif cmd.kind == 'land':
            if not self.land_sent:
                self.land_sent = True
                self.finished = True
                self.get_logger().info('landing')
                self._call(self.cli_land, CommandTOL.Request(), 'land')

    def _control(self):
        """Publish a velocity setpoint every tick, whatever else is going on."""
        if self.pose is None:
            return
        if not self.started:
            if self.par['auto_start'] and self.t_pose - self.t_first >= self.par['start_delay']:
                self.started = True
                self.get_logger().info('starting the mission')
            return
        if self.land_sent:
            return
        v = np.zeros(3)
        stale = self._last_scan_t is None or (self.t_pose - self._last_scan_t) > self.par['odom_timeout']
        if stale:
            if self._last_scan_t is not None:
                self.get_logger().warn('no scan for too long — holding still', throttle_duration_sec=2.0)
        elif self.cmd is not None and self.cmd.kind == 'velocity':
            v = np.asarray(self.cmd.velocity, dtype=float)
        # takeoff climbs on the FCU's own ramp; don't fight it with velocities
        if self.cmd is not None and self.cmd.kind == 'takeoff':
            return
        enu = odom_to_enu(v)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = (float(a) for a in enu)
        msg.angular.z = float(self.cmd.yaw_rate) if self.cmd is not None else 0.0
        self.pub_vel.publish(msg)

    # ── debug ───────────────────────────────────────────────────────────────
    def _publish_debug(self):
        if self.cmd is None:
            return
        d = self.cmd.debug
        self.pub_phase.publish(String(data=f"{d['phase']} {'|'.join(d['fallback_used'])}"))
        self._publish_grid(d)
        self._publish_path(d)
        self._publish_markers(d)

    def _publish_grid(self, d):
        g, crop = d.get('grid'), d.get('crop')
        if g is None:
            return
        crop = crop or g.crop(1.0)
        i0, i1, j0, j1 = crop
        free = g.free()[i0:i1, j0:j1]
        occ = g.occupied()[i0:i1, j0:j1]
        cov = d.get('covered')
        data = np.full(free.shape, -1, dtype=np.int8)
        data[free] = 0
        if cov is not None and cov.shape == free.shape:
            data[free & cov] = 40         # roofed free space, so it stands out
        data[occ] = 100
        m = OccupancyGrid()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = g.p.res
        # the core indexes [x, y]; an OccupancyGrid runs x fastest, so transpose
        m.info.width, m.info.height = data.shape[0], data.shape[1]
        m.info.origin.position.x = g.origin + i0 * g.p.res
        m.info.origin.position.y = g.origin + j0 * g.p.res
        m.info.origin.position.z = self.par['flight_z'] - 0.4
        m.info.origin.orientation.w = 1.0
        m.data = data.T.ravel(order='C').astype(np.int8).tolist()
        self.pub_grid.publish(m)

    def _publish_path(self, d):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for q in (d.get('path') if d.get('path') is not None else []):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(q[0]), float(q[1])
            ps.pose.position.z = float(self.par['flight_z'])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _marker(self, ns, mid, kind, scale, color):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        m.pose.orientation.w = 1.0
        return m

    def _publish_markers(self, d):
        arr = MarkerArray()
        z = float(self.par['flight_z'])
        colors = {'window': (1.0, 0.2, 0.2, 1.0), 'door': (0.2, 0.4, 1.0, 1.0),
                  'unknown': (0.2, 1.0, 0.2, 1.0), 'predicted': (1.0, 1.0, 0.2, 1.0)}
        for k, o in enumerate(d.get('openings') or []):
            m = self._marker('openings', k, Marker.ARROW, 0.06, colors.get(o.kind, (1., 1., 1., 1.)))
            m.scale.x, m.scale.y, m.scale.z = 0.06, 0.12, 0.12
            m.points = [Point(x=float(o.center[0] - o.normal[0] * 0.5),
                              y=float(o.center[1] - o.normal[1] * 0.5), z=z),
                        Point(x=float(o.center[0]), y=float(o.center[1]), z=z)]
            if not o.confirmed:
                m.color.a = 0.4
            arr.markers.append(m)
        fr = self._marker('frontiers', 0, Marker.SPHERE_LIST, 0.12, (1.0, 0.6, 0.0, 0.9))
        for f in (d.get('frontiers') or []):
            fr.points.append(Point(x=float(f.centroid[0]), y=float(f.centroid[1]), z=z))
        arr.markers.append(fr)
        for name, color, key in (('entrance', (1.0, 0.0, 0.0, 1.0), 'entrance'),
                                 ('exit', (1.0, 0.0, 1.0, 1.0), 'exit')):
            o = d.get(key)
            if o is not None:
                m = self._marker(name, 0, Marker.SPHERE, 0.25, color)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                    float(o.center[0]), float(o.center[1]), z
                arr.markers.append(m)
        if d.get('landing') is not None:
            m = self._marker('landing', 0, Marker.CYLINDER, 0.4, (0.1, 1.0, 1.0, 0.8))
            m.scale.z = 0.05
            m.pose.position.x, m.pose.position.y = float(d['landing'][0]), float(d['landing'][1])
            arr.markers.append(m)
        if d.get('arena') is not None:
            a = d['arena']
            m = self._marker('arena', 0, Marker.LINE_STRIP, 0.05, (0.0, 1.0, 1.0, 0.8))
            cs = np.vstack([a.corners(), a.corners()[:1]])
            m.points = [Point(x=float(c[0]), y=float(c[1]), z=z) for c in cs]
            arr.markers.append(m)
        self.pub_mark.publish(arr)


def main():
    rclpy.init()
    node = MazeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
