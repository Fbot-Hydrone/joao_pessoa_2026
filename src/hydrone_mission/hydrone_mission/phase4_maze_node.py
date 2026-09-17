#!/usr/bin/env python3
"""
phase4_maze_node — fly into the little 3D maze right of spawn and out the other end.

Minimal on purpose (docs/Phase 4 Maze Mission.md):

  TAKEOFF   arm, GUIDED, climb to takeoff_alt
  APPROACH  forward clear of the structure, down to fly_z, sideways to the gap
  MAZE      replan every replan_s: A* on a 2D grid cut from the live voxel map
            between fly_z - below and fly_z + above, obstacles inflated by
            radius. Unknown cells are free: the path gets corrected as the
            lidar sees round each corner. Planning is fenced to the structure's
            footprint (plus the entry and exit aprons) so the shortest path
            can't simply go round the outside.
  EXIT      out through the exit gap to exit_xy at fly_z, then climb
  LAND

Everything is in `odom` (base_link at takeoff, from the LIO). MAVROS wants its
local ENU frame; vision_odom_bridge feeds the EKF odom rotated +90 deg about z,
so local = (-y, x, z) + a constant offset measured at arming.

The entrance/exit and the fence are parameters measured once from the saved
map (tools/phase4/slice_map.py). Finding them automatically is future work.
"""

import heapq
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2


def cloud_xyz(msg):
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0]
                     for c in 'xyz'], axis=1)


def astar(blocked, start, goal):
    """8-connected A* on a bool grid. Returns [(i, j), ...] or None."""
    h, w = blocked.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w and 0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    g = {start: 0.0}
    came = {}
    heap = [(0.0, start)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for di, dj, c in moves:
            nxt = (cur[0] + di, cur[1] + dj)
            if not (0 <= nxt[0] < h and 0 <= nxt[1] < w) or blocked[nxt]:
                continue
            # no corner cutting between two blocked cells
            if di and dj and (blocked[cur[0] + di, cur[1]] or blocked[cur[0], cur[1] + dj]):
                continue
            ng = g[cur] + c
            if ng < g.get(nxt, math.inf):
                g[nxt], came[nxt] = ng, cur
                heapq.heappush(heap, (ng + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1]), nxt))
    return None


def inflate(occ, cells):
    """Dilate a bool grid by a disc of `cells` radius (numpy only)."""
    if cells <= 0:
        return occ.copy()
    out = occ.copy()
    for di in range(-cells, cells + 1):
        for dj in range(-cells, cells + 1):
            if di * di + dj * dj > cells * cells:
                continue
            out |= np.roll(np.roll(occ, di, axis=0), dj, axis=1)
    return out


class MazeNode(Node):

    def __init__(self):
        super().__init__('phase4_maze_node')
        dp = self.declare_parameter
        dp('takeoff_alt', 1.0)
        # Measured on the saved map: floor at odom z -0.62, maze roof at +0.92.
        # The Kopis' base_link rests ~0.6 m above the floor and the lidar sits
        # 0.5 m above base_link, so the band that clears both is ~0.1..0.3.
        dp('fly_z', 0.2)               # odom z inside the maze
        dp('below', 0.55)              # obstacle slice under base_link (floor excluded)
        dp('above', 0.6)               # and over it (roof excluded)
        dp('radius', 0.3)              # inflation, m
        dp('resolution', 0.1)
        # Front-face gaps seen on the saved map (tools/phase4/slice_map.py),
        # 0.55 m outside the front wall at x = 1.35.
        dp('entry_xy', [1.9, -2.45])
        dp('exit_xy', [1.9, -6.5])
        dp('approach_y', 0.0)          # spawn side, clear of the structure
        # planning fence [x_min, x_max, y_min, y_max]: inside the structure, so
        # the shortest path can't go round the outside
        dp('fence', [-0.9, 1.3, -7.9, -0.3])
        # voxels seen fewer times than this are treated as noise
        dp('min_hits', 2)
        dp('replan_s', 1.0)
        dp('lookahead', 0.5)
        dp('reach_tol', 0.25)
        dp('leg_timeout_s', 300.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = p

        self.state = State()
        self.local = None
        self.odom = None
        self.occ_pts = None
        self.offset = None
        self.phase = 'TAKEOFF'
        self.path = None
        self.t_phase = self.now()

        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._cb_local,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/hydrone/lio/odom', self._cb_odom, 10)
        self.create_subscription(PointCloud2, '/hydrone/map/voxels', self._cb_map, 1)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.path_pub = self.create_publisher(Path, '/hydrone/maze/path', 1)
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_land = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.target = None
        self._last_plan = 0.0
        self._calls = {}
        self.create_timer(0.1, self._tick)
        self.get_logger().info('maze mission ready')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── inputs ──────────────────────────────────────────────────────────────
    def _cb_local(self, m):
        self.local = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])

    def _cb_odom(self, m):
        pp = m.pose.pose.position
        self.odom = np.array([pp.x, pp.y, pp.z])

    def _cb_map(self, m):
        pts = cloud_xyz(m)
        offs = {f.name: f.offset for f in m.fields}
        fz = float(self.p('fly_z'))
        keep = (pts[:, 2] > fz - float(self.p('below'))) & (pts[:, 2] < fz + float(self.p('above')))
        if 'intensity' in offs:
            raw = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(-1, m.point_step)
            hits = raw[:, offs['intensity']:offs['intensity'] + 4].copy().view(np.float32)[:, 0]
            keep &= hits >= float(self.p('min_hits'))
        self.occ_pts = pts[keep]

    # ── frames ──────────────────────────────────────────────────────────────
    def to_local(self, xyz):
        x, y, z = xyz
        return np.array([-y, x, z]) + self.offset

    def to_odom(self, local):
        v = local - self.offset
        return np.array([v[1], -v[0], v[2]])

    def publish_sp(self, odom_xyz):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in self.to_local(odom_xyz))
        m.pose.orientation.w = 1.0
        self.sp_pub.publish(m)

    # ── services, fire and forget, at most one call per client every 2 s ──
    def call(self, cli, req):
        last = self._calls.get(id(cli), 0.0)
        if cli.service_is_ready() and self.now() - last > 2.0:
            self._calls[id(cli)] = self.now()
            cli.call_async(req)

    def go(self, phase):
        self.get_logger().info(f'{self.phase} -> {phase}')
        self.phase, self.t_phase, self.target = phase, self.now(), None

    def near(self, odom_xyz):
        return self.odom is not None and np.linalg.norm(self.odom - np.asarray(odom_xyz)) < float(self.p('reach_tol'))

    # ── planning ────────────────────────────────────────────────────────────
    def plan(self, start_xy, goal_xy):
        res = float(self.p('resolution'))
        x0, x1, y0, y1 = (float(v) for v in self.p('fence'))
        h, w = int(round((x1 - x0) / res)) + 1, int(round((y1 - y0) / res)) + 1
        occ = np.zeros((h, w), dtype=bool)
        if self.occ_pts is not None and len(self.occ_pts):
            i = np.floor((self.occ_pts[:, 0] - x0) / res).astype(int)
            j = np.floor((self.occ_pts[:, 1] - y0) / res).astype(int)
            ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
            occ[i[ok], j[ok]] = True
        # np.roll wraps; pad so the fence edge isn't dilated from the far side
        pad = int(math.ceil(float(self.p('radius')) / res)) + 1
        big = np.pad(occ, pad)
        blocked = inflate(big, int(math.ceil(float(self.p('radius')) / res)))[pad:-pad, pad:-pad]
        cell = lambda xy: (int(np.clip(round((xy[0] - x0) / res), 0, h - 1)),  # noqa: E731
                           int(np.clip(round((xy[1] - y0) / res), 0, w - 1)))
        s, g = cell(start_xy), cell(goal_xy)
        blocked[s] = blocked[g] = False
        path = astar(blocked, s, g)
        if path is None:
            return None
        return [(x0 + i * res, y0 + j * res) for i, j in path]

    def carrot(self, path, pos_xy):
        """The path point `lookahead` metres past the closest one."""
        pts = np.array(path)
        k = int(np.argmin(np.linalg.norm(pts - pos_xy, axis=1)))
        acc = 0.0
        while k + 1 < len(pts) and acc < float(self.p('lookahead')):
            acc += float(np.linalg.norm(pts[k + 1] - pts[k]))
            k += 1
        return pts[k]

    def publish_path(self, path, z):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ── the mission ─────────────────────────────────────────────────────────
    def _tick(self):
        if self.local is None or self.odom is None:
            return
        alt, fz = float(self.p('takeoff_alt')), float(self.p('fly_z'))
        entry, exit_ = np.array(self.p('entry_xy')), np.array(self.p('exit_xy'))
        if self.phase != 'TAKEOFF' and self.now() - self.t_phase > float(self.p('leg_timeout_s')) \
                and self.phase not in ('LAND', 'DONE'):
            self.get_logger().error(f'{self.phase} timed out, landing where we are')
            self.go('LAND')

        if self.phase == 'TAKEOFF':
            if self.offset is None:
                self.offset = self.local - np.array([-self.odom[1], self.odom[0], self.odom[2]])
            if self.state.mode != 'GUIDED':
                self.call(self.cli_mode, SetMode.Request(custom_mode='GUIDED'))
                return
            if not self.state.armed:
                self.call(self.cli_arm, CommandBool.Request(value=True))
                return
            if self.target is None and self.cli_takeoff.service_is_ready():
                self.target = np.array([self.odom[0], self.odom[1], alt])
                self.call(self.cli_takeoff, CommandTOL.Request(altitude=alt))
            if self.odom[2] > alt * 0.85:
                self.go('APPROACH_HIGH')

        # The approach never crosses the structure: forward along y = approach_y
        # (clear of it), down, then sideways along the front face to the gap.
        # The Kopis' body hangs ~0.6 m under base_link, so a diagonal over the
        # roof corner at takeoff_alt hits it (it did, first run).
        elif self.phase == 'APPROACH_HIGH':
            t = np.array([entry[0], float(self.p('approach_y')), alt])
            self.publish_sp(t)
            if self.near(t):
                self.go('APPROACH_LOW')

        elif self.phase == 'APPROACH_LOW':
            t = np.array([entry[0], float(self.p('approach_y')), fz])
            self.publish_sp(t)
            if self.near(t):
                self.go('APPROACH_SIDE')

        elif self.phase == 'APPROACH_SIDE':
            t = np.array([entry[0], entry[1], fz])
            self.publish_sp(t)
            if self.near(t):
                self.go('MAZE')

        elif self.phase == 'MAZE':
            pos = self.odom[:2]
            x0, x1, y0, y1 = (float(v) for v in self.p('fence'))
            goal = np.array([np.clip(exit_[0], x0, x1), np.clip(exit_[1], y0, y1)])
            if self.now() - self._last_plan > float(self.p('replan_s')) or self.path is None:
                self._last_plan = self.now()
                path = self.plan(pos, goal)
                if path is None:
                    self.get_logger().warn('no path to the exit through the known map; holding',
                                           throttle_duration_sec=5.0)
                else:
                    self.path = path
                    self.publish_path(path, fz)
            if self.path is None:
                self.publish_sp(np.array([pos[0], pos[1], fz]))
                return
            c = self.carrot(self.path, pos)
            self.publish_sp(np.array([c[0], c[1], fz]))
            if np.linalg.norm(pos - goal) < float(self.p('reach_tol')):
                self.go('EXIT_LOW')

        elif self.phase == 'EXIT_LOW':
            t = np.array([exit_[0], exit_[1], fz])
            self.publish_sp(t)
            if self.near(t):
                self.go('EXIT')

        elif self.phase == 'EXIT':
            t = np.array([exit_[0], exit_[1], alt])
            self.publish_sp(t)
            if self.near(t):
                self.go('LAND')

        elif self.phase == 'LAND':
            if self.state.mode == 'LAND':
                self.go('DONE')
            else:
                self.call(self.cli_land, CommandTOL.Request())

        elif self.phase == 'DONE':
            if not self.state.armed and self.now() - self.t_phase > 5.0:
                self.get_logger().info('maze mission finished', once=True)


def main(args=None):
    rclpy.init(args=args)
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
