#!/usr/bin/env python3
"""
lio_map_node — the persistent map built from LIO poses.

Three layers, all in `odom` (base_link at start):

  voxels       occupancy-style grid (voxel_size), a hit count per cell.
               What planning reads. /hydrone/map/voxels
  dense cloud  the plain points, without the swarm: a point is kept only if
               its dedup cell (dedup_size) is still empty, so revisiting a wall
               adds nothing. /hydrone/map/cloud
  keyframes    raw registered scans + the pose they were taken at, every
               keyframe_dist m or keyframe_angle deg. Lossless, for
               re-processing later.

Saved on /hydrone/map/save (std_srvs/Trigger) and on shutdown to
<map_dir>/<map_name>/ as voxels.pcd, cloud_dedup.pcd, keyframes/NNNN.pcd,
poses.csv. `load_map` starts from voxels + cloud of a saved map (no
relocalization: it assumes the drone starts where that map's odom began).

FAST-LIO registers scans in camera_init (the lidar's pose at start), so every
point is moved by the fixed mount transform to land in odom.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

from hydrone_lio.geometry import quat_to_matrix, rpy_to_matrix


def cloud_xyz(msg):
    """xyz (N,3) float32 out of any PointCloud2 with float32 x/y/z fields."""
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    return np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0]
                     for c in 'xyz'], axis=1)


def make_cloud(points, frame, stamp, extra=None):
    """PointCloud2 from (N,3) float32 plus optional named float32 columns."""
    cols = [points.astype(np.float32)]
    names = ['x', 'y', 'z']
    for name, values in (extra or {}).items():
        cols.append(np.asarray(values, dtype=np.float32).reshape(-1, 1))
        names.append(name)
    data = np.hstack(cols).astype(np.float32)
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.height, msg.width = 1, len(data)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(names)]
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * len(data)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


def write_pcd(path, points, extra=None):
    """Binary PCD, float32 fields."""
    cols = [points.astype(np.float32)]
    names = ['x', 'y', 'z']
    for name, values in (extra or {}).items():
        cols.append(np.asarray(values, dtype=np.float32).reshape(-1, 1))
        names.append(name)
    data = np.hstack(cols).astype(np.float32)
    n = len(data)
    header = (
        '# .PCD v0.7\nVERSION 0.7\n'
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(['4'] * len(names))}\n"
        f"TYPE {' '.join(['F'] * len(names))}\n"
        f"COUNT {' '.join(['1'] * len(names))}\n"
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(data.tobytes())


def read_pcd(path):
    """Reads what write_pcd writes. Returns (names, (N,k) float32)."""
    with open(path, 'rb') as f:
        names, n = [], 0
        while True:
            line = f.readline().decode().strip()
            if line.startswith('FIELDS'):
                names = line.split()[1:]
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
            elif line.startswith('DATA'):
                break
        data = np.frombuffer(f.read(), dtype=np.float32).reshape(n, len(names))
    return names, data


def voxel_keys(points, size):
    """int64 key per point; 21 bits per axis covers +-10 km at 1 cm."""
    idx = np.floor(points / size).astype(np.int64) + (1 << 20)
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def key_centers(keys, size):
    mask = (1 << 21) - 1
    idx = np.stack([(keys >> 42) & mask, (keys >> 21) & mask, keys & mask], axis=1) - (1 << 20)
    return ((idx + 0.5) * size).astype(np.float32)


class LioMapNode(Node):

    def __init__(self):
        super().__init__('lio_map_node')
        dp = self.declare_parameter
        dp('in_cloud', '/cloud_registered')
        dp('in_odom', '/hydrone/lio/odom_raw')
        dp('frame', 'odom')
        dp('mount_xyz', [0.0, 0.0, 0.1])
        dp('mount_rpy_deg', [0.0, 0.0, 0.0])
        dp('voxel_size', 0.05)
        dp('dedup_size', 0.02)
        dp('keyframe_dist', 0.5)
        dp('keyframe_angle_deg', 15.0)
        dp('publish_period_s', 2.0)
        dp('map_dir', os.environ.get('HYDRONE_MAPS', '/ws/maps'))
        dp('map_name', 'phase4')
        dp('load_map', False)
        dp('save_on_exit', True)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.frame = p('frame')
        self.voxel, self.dedup = float(p('voxel_size')), float(p('dedup_size'))
        self.kf_dist = float(p('keyframe_dist'))
        self.kf_angle = math.radians(float(p('keyframe_angle_deg')))
        self.out_dir = os.path.join(p('map_dir'), p('map_name'))
        self.save_on_exit = bool(p('save_on_exit'))
        # camera_init -> odom is the lidar mount, fixed
        self.r_mount = rpy_to_matrix(*(math.radians(v) for v in p('mount_rpy_deg')))
        self.t_mount = np.array(p('mount_xyz'), dtype=np.float64)

        self.voxels = {}            # key -> hits
        self.dedup_keys = set()
        self.cloud_chunks = []      # (N,3) float32, the kept points
        self.keyframes = []         # (stamp_s, pose 4x4, points in odom)
        self._last_kf_pose = None
        self._pose = None
        self._dirty = False
        # stamp of the newest scan: the map is published with it, not with
        # now(), because the sensors run on sim time and a wall-clock stamp
        # lands minutes "in the future" of every TF and RViz drops it
        self._last_stamp = None

        if p('load_map'):
            self._load()

        self.pub_vox = self.create_publisher(PointCloud2, '/hydrone/map/voxels', 1)
        self.pub_cloud = self.create_publisher(PointCloud2, '/hydrone/map/cloud', 1)
        self.create_subscription(PointCloud2, p('in_cloud'), self._cb_cloud, 5)
        self.create_subscription(Odometry, p('in_odom'), self._cb_odom, 20)
        self.create_service(Trigger, '/hydrone/map/save', self._srv_save)
        self.create_timer(float(p('publish_period_s')), self._publish)
        self.get_logger().info(f'lio_map: {p("in_cloud")} -> {self.out_dir}')

    def _cb_odom(self, msg: Odometry):
        q, pp = msg.pose.pose.orientation, msg.pose.pose.position
        pose = np.eye(4)
        pose[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
        pose[:3, 3] = [pp.x, pp.y, pp.z]
        self._pose = pose

    def _is_keyframe(self):
        if self._pose is None:
            return False
        if self._last_kf_pose is None:
            return True
        d = np.linalg.norm(self._pose[:3, 3] - self._last_kf_pose[:3, 3])
        rel = self._last_kf_pose[:3, :3].T @ self._pose[:3, :3]
        ang = math.acos(max(-1.0, min(1.0, (np.trace(rel) - 1) / 2)))
        return d >= self.kf_dist or ang >= self.kf_angle

    def _cb_cloud(self, msg: PointCloud2):
        if msg.width == 0:
            return
        self._last_stamp = msg.header.stamp
        pts = cloud_xyz(msg).astype(np.float64) @ self.r_mount.T + self.t_mount
        pts = pts[np.all(np.isfinite(pts), axis=1)].astype(np.float32)

        keys, counts = np.unique(voxel_keys(pts, self.voxel), return_counts=True)
        for k, c in zip(keys.tolist(), counts.tolist()):
            self.voxels[k] = self.voxels.get(k, 0) + c

        dkeys = voxel_keys(pts, self.dedup)
        _, first = np.unique(dkeys, return_index=True)
        fresh = [i for i in first.tolist() if dkeys[i] not in self.dedup_keys]
        if fresh:
            self.dedup_keys.update(dkeys[fresh].tolist())
            self.cloud_chunks.append(pts[fresh])

        if self._is_keyframe():
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.keyframes.append((stamp, self._pose.copy(), pts))
            self._last_kf_pose = self._pose.copy()
        self._dirty = True

    def _publish(self):
        if not self._dirty:
            return
        self._dirty = False
        stamp = self._last_stamp or self.get_clock().now().to_msg()
        if self.voxels and self.pub_vox.get_subscription_count():
            keys = np.fromiter(self.voxels.keys(), dtype=np.int64, count=len(self.voxels))
            hits = np.fromiter(self.voxels.values(), dtype=np.float32, count=len(self.voxels))
            self.pub_vox.publish(make_cloud(key_centers(keys, self.voxel), self.frame, stamp,
                                            {'intensity': hits}))
        if self.cloud_chunks and self.pub_cloud.get_subscription_count():
            self.pub_cloud.publish(make_cloud(self._dense(), self.frame, stamp))

    def _dense(self):
        if len(self.cloud_chunks) > 1:
            self.cloud_chunks = [np.concatenate(self.cloud_chunks)]
        return self.cloud_chunks[0] if self.cloud_chunks else np.zeros((0, 3), np.float32)

    def save(self):
        os.makedirs(os.path.join(self.out_dir, 'keyframes'), exist_ok=True)
        keys = np.fromiter(self.voxels.keys(), dtype=np.int64, count=len(self.voxels))
        hits = np.fromiter(self.voxels.values(), dtype=np.float32, count=len(self.voxels))
        write_pcd(os.path.join(self.out_dir, 'voxels.pcd'), key_centers(keys, self.voxel),
                  {'hits': hits})
        dense = self._dense()
        write_pcd(os.path.join(self.out_dir, 'cloud_dedup.pcd'), dense)
        with open(os.path.join(self.out_dir, 'poses.csv'), 'w') as f:
            f.write('keyframe,stamp,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22\n')
            for i, (stamp, pose, pts) in enumerate(self.keyframes):
                write_pcd(os.path.join(self.out_dir, 'keyframes', f'{i:04d}.pcd'), pts)
                r = pose[:3, :3].ravel()
                f.write(f'{i},{stamp:.6f},{pose[0, 3]:.4f},{pose[1, 3]:.4f},{pose[2, 3]:.4f},'
                        + ','.join(f'{v:.6f}' for v in r) + '\n')
        return (f'{len(keys)} voxels, {len(dense)} points, {len(self.keyframes)} keyframes '
                f'-> {self.out_dir}')

    def _srv_save(self, request, response):
        try:
            response.message = self.save()
            response.success = True
            self.get_logger().info(response.message)
        except OSError as e:
            response.success, response.message = False, str(e)
            self.get_logger().error(f'map save failed: {e}')
        return response

    def _load(self):
        vox_path = os.path.join(self.out_dir, 'voxels.pcd')
        if not os.path.exists(vox_path):
            self.get_logger().warn(f'load_map: nothing at {self.out_dir}, starting empty')
            return
        names, vox = read_pcd(vox_path)
        keys = voxel_keys(vox[:, :3], self.voxel)
        hits = vox[:, names.index('hits')] if 'hits' in names else np.ones(len(vox))
        self.voxels = dict(zip(keys.tolist(), hits.astype(int).tolist()))
        cloud_path = os.path.join(self.out_dir, 'cloud_dedup.pcd')
        if os.path.exists(cloud_path):
            _, cloud = read_pcd(cloud_path)
            cloud = cloud[:, :3].copy()
            self.cloud_chunks = [cloud]
            self.dedup_keys = set(voxel_keys(cloud, self.dedup).tolist())
        self._dirty = True
        self.get_logger().info(
            f'loaded {len(self.voxels)} voxels, {len(self._dense())} points from {self.out_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = LioMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.save_on_exit and (node.voxels or node.keyframes):
            try:
                print(node.save(), flush=True)
            except OSError as e:
                print(f'map save on exit failed: {e}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
