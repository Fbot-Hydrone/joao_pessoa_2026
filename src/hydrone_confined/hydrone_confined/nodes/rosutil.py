"""Message <-> numpy helpers shared by the nodes. Only numpy at import time,
so the conversions can be tested without ROS."""
import math
import struct

import numpy as np


def cloud_xyz(msg, stride=1):
    """(N,3) float32 from a PointCloud2 with float32 x, y, z fields."""
    offs = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    raw = raw[::stride]
    return np.stack([raw[:, offs[c]:offs[c] + 4].copy().view(np.float32)[:, 0]
                     for c in 'xyz'], axis=1)


def _align(i, n):
    return (i + n - 1) // n * n


def livox_custom_xyz(data):
    """(N,3) float32 from a serialized livox_ros_driver2/CustomMsg (raw CDR bytes).

    Built into rclpy's objects, 20k CustomPoints cost tens of ms in Python
    before any work is done; read straight out of the buffer instead.
    Layout (XCDR1, 4-byte encapsulation header, alignment from after it):
      stamp int32 + uint32, frame_id string, timebase uint64 (8-aligned),
      point_num uint32, lidar_id uint8, rsvd uint8[3], points: uint32 count
      then CustomPoint {uint32 offset_time, float32 x, y, z, uint8 x3} = 19
      bytes, padded to 20 between elements.
    """
    buf = memoryview(data)
    little = buf[1] == 1
    e = '<' if little else '>'
    b = bytes(buf[4:])
    i = 8                                           # stamp
    (n,) = struct.unpack_from(e + 'I', b, i)        # frame_id length incl. NUL
    i += 4 + n
    i = _align(i, 8) + 8                            # timebase
    i += 4                                          # point_num
    i += 1 + 3                                      # lidar_id, rsvd
    i = _align(i, 4)
    (count,) = struct.unpack_from(e + 'I', b, i)
    i += 4
    if count == 0:
        return np.zeros((0, 3), dtype=np.float32)
    need = count * 20
    chunk = b[i:i + need]
    if len(chunk) < need:
        chunk = chunk + b'\0' * (need - len(chunk))   # the last element has no pad
    arr = np.frombuffer(chunk, dtype=np.uint8).reshape(count, 20)
    xyz = arr[:, 4:16].copy().view(np.dtype(e + 'f4')).reshape(count, 3)
    return xyz.astype(np.float32)


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def odom_to_enu(v):
    """nav/odom (x forward, y left) -> MAVROS local ENU, as vision_odom_bridge feeds it."""
    return np.array([-v[1], v[0], v[2]])


def enu_to_odom(v):
    return np.array([v[1], -v[0], v[2]])


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9
