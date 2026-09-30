"""Small rigid-transform helpers, numpy only (no tf_transformations in the image)."""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    """(x, y, z, w), robust for any rotation."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return ((r[2, 1] - r[1, 2]) * s, (r[0, 2] - r[2, 0]) * s,
                (r[1, 0] - r[0, 1]) * s, 0.25 / s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return q[0], q[1], q[2], (r[k, j] - r[j, k]) / s


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def yaw_of(r):
    return math.atan2(r[1, 0], r[0, 0])


def make_tf(r, t):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def inv_tf(m):
    out = np.eye(4)
    out[:3, :3] = m[:3, :3].T
    out[:3, 3] = -m[:3, :3].T @ m[:3, 3]
    return out
