"""Camera mount math shared by the ZED sources (zed_mimic_node, zed_sdk_node).

Pure numpy, no ROS, so it runs in plain pytest.

A mount is `xyz` (metres, base_link) + `rpy_deg` ([roll, pitch, yaw], degrees,
fixed-axis XYZ, i.e. R = Rz(yaw)·Ry(pitch)·Rx(roll)). Same convention as
BiguaSim's sensor `rotation` and down_cam_mimic_node: +pitch tips the lens
DOWN. A camera upside down on its support is roll 180.

    forward ZED, level              [0,   0, 0]
    tilted 15 deg down              [0,  15, 0]
    upside down + 15 deg down       [180, 15, 0]   (the drone's t15 block)
"""

import math

import numpy as np

# camera_link -> camera_optical_frame, RPY(-90, 0, -90): X fwd/Y left/Z up to
# Z fwd/X right/Y down.
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)            # (x, y, z, w)
# The same frame turned 180 deg about its own Z (the viewing axis). It is the
# optical frame of an image rotated 180 deg, see rotate_intrinsics_180.
OPTICAL_QUAT_ROT180 = (0.5, 0.5, 0.5, 0.5)


def quat_from_rpy_deg(rpy_deg):
    """[roll, pitch, yaw] degrees -> (x, y, z, w)."""
    roll, pitch, yaw = (math.radians(float(v)) * 0.5 for v in rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def matrix_from_quat(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_matrix(R):
    """3x3 rotation -> (x, y, z, w), Shepperd's method."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s, (R[1, 0] - R[0, 1]) / s)


def transform(xyz, q):
    """4x4 from a translation and a quaternion."""
    T = np.eye(4)
    T[:3, :3] = matrix_from_quat(q)
    T[:3, 3] = [float(v) for v in xyz]
    return T


def mount_matrix(xyz, rpy_deg):
    """4x4 T_base_camera."""
    return transform(xyz, quat_from_rpy_deg(rpy_deg))


def is_upside_down(rpy_deg):
    """True when the camera's own up points below the body's horizon."""
    return mount_matrix((0, 0, 0), rpy_deg)[2, 2] < 0.0


def base_from_camera_pose(T_world_cam, T_base_cam, gravity_world):
    """A tracked camera pose -> the body's pose (odom -> base_link).

    gravity_world: the tracker's world is levelled by an IMU (ZED 2/2i/mini,
    SDK default), so it is already a valid odom frame; only the mount is
    peeled off the camera end. Otherwise the world is the camera's own pose
    at start (first-gen ZED, or SDK image flip on), tilted like the mount, so
    it is re-expressed from the body's start pose instead.
    """
    T_cam_base = np.linalg.inv(T_base_cam)
    if gravity_world:
        return T_world_cam @ T_cam_base
    return T_base_cam @ T_world_cam @ T_cam_base


def rotate_intrinsics_180(cx, cy, width, height):
    """Principal point of the image rotated by 180 deg (img[::-1, ::-1])."""
    return (width - 1) - cx, (height - 1) - cy
