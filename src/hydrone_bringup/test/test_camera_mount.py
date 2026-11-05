#!/usr/bin/env python3
"""Tilted / inverted ZED mounts: the math that turns camera poses into base_link.

Plain numpy, runs anywhere:  python3 -m pytest src/hydrone_bringup/test/test_camera_mount.py
"""

import math

import numpy as np
import pytest

from hydrone_bringup.camera_mount import (
    OPTICAL_QUAT, OPTICAL_QUAT_ROT180, base_from_camera_pose, is_upside_down,
    matrix_from_quat, mount_matrix, quat_from_matrix, quat_from_rpy_deg,
    rotate_intrinsics_180, transform)

TILTS = [0.0, 10.0, 15.0, 25.0, 45.0]
ROLLS = [0.0, 180.0]
FX, FY, CX, CY, W, H = 260.0, 260.0, 330.4, 190.2, 672, 376


def _rand_pose(rng, max_deg=30.0):
    rpy = rng.uniform(-max_deg, max_deg, 3)
    rpy[2] = rng.uniform(-180, 180)
    return mount_matrix(rng.uniform(-5, 5, 3), rpy)


def _project(P_opt, cx, cy):
    X, Y, Z = P_opt
    return FX * X / Z + cx, FY * Y / Z + cy


# ── conventions ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize("tilt", TILTS)
def test_positive_pitch_tips_the_lens_down(tilt):
    fwd = mount_matrix((0, 0, 0), [0, tilt, 0])[:3, :3] @ [1, 0, 0]
    t = math.radians(tilt)
    assert np.allclose(fwd, [math.cos(t), 0, -math.sin(t)], atol=1e-12)


def test_inverted_mount_keeps_looking_forward_and_down():
    R = mount_matrix((0, 0, 0), [180, 15, 0])[:3, :3]
    t = math.radians(15)
    assert np.allclose(R @ [1, 0, 0], [math.cos(t), 0, -math.sin(t)], atol=1e-12)
    # the camera's own up points (mostly) down, and its left is the body's right
    assert (R @ [0, 0, 1])[2] < -0.9
    assert np.allclose(R @ [0, 1, 0], [0, -1, 0], atol=1e-12)


def test_matches_the_support_cad():
    """drones/supports/parts/zed_mount.py, t15 block: zed_orig_to_drone()'s
    rotation, re-expressed from the CAD's (X left, Y up, Z fwd) into FLU."""
    R_cad = np.array([[-1.0, 0.0, 0.0],
                      [0.0, -0.966, -0.259],
                      [0.0, -0.259, 0.966]])
    # CAD axes -> FLU: x=Z, y=X, z=Y; ZED-local axes likewise (lateral, up, fwd).
    P = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)
    R_flu = P @ R_cad @ P.T
    assert np.allclose(R_flu, mount_matrix((0, 0, 0), [180, 15, 0])[:3, :3],
                       atol=2e-3)


def test_upside_down_detection():
    assert not is_upside_down([0, 45, 0])
    assert is_upside_down([180, 15, 0])
    assert is_upside_down([180, 45, 0])


def test_quat_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(200):
        R = _rand_pose(rng, 180)[:3, :3]
        assert np.allclose(matrix_from_quat(quat_from_matrix(R)), R, atol=1e-9)


def test_rotated_optical_frame_is_the_optical_frame_turned_about_its_axis():
    R = matrix_from_quat(OPTICAL_QUAT) @ np.diag([-1.0, -1.0, 1.0])
    assert np.allclose(matrix_from_quat(OPTICAL_QUAT_ROT180), R, atol=1e-12)


# ── odometry: camera pose -> base pose ──────────────────────────────────────

@pytest.mark.parametrize("roll", ROLLS)
@pytest.mark.parametrize("tilt", TILTS)
def test_gravity_world_recovers_the_base(tilt, roll):
    """ZED 2i: the SDK world is levelled, the pose is the left lens'."""
    rng = np.random.default_rng(int(tilt) + int(roll))
    T_bc = mount_matrix((0.08, 0.0, -0.037), [roll, tilt, 0])
    T_cl = transform((0, 0.06, 0), (0, 0, 0, 1))
    T_bl = T_bc @ T_cl
    for _ in range(50):
        T_wb = _rand_pose(rng)
        T_wl = T_wb @ T_bl                    # what the SDK reports
        assert np.allclose(base_from_camera_pose(T_wl, T_bl, True), T_wb,
                           atol=1e-9)


@pytest.mark.parametrize("roll", ROLLS)
@pytest.mark.parametrize("tilt", TILTS)
def test_start_pose_world_recovers_the_base(tilt, roll):
    """No IMU: the SDK world is the lens' start pose; odom = base start pose."""
    rng = np.random.default_rng(100 + int(tilt) + int(roll))
    T_bl = mount_matrix((0.08, -0.06, -0.037), [roll, tilt, 0])
    T_wb0 = _rand_pose(rng)
    for _ in range(50):
        T_wb = _rand_pose(rng)
        T_w0_l = np.linalg.inv(T_wb0 @ T_bl) @ (T_wb @ T_bl)
        expected = np.linalg.inv(T_wb0) @ T_wb
        assert np.allclose(base_from_camera_pose(T_w0_l, T_bl, False),
                           expected, atol=1e-9)


def test_level_drone_under_tilted_camera_reads_level():
    """The bench case: drone level, camera 15 deg down and inverted. The SDK
    reports that attitude for the lens; base_link must come out level."""
    T_bl = mount_matrix((0.08, 0.0, -0.037), [180, 15, 0])
    T_wb = transform((1.0, 2.0, 0.5), quat_from_rpy_deg([0, 0, 30]))
    out = base_from_camera_pose(T_wb @ T_bl, T_bl, True)
    assert np.allclose(out[:3, :3] @ [0, 0, 1], [0, 0, 1], atol=1e-12)


# ── upright images from an inverted camera ──────────────────────────────────

def test_rotated_image_and_frame_agree():
    """A point seen by the physical camera, and the same point projected into
    the 180-rotated image through the rotated optical frame, must land on
    pixels that are each other's 180 rotation."""
    rng = np.random.default_rng(7)
    cx2, cy2 = rotate_intrinsics_180(CX, CY, W, H)
    R_flip = matrix_from_quat(OPTICAL_QUAT).T @ matrix_from_quat(OPTICAL_QUAT_ROT180)
    for _ in range(100):
        P = np.array([rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(1, 5)])
        u, v = _project(P, CX, CY)
        u2, v2 = _project(R_flip.T @ P, cx2, cy2)
        assert u2 == pytest.approx((W - 1) - u)
        assert v2 == pytest.approx((H - 1) - v)
