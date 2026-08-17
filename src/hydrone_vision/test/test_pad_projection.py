#!/usr/bin/env python3
"""
Frame-algebra tests for pad_detector_node's pixel -> world projection.

This is the part of the landing pipeline that cannot be checked by staring at
it: a sign error in the optical/base/world chain produces a perfectly plausible
number that puts the pad on the wrong side of the drone, and the only symptom is
a landing that misses. So the chain is exercised here against poses whose answer
can be worked out by hand.

The node is driven directly — state is injected, `_project` is called — so no
simulator, no camera and no MAVROS are needed.

Run inside the stack container, with the workspace built:

    docker run --rm -v $PWD:/repo -w /repo joao_pessoa_2026-hydrone:latest bash -c \
      '. /ws/install/setup.sh && python3 -m pytest \
       src/hydrone_vision/test/test_pad_projection.py -q'
"""

import math
import os
import sys

import numpy as np
import pytest
import rclpy

from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hydrone_bringup.down_cam_mimic_node import (  # noqa: E402
    DownCamMimicNode, OPTICAL_QUAT)
from hydrone_msgs.msg import PadDetection  # noqa: E402
from hydrone_vision.pad_detector_node import (  # noqa: E402
    PadDetectorNode, quat_to_matrix)


# 640x480 at 90 deg horizontal FOV — what both sim cameras publish.
FX = FY = 320.0
CX, CY = 320.0, 240.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1.0]])

# camera_link -> optical: the standard ROS rotation both mimic nodes publish.
R_LINK_OPT = quat_to_matrix(*OPTICAL_QUAT)


@pytest.fixture(scope="module", autouse=True)
def _ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(**overrides):
    """A PadDetectorNode with no live inputs, ready to have state injected.

    Overrides go in as parameter_overrides, not set_parameters: the node reads
    its parameters once in __init__ and caches them, so setting one afterwards
    would change nothing and the test would quietly pass for the wrong reason.
    """
    node = PadDetectorNode(parameter_overrides=[
        rclpy.parameter.Parameter(k, value=v) for k, v in overrides.items()])
    node.K = K
    return node


def set_pose(node, x, y, z, yaw=0.0, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    node.pose = pose


def mount_down(node, offset=(0.0, 0.0, -0.12)):
    """Install the belly camera's mount, derived the same way the launch does:
    from config.yaml's `rotation: [0, -90, 0]` via down_cam_mimic_node."""
    q = DownCamMimicNode._mount_quaternion([0.0, -90.0, 0.0])
    node.R_base_opt = quat_to_matrix(*q) @ R_LINK_OPT
    node.t_base_opt = np.array(offset, dtype=float)


def mount_forward(node, offset=(0.14, 0.0, -0.08)):
    """The ZED's mount: no rotation on the link, just the optical convention."""
    node.R_base_opt = R_LINK_OPT
    node.t_base_opt = np.array(offset, dtype=float)


# ── The mount rotation itself ────────────────────────────────────────────────

def test_down_camera_mount_points_the_lens_at_the_ground():
    """config.yaml `rotation: [0, -90, 0]` must end up aiming DOWN, not at the sky.

    BiguaSim's pitch is nose-up-positive; ROS RPY about base_link's +Y (left) is
    not. down_cam_mimic_node owns that sign flip, and this is what pins it.
    """
    R = quat_to_matrix(*DownCamMimicNode._mount_quaternion([0.0, -90.0, 0.0]))
    # down_cam_link's own X (its "forward") must point along base_link -Z.
    assert np.allclose(R @ [1, 0, 0], [0, 0, -1], atol=1e-9)
    # And it must not have rolled or yawed on the way.
    assert np.allclose(R @ [0, 1, 0], [0, 1, 0], atol=1e-9)


def test_down_camera_optical_axis_points_down():
    """Through the optical convention too: +Z_optical is the viewing direction."""
    q = DownCamMimicNode._mount_quaternion([0.0, -90.0, 0.0])
    R_base_opt = quat_to_matrix(*q) @ R_LINK_OPT
    assert np.allclose(R_base_opt @ [0, 0, 1], [0, 0, -1], atol=1e-9)


def test_forward_camera_optical_axis_points_forward():
    assert np.allclose(R_LINK_OPT @ [0, 0, 1], [1, 0, 0], atol=1e-9)


def test_real_hardware_static_tf_matches_the_simulated_mount():
    """The belly camera's real-drone TF must equal the one the sim publishes.

    sources_real.launch.py documents a single static_transform_publisher
    (yaw=-pi/2, pitch=0, roll=-pi) as the replacement for down_cam_mimic's two
    hops. If it did not compose to the same rotation, everything would work in
    sim and land in the wrong place on the drone — the worst possible split. So
    the two are compared here rather than trusted.
    """
    # static_transform_publisher's positional form is x y z yaw pitch roll,
    # composed as Rz(yaw)·Ry(pitch)·Rx(roll).
    yaw, pitch, roll = -math.pi / 2.0, 0.0, -math.pi

    def rz(a):
        return np.array([[math.cos(a), -math.sin(a), 0],
                         [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    def ry(a):
        return np.array([[math.cos(a), 0, math.sin(a)],
                         [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])

    def rx(a):
        return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)],
                         [0, math.sin(a), math.cos(a)]])

    real = rz(yaw) @ ry(pitch) @ rx(roll)
    sim = (quat_to_matrix(*DownCamMimicNode._mount_quaternion([0.0, -90.0, 0.0]))
           @ R_LINK_OPT)
    assert np.allclose(real, sim, atol=1e-9), (
        f"real-drone TF and sim mount disagree:\n{real}\nvs\n{sim}")


# ── Down camera: ground-plane projection ─────────────────────────────────────

def test_down_camera_centre_pixel_lands_directly_below():
    node = make_node()
    mount_down(node)
    set_pose(node, 5.0, 3.0, 2.0)

    point, source, rng, frame = node._project(CX, CY)
    assert source == PadDetection.SOURCE_GROUND_PLANE
    assert frame == "map"
    assert np.allclose(point, [5.0, 3.0, 0.0], atol=1e-6)
    # The camera sits 12 cm under the origin, so the ray is 1.88 m long.
    assert rng == pytest.approx(1.88, abs=1e-6)
    node.destroy_node()


def test_down_camera_pixel_to_the_right_maps_to_the_drones_right():
    """Right in the image must be right of the drone, i.e. -Y in ENU at yaw 0.

    Getting this backwards is the classic failure: the drone chases the pad away
    from itself and the alignment never converges.
    """
    node = make_node()
    mount_down(node)
    set_pose(node, 0.0, 0.0, 2.12)   # camera exactly 2.00 m up

    # tan(theta) = 0.5 to the right of the principal point.
    point, _, _, _ = node._project(CX + 0.5 * FX, CY)
    assert point[0] == pytest.approx(0.0, abs=1e-6)
    assert point[1] == pytest.approx(-1.0, abs=1e-6)
    assert point[2] == pytest.approx(0.0, abs=1e-6)
    node.destroy_node()


def test_down_camera_pixel_above_centre_maps_forward():
    """Up in the image is forward of the drone (+X at yaw 0)."""
    node = make_node()
    mount_down(node)
    set_pose(node, 0.0, 0.0, 2.12)

    point, _, _, _ = node._project(CX, CY - 0.5 * FY)
    assert point[0] == pytest.approx(1.0, abs=1e-6)
    assert point[1] == pytest.approx(0.0, abs=1e-6)
    node.destroy_node()


def test_down_camera_respects_vehicle_yaw():
    """Yawed 90 deg, the drone's 'forward' is world +Y. The projection must
    follow the airframe, not the world."""
    node = make_node()
    mount_down(node)
    set_pose(node, 0.0, 0.0, 2.12, yaw=math.pi / 2.0)

    point, _, _, _ = node._project(CX, CY - 0.5 * FY)
    assert point[0] == pytest.approx(0.0, abs=1e-6)
    assert point[1] == pytest.approx(1.0, abs=1e-6)
    node.destroy_node()


def test_ground_plane_height_is_configurable():
    """An elevated pad's plane can be raised; the ray meets it sooner."""
    node = make_node(ground_z=0.5)
    mount_down(node)
    set_pose(node, 1.0, 2.0, 2.12)

    point, _, rng, _ = node._project(CX, CY)
    assert np.allclose(point, [1.0, 2.0, 0.5], atol=1e-6)
    assert rng == pytest.approx(1.5, abs=1e-6)
    node.destroy_node()


# ── Forward camera: depth back-projection ────────────────────────────────────

def test_forward_camera_uses_depth_when_available():
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0)
    node.depth = np.full((480, 640), 10.0, dtype=np.float32)

    point, source, rng, _ = node._project(CX, CY)
    assert source == PadDetection.SOURCE_DEPTH
    # Camera at (0.14, 0, 1.92); 10 m straight down the optical axis is +X.
    assert np.allclose(point, [10.14, 0.0, 1.92], atol=1e-6)
    assert rng == pytest.approx(10.0, abs=1e-6)
    node.destroy_node()


def test_forward_camera_depth_follows_yaw():
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0, yaw=math.pi / 2.0)
    node.depth = np.full((480, 640), 10.0, dtype=np.float32)

    point, _, _, _ = node._project(CX, CY)
    assert point[0] == pytest.approx(0.0, abs=1e-6)
    assert point[1] == pytest.approx(10.14, abs=1e-6)
    node.destroy_node()


def test_forward_camera_falls_back_to_the_ground_plane_without_depth():
    """Past the depth range the ZED returns NaN. A pad on the floor is still
    placeable: the ray is tilted down, so it meets the floor."""
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0)
    node.depth = np.full((480, 640), np.nan, dtype=np.float32)

    # A pixel below the principal point looks downward by atan(0.25).
    point, source, _, _ = node._project(CX, CY + 0.25 * FY)
    assert source == PadDetection.SOURCE_GROUND_PLANE
    # Camera 1.92 m up, ray dropping 0.25 per 1.0 forward -> 7.68 m ahead.
    assert point[0] == pytest.approx(0.14 + 7.68, abs=1e-6)
    assert point[2] == pytest.approx(0.0, abs=1e-6)
    node.destroy_node()


def test_level_or_upward_ray_is_refused():
    """A ray at or above the horizon never meets the floor. It must return None,
    not a huge extrapolated number that would poison the map."""
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0)

    assert node._project(CX, CY) is None                 # dead level
    assert node._project(CX, CY - 0.3 * FY) is None      # tilted up
    node.destroy_node()


# ── Guards ───────────────────────────────────────────────────────────────────

def test_no_pose_means_no_projection():
    node = make_node()
    mount_forward(node)
    assert node._project(CX, CY) is None
    node.destroy_node()


def test_no_camera_info_means_no_projection():
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0)
    node.K = None
    assert node._project(CX, CY) is None
    node.destroy_node()


def test_stale_pose_is_refused():
    """Projecting a fresh image against a pose from seconds ago would place the
    pad wherever the drone used to be."""
    node = make_node(max_pose_age_s=0.5)
    mount_down(node)
    set_pose(node, 0.0, 0.0, 2.12)
    old = rclpy.time.Time(seconds=node.get_clock().now().nanoseconds * 1e-9 - 5.0)
    node.pose.header.stamp = old.to_msg()
    assert node._project(CX, CY) is None
    node.destroy_node()


def test_sparse_depth_patch_falls_back_rather_than_guessing():
    """A handful of valid depth pixels in the window is not enough to trust."""
    node = make_node()
    mount_forward(node)
    set_pose(node, 0.0, 0.0, 2.0)
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    depth[240, 320] = 10.0        # one lonely sample
    node.depth = depth

    result = node._project(CX, CY + 0.25 * FY)
    assert result is not None
    assert result[1] == PadDetection.SOURCE_GROUND_PLANE
    node.destroy_node()
