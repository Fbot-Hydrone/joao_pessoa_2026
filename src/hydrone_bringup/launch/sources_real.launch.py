"""
hydrone_bringup/launch/sources_real.launch.py

REAL SOURCES layer — the drone's hardware, publishing the buses the autonomy
layer already consumes.

This is the real-drone counterpart of sources_sim.launch.py and it produces the
SAME output contract, so the autonomy stack runs unchanged when the flag flips:

  OUTPUT CONTRACT (matches sources_sim.launch.py):
    /zed/zed_node/rgb/image_rect_color, /zed/zed_node/rgb/camera_info,
    /zed/zed_node/depth/depth_registered, /zed/zed_node/depth/camera_info,
    /zed/zed_node/point_cloud/cloud_registered,
    /zed/zed_node/odom,
    /down_cam/image_raw, /down_cam/camera_info,
    /mavros/*  (incl. /mavros/vision_pose/pose,
                /mavros/distance_sensor/rangefinder)
    TF: odom -> base_link
        base_link -> zed_camera_link -> zed_left_camera_frame
                                     -> zed_left_camera_optical_frame
        base_link -> down_cam_link   -> down_cam_optical_frame

NOTHING ABOVE THE SOURCES IS TOLD APART. The autonomy layer takes no sim/real
argument at all — phase1.launch.py and landing_sites.launch.py run with
identical defaults in both worlds. If you ever find yourself wanting to pass an
autonomy override from here, the fix belongs on this side: make the hardware
publish what the simulator publishes.

What differs from sim (sources only, never autonomy)
----------------------------------------------------
  - NO SITL, NO ardubridge, NO zed_mimic, NO down_cam_mimic,
    NO visual_odometry_node, NO rangefinder_bridge.
  - zed_sdk_node replaces zed_mimic_node AND visual_odometry_node: the ZED SDK
    produces the images, the depth, the cloud and its own visual tracking.
  - down_cam_usb_node replaces down_cam_mimic_node and carries the mount TF the
    mimic used to provide.
  - The VL53L1X is wired over I2C into the Pixhawk. ArduPilot reads it natively
    and MAVROS PUBLISHES /mavros/distance_sensor/* — which is why there is no
    rangefinder_bridge here, and why mavros_distance_sensor.yaml (which puts
    that plugin in SUBSCRIBER mode) is sim-only.
  - vision_odom_bridge is AGNOSTIC and runs in both worlds, unchanged.

Why zed_sdk_node and not zed_wrapper
------------------------------------
`zed-ros2-wrapper` is the right answer on hardware that can run it, and the
contract above is deliberately the wrapper's topic names so that swapping it in
is a one-line change here. This drone cannot run it today: the board is a Tegra
X1 on L4T R32.6.1, so the ZED SDK is 4.0.1 against CUDA 10.2, while ROS 2 Humble
needs Ubuntu 22.04. See zed_sdk_node's docstring and docs/JETSON-REAL-STACK.md.

Two things zed_wrapper gets wrong for this stack if it is ever used here:
  - the point cloud must be ON — feature_map_node has no other geometry source;
  - `pos_tracking.publish_map_tf` must be FALSE. The wrapper broadcasts
    map -> odom by default, where `map` is the ZED's loop-closed frame. Here
    `map` is the FCU's frame and map_odom_node owns that edge. Two broadcasters
    of one edge, meaning two different things, is a corrupt TF tree.

BEFORE THE FIRST FLIGHT
-----------------------
  * DONE 2026-08-22: the belly camera is calibrated and the numbers are the
    defaults below. Re-do it if the camera is swapped or the capture
    resolution changes -- fx/fy/cx/cy are in PIXELS and do not survive either.
  * Measure down_cam_mount_xyz / down_cam_mount_rpy_deg on the airframe. The
    defaults are the SIMULATED mount, which is a starting point and not your
    drone.
  * Check fcu_url matches how the Pixhawk is actually wired.
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_pkg = get_package_share_directory("hydrone_bringup")
    mavros_share = get_package_share_directory("mavros")

    args = [
        # ── Flight controller ───────────────────────────────────────────────
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyTHS1:921600",
            description="How MAVROS reaches the Pixhawk. Serial on the Jetson's "
                        "UART by default; use /dev/ttyACM0:921600 for USB, or a "
                        "udp:// URL for a companion link."),
        DeclareLaunchArgument("gcs_url", default_value=""),

        # ── Forward ZED ─────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "zed_resolution", default_value="VGA",
            description="VGA | HD720 | HD1080 | HD2K. VGA is 672x376 per eye "
                        "and is the sane default on a Tegra X1 — the depth pass "
                        "scales with pixels and the detectors need the CPU."),
        DeclareLaunchArgument("zed_fps", default_value="15"),
        DeclareLaunchArgument(
            "zed_depth_mode", default_value="PERFORMANCE",
            description="NONE | PERFORMANCE | QUALITY | ULTRA | NEURAL. The "
                        "pads are metres away and metres wide; this is not a "
                        "scene-reconstruction job."),
        DeclareLaunchArgument(
            "zed_point_cloud", default_value="false",
            description="Publish /zed/zed_node/point_cloud/cloud_registered. "
                        "Only feature_map_node reads it, and it is the most "
                        "expensive publication here — leave it off unless the "
                        "coverage map is wanted."),
        DeclareLaunchArgument(
            "zed_odom", default_value="true",
            description="Publish the SDK's visual tracking as "
                        "/zed/zed_node/odom, which vision_odom_bridge relays to "
                        "the FCU. Set false if something else owns the position "
                        "estimate."),
        DeclareLaunchArgument(
            "zed_offset_xyz", default_value="[0.10, 0.0, 0.0]",
            description="Where the ZED sits on the airframe, metres in "
                        "base_link."),

        # ── Belly camera ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "down_cam_device",
            default_value="/dev/v4l/by-path/"
                          "platform-70090000.xusb-usb-0:2.2:1.0-video-index0",
            description="The belly camera, addressed BY USB PORT rather than "
                        "as /dev/videoN. Enumeration order is not stable: the "
                        "C270 and the ZED both register V4L2 nodes, and after "
                        "a reboot or a replug the ZED can take video0 — which "
                        "would feed stereo side-by-side frames to the pad "
                        "detector with nothing reporting an error. This path "
                        "is the port on the airframe, so it survives swapping "
                        "the camera for another C270. It does NOT survive "
                        "moving it to a different USB socket: re-read "
                        "`ls -l /dev/v4l/by-path/` if you do."),
        DeclareLaunchArgument("down_cam_width", default_value="640"),
        DeclareLaunchArgument("down_cam_height", default_value="480"),
        DeclareLaunchArgument("down_cam_fps", default_value="15"),
        DeclareLaunchArgument(
            "down_cam_calibrated", default_value="true",
            description="False publishes a NOMINAL pinhole camera_info from "
                        "nominal_hfov_deg. True uses the measured fx/fy/cx/cy "
                        "and distortion below."),
        DeclareLaunchArgument("down_cam_hfov_deg", default_value="60.0"),

        # Measured 2026-08-22 with the C270 on a desktop, ChArUco 9x11,
        # 22 mm squares, 16 mm markers, DICT_4X4_250; 18 views, reprojection
        # error 0.4283 px. See docs/CALIBRATION.md.
        #
        # The sanity check these passed matters as much as the error: fx 814.6
        # implies a 52 deg DIAGONAL field of view, which is what a C270 has.
        # Earlier attempts on thin coverage returned fx from 1085 to 1599 --
        # a 40 deg lens and narrower -- at reprojection errors as low as
        # 0.4866 px. Low error does not mean correct; see CALIBRATION.md 4.
        DeclareLaunchArgument("down_cam_fx", default_value="814.643"),
        DeclareLaunchArgument("down_cam_fy", default_value="818.604"),
        DeclareLaunchArgument("down_cam_cx", default_value="299.707"),
        DeclareLaunchArgument("down_cam_cy", default_value="249.397"),
        DeclareLaunchArgument(
            "down_cam_distortion",
            default_value="[0.015598, -0.369012, 0.007169, -0.006748, "
                          "1.862586]",
            description="OpenCV plumb-bob [k1, k2, p1, p2, k3]. The node had "
                        "this parameter from the start but the launch never "
                        "passed it, so a distortion argument silently did "
                        "nothing. k3 = 1.86 is large and was fitted from only "
                        "18 views; if pad positions look systematically off "
                        "toward the frame edges, recapture with wider "
                        "coverage before blaming the mount."),
        DeclareLaunchArgument(
            "down_cam_mount_xyz", default_value="[0.0, 0.0, -0.12]",
            description="Belly camera position in base_link, metres. THE "
                        "SIMULATED MOUNT is the default — measure your own."),
        DeclareLaunchArgument(
            "down_cam_mount_rpy_deg", default_value="[0.0, 90.0, 0.0]",
            description="Belly camera orientation in base_link, degrees. "
                        "pitch +90 points the lens at the ground."),
    ]

    # ── The forward ZED ─────────────────────────────────────────────────────
    zed = Node(
        package="hydrone_bringup",
        executable="zed_sdk_node",
        name="zed_sdk",
        output="screen",
        parameters=[{
            "resolution": LaunchConfiguration("zed_resolution"),
            "fps": ParameterValue(LaunchConfiguration("zed_fps"),
                                  value_type=int),
            "depth_mode": LaunchConfiguration("zed_depth_mode"),
            "publish_point_cloud": ParameterValue(
                LaunchConfiguration("zed_point_cloud"), value_type=bool),
            "publish_odom": ParameterValue(
                LaunchConfiguration("zed_odom"), value_type=bool),
            "camera_offset_xyz": ParameterValue(
                LaunchConfiguration("zed_offset_xyz"),
                value_type=List[float]),
        }],
    )

    # ── The belly camera ────────────────────────────────────────────────────
    down_cam = Node(
        package="hydrone_bringup",
        executable="down_cam_usb_node",
        name="down_cam_usb",
        output="screen",
        parameters=[{
            "device": LaunchConfiguration("down_cam_device"),
            "width": ParameterValue(LaunchConfiguration("down_cam_width"),
                                    value_type=int),
            "height": ParameterValue(LaunchConfiguration("down_cam_height"),
                                     value_type=int),
            "fps": ParameterValue(LaunchConfiguration("down_cam_fps"),
                                  value_type=int),
            "calibrated": ParameterValue(
                LaunchConfiguration("down_cam_calibrated"), value_type=bool),
            "nominal_hfov_deg": ParameterValue(
                LaunchConfiguration("down_cam_hfov_deg"), value_type=float),
            "fx": ParameterValue(LaunchConfiguration("down_cam_fx"),
                                 value_type=float),
            "fy": ParameterValue(LaunchConfiguration("down_cam_fy"),
                                 value_type=float),
            "cx": ParameterValue(LaunchConfiguration("down_cam_cx"),
                                 value_type=float),
            "cy": ParameterValue(LaunchConfiguration("down_cam_cy"),
                                 value_type=float),
            "distortion": ParameterValue(
                LaunchConfiguration("down_cam_distortion"),
                value_type=List[float]),
            "mount_xyz": ParameterValue(
                LaunchConfiguration("down_cam_mount_xyz"),
                value_type=List[float]),
            "mount_rpy_deg": ParameterValue(
                LaunchConfiguration("down_cam_mount_rpy_deg"),
                value_type=List[float]),
        }],
    )

    # ── MAVROS against the real FCU ─────────────────────────────────────────
    # Same plugin set as sim. Two files the sim loads are deliberately absent:
    #
    #   mavros_distance_sensor.yaml — puts the distance_sensor plugin in
    #     SUBSCRIBER mode so rangefinder_bridge can feed SITL. On the drone the
    #     plugin PUBLISHES /mavros/distance_sensor/* from the VL53L1X that
    #     ArduPilot read over I2C, which is what the autonomy layer reads in
    #     both worlds.
    #   timeouts.yaml — widens MAVROS's steady-clock command timeouts because
    #     BiguaSim runs below real time. Real time runs at real time.
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "apm_config.yaml"),
            {
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
    )

    # ── AGNOSTIC plumbing — the same node the simulator runs ────────────────
    # /zed/zed_node/odom -> /mavros/vision_pose/pose (VISION_POSITION_ESTIMATE).
    # Only started when the ZED is actually producing odometry; relaying a topic
    # nobody publishes would leave the EKF waiting on vision that never comes,
    # which looks exactly like a bad estimate rather than a missing one.
    vision_odom = Node(
        package="hydrone_localization",
        executable="vision_odom_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("zed_odom")),
    )

    return LaunchDescription(args + [zed, down_cam, mavros, vision_odom])
