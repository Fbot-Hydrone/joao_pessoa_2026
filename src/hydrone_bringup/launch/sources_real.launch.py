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
  - The downward rangefinder is the range half of the DroneCAN optical-flow
    module on the Pixhawk's CAN bus. ArduPilot reads it natively and MAVROS
    PUBLISHES /mavros/distance_sensor/rangefinder (mavros_distance_sensor_real
    .yaml) — which is why there is no rangefinder_bridge here, and why
    mavros_distance_sensor.yaml (SUBSCRIBER mode) is sim-only.
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
  * DONE 2026-11-23 for the C922 (the C270 was 2026-08-22): the belly camera
    is calibrated, at focus 5, and the numbers are the defaults below. Re-do it if the camera is swapped or the capture
    resolution changes -- fx/fy/cx/cy are in PIXELS and do not survive either.
  * Measure down_cam_mount_xyz / down_cam_mount_rpy_deg on the airframe. The
    defaults are the SIMULATED mount, which is a starting point and not your
    drone.
  * Check fcu_url matches how the Pixhawk is actually wired (Ethernet
    by default).
"""

import glob
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def gripper_port_default() -> str:
    """The ArbotiX-M's FTDI cable, WHICHEVER one is plugged in.

    It used to be pinned to one cable's serial number (AK06RLGW); swapping the
    cable (now AK06RLVB, 2026-11-21) silently broke the gripper. The Orin has
    no other FT232R (Pixhawk on Ethernet, ZED is not a tty), so take the first
    one found, else /dev/ttyUSB0. gripper_port:=... still overrides.
    """
    found = sorted(glob.glob("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_*"))
    return found[0] if found else "/dev/ttyUSB0"


def generate_launch_description():
    bringup_pkg = get_package_share_directory("hydrone_bringup")
    mavros_share = get_package_share_directory("mavros")

    args = [
        # ── Flight controller ───────────────────────────────────────────────
        DeclareLaunchArgument(
            "fcu_url",
            default_value="udp://192.168.144.100:14550@192.168.144.14:14550",
            description="How MAVROS reaches the Pixhawk. Ethernet by default: "
                        "the Pixhawk 6X is 192.168.144.14 (NET_P1 = MAVLink "
                        "UDP server on 14550) and the Orin's enP8p1s0 is "
                        "192.168.144.100 (NetworkManager profile `pixhawk`). "
                        "The Pixhawk only streams once it has heard from us, "
                        "which the remote half of this URL does. Use "
                        "/dev/ttyTHS1:921600 for the UART or "
                        "/dev/ttyACM0:921600 for USB."),
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
            "zed_offset_xyz", default_value="[0.080, 0.0, -0.037]",
            description="Where the ZED body centre sits on the airframe, "
                        "metres in base_link. Default: the t15 support's CAD "
                        "(drones/supports/parts/zed_mount.py)."),
        DeclareLaunchArgument(
            "zed_rpy_deg", default_value="[180.0, 15.0, 0.0]",
            description="ZED mount rotation [roll, pitch, yaw] deg, +pitch = "
                        "lens down. Default: the t15 support, upside down and "
                        "15 deg down. For the t00/t25 blocks use pitch 0/25."),
        DeclareLaunchArgument(
            "zed_tilt_from_imu", default_value="true",
            description="Measure the ZED's roll and pitch with its own IMU at "
                        "start and use those instead of zed_rpy_deg's (yaw "
                        "always comes from zed_rpy_deg). The drone must be "
                        "still and level when the stack starts. A reading "
                        "that is not 1 g, or is more than 10 deg from "
                        "zed_rpy_deg, is refused and zed_rpy_deg is used. "
                        "2026-11-17: the t15 support measured 11.7 deg."),

        # ── Belly camera ────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "down_cam_device",
            # C922 since 2026-11-23 (the C270 was 7F0905D0)
            default_value="/dev/v4l/by-id/"
                          "usb-046d_C922_Pro_Stream_Webcam_D4B65D5F-"
                          "video-index0",
            description="The belly camera, addressed BY SERIAL NUMBER rather "
                        "than as /dev/videoN. Enumeration order is not stable: "
                        "the C270 and the ZED both register V4L2 nodes, and "
                        "after a reboot or a replug the ZED can take video0 — "
                        "which would feed stereo side-by-side frames to the pad "
                        "detector with nothing reporting an error. This path "
                        "is THIS C270 (serial 7F0905D0), so it survives moving "
                        "it to any USB socket on the Orin. It does NOT survive "
                        "swapping the camera: re-read `ls -l /dev/v4l/by-id/` "
                        "if you do."),
        DeclareLaunchArgument("down_cam_width", default_value="640"),
        DeclareLaunchArgument("down_cam_height", default_value="480"),
        DeclareLaunchArgument("down_cam_fps", default_value="15"),
        DeclareLaunchArgument(
            # swept 2026-11-23 at 1.2 m: flat 0-10, soft from 20; 5 is the
            # middle. The calibration below was made at this value.
            "down_cam_focus", default_value="5",
            description="C922 focus_absolute to lock, autofocus off, re-applied "
                        "every start (the camera forgets it at power-up). Find "
                        "it with scripts/v4l2_focus.py. -1 = leave autofocus"),
        DeclareLaunchArgument(
            "down_cam_calibrated", default_value="true",
            description="False publishes a NOMINAL pinhole camera_info from "
                        "nominal_hfov_deg. True uses the measured fx/fy/cx/cy "
                        "and distortion below."),
        DeclareLaunchArgument("down_cam_hfov_deg", default_value="60.0"),

        # C922, measured 2026-11-23 at focus 5: ChArUco 9x11 shown on a
        # screen, 37 views, 0.13 px, 52 deg horizontal. 20 random halves of
        # the views gave fx 644-657 and cx 311-321, so it is stable even
        # without close views. Low error alone does not mean correct; see
        # CALIBRATION.md 4. (The C270 was fx 814.6, cx 299.7, cy 249.4.)
        DeclareLaunchArgument("down_cam_fx", default_value="651.801"),
        DeclareLaunchArgument("down_cam_fy", default_value="654.484"),
        DeclareLaunchArgument("down_cam_cx", default_value="316.031"),
        DeclareLaunchArgument("down_cam_cy", default_value="229.937"),
        DeclareLaunchArgument(
            "down_cam_distortion",
            default_value="[0.062683, -0.217481, -0.001277, -0.000350, "
                          "0.128486]",
            description="OpenCV plumb-bob [k1, k2, p1, p2, k3]. The node had "
                        "this parameter from the start but the launch never "
                        "passed it, so a distortion argument silently did "
                        "nothing."),
        DeclareLaunchArgument(
            "down_cam_mount_xyz", default_value="[0.065, 0.071, -0.12]",
            description="Belly camera position in base_link (ROS FLU: x "
                        "forward, y LEFT), metres. x/y MEASURED on the big "
                        "drone 2026-11-18 (6.5 cm forward, 7.1 cm left); z is "
                        "still the simulated -0.12. phase1_real's "
                        "belly_offset_xy must mirror x/y (it is y RIGHT)."),
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
            "camera_rpy_deg": ParameterValue(
                LaunchConfiguration("zed_rpy_deg"),
                value_type=List[float]),
            "camera_tilt_from_imu": ParameterValue(
                LaunchConfiguration("zed_tilt_from_imu"), value_type=bool),
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
            "focus": ParameterValue(LaunchConfiguration("down_cam_focus"),
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
    #     plugin PUBLISHES /mavros/distance_sensor/rangefinder from the
    #     optical-flow module's rangefinder (mavros_distance_sensor_real.yaml),
    #     which is what the autonomy layer reads in both worlds.
    #   timeouts.yaml — widens MAVROS's steady-clock command timeouts because
    #     BiguaSim runs below real time. Real time runs at real time.
    def _mavros(distance_yaml, condition):
        return Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            condition=condition,
            parameters=[
                os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
                os.path.join(mavros_share, "launch", "apm_config.yaml"),
                os.path.join(bringup_pkg, "config", distance_yaml),
                {
                    "fcu_url": LaunchConfiguration("fcu_url"),
                    "gcs_url": LaunchConfiguration("gcs_url"),
                    "tgt_system": 1,
                    "tgt_component": 1,
                    "fcu_protocol": "v2.0",
                },
            ],
        )

    # surface_offset:=true (default since 2026-11-18): the EKF's height comes
    # from the Orin (RNGFND1, MAVLink) with the bases' steps removed, and the
    # real sensor is RNGFND2. The FCU must match: scripts/fcu_surface_offset.py.
    mavros = _mavros("mavros_distance_sensor_real_floor.yaml",
                     IfCondition(LaunchConfiguration("surface_offset")))
    mavros_plain = _mavros("mavros_distance_sensor_real.yaml",
                           UnlessCondition(
                               LaunchConfiguration("surface_offset")))

    # ── The floor-height bridge (hydrone_localization.surface_offset_node) ──
    surface = Node(
        package="hydrone_localization",
        executable="surface_offset_node",
        name="surface_offset",
        output="screen",
        condition=IfCondition(LaunchConfiguration("surface_offset")),
        parameters=[{
            "start_surface_m": ParameterValue(
                LaunchConfiguration("takeoff_base_height_m"),
                value_type=float),
            "rangefinder_xy": ParameterValue(
                LaunchConfiguration("rangefinder_xy"),
                value_type=List[float]),
            "camera_xy": ParameterValue(
                LaunchConfiguration("surface_camera_xy"),
                value_type=List[float]),
            "arena_size_m": ParameterValue(
                LaunchConfiguration("arena_size_m"), value_type=List[float]),
            "takeoff_base_arena": ParameterValue(
                LaunchConfiguration("takeoff_base_arena"),
                value_type=List[float]),
            "zone_edges_x_m": ParameterValue(
                LaunchConfiguration("zone_edges_x_m"),
                value_type=List[float]),
            "zone_raised": ParameterValue(
                LaunchConfiguration("zone_raised"), value_type=List[bool]),
            "confirm_timeout_s": ParameterValue(
                LaunchConfiguration("surface_confirm_timeout_s"),
                value_type=float),
            "calibrate_datum": ParameterValue(
                LaunchConfiguration("calibrate_datum"), value_type=bool),
        }],
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

    # ── Payload gripper (Phase 2) — Dynamixel AX-12A ───────────────────────
    # Serves /hydrone/gripper/command, the same action ardubridge_node serves
    # in sim. Off by default: Phase 1 flies without it.
    #
    # The servo is reached through an ArbotiX-M running the official ArbotiX
    # "ros" firmware: 115200 baud on the USB side, and it passes servo
    # READ/WRITE packets through to the 1 Mbps Dynamixel bus. It does NOT
    # answer PING (neither for itself, id 253, nor for servos) — scan with
    # register reads. Board swapped and re-flashed with that firmware on
    # 2026-11-21 (servo id 1, AX-12A, 12.1 V). Positions were
    # measured by hand on 2026-10-14 (joint mode, CW 0 / CCW 1023): closed 765,
    # open 254 (opening is clockwise = decreasing). The cam disc was remounted
    # so the whole travel sits inside the AX-12A's 0..300 deg sensing range;
    # mounted across its dead zone, the servo reads 0/1023 and loses itself.
    # The targets keep 10 ticks off each mechanical stop.
    gripper = Node(
        package="hydrone_bringup",
        executable="gripper_dynamixel_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gripper")),
        parameters=[{
            "port": LaunchConfiguration("gripper_port"),
            "baudrate": 115200,
            "open_position": 264,
            "closed_position": 755,
        }],
    )

    surface_args = [
        DeclareLaunchArgument(
            "surface_offset", default_value="true",
            description="Feed the EKF a rangefinder with the bases' steps "
                        "removed (z = height above the arena FLOOR). Needs "
                        "the FCU set by scripts/fcu_surface_offset.py apply; "
                        "false needs it reverted."),
        # Off since 2026-11-23: the datum's bias pushed RNGFND1 to 3 m on the
        # ground and the FCU refused every takeoff. Feed = raw + surface; the
        # EKF's floor offset is measured before the mission instead
        # (orin_flight/floor_z.py -> ground_z).
        DeclareLaunchArgument(
            "calibrate_datum", default_value="false",
            description="Bias the feed until FCU z = height above the floor "
                        "(datum.py). Lies to every FCU rangefinder consumer."),
        DeclareLaunchArgument(
            "takeoff_base_height_m", default_value="0.5",
            description="Height of the takeoff base the drone is set on. The "
                        "floor height starts at this."),
        DeclareLaunchArgument(
            "rangefinder_xy", default_value="[0.14, 0.10]",
            description="Rangefinder position in base_link (FLU), m. Same as "
                        "RNGFND_POS_X/Y on the FCU."),
        DeclareLaunchArgument(
            "surface_camera_xy", default_value="[0.065, 0.071]",
            description="Belly camera x/y in base_link (FLU), m — keep equal "
                        "to down_cam_mount_xyz."),
        DeclareLaunchArgument(
            "arena_size_m", default_value="[0.0, 0.0]",
            description="Arena [Lx, Ly], m; origin at a corner, x along the "
                        "side the zones split. 0 = not given."),
        DeclareLaunchArgument(
            "takeoff_base_arena", default_value="[0.0, 0.0, 0.0]",
            description="Takeoff base centre [x, y] in the arena and the "
                        "heading [deg] the drone is set down with."),
        DeclareLaunchArgument(
            "zone_edges_x_m", default_value="[0.0]",
            description="Zone strip edges along arena x, from 0 to Lx, e.g. "
                        "[0.0, 2.0, 8.0]. Fewer than two = no zones: then "
                        "every raised step is held (never released)."),
        DeclareLaunchArgument(
            "zone_raised", default_value="[false]",
            description="Per zone: may it hold a base or the house? A step in "
                        "a raised zone is believed; elsewhere only the belly "
                        "camera can vouch for it, e.g. [true, true]."),
        DeclareLaunchArgument(
            "surface_confirm_timeout_s", default_value="1.5",
            description="How long a raised step waits for a zone or the "
                        "camera before it is released (vehicle climbs)."),
    ]

    return LaunchDescription(args + surface_args + [
        DeclareLaunchArgument("gripper", default_value="false",
                              description="Start the AX-12A gripper (Phase 2)."),
        DeclareLaunchArgument(
            "gripper_port", default_value=gripper_port_default(),
            description="The ArbotiX-M's FTDI cable. Default: the first "
                        "FT232R found under /dev/serial/by-id (any cable), "
                        "else /dev/ttyUSB0."),
        zed, down_cam, mavros, mavros_plain, surface, vision_odom, gripper])
