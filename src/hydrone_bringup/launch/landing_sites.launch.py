"""
hydrone_bringup/launch/landing_sites.launch.py

AUTONOMY layer for the landing-site mission: find the pads, land on them, take
off again, keep going.

This is an ALTERNATIVE to hydrone.launch.py, not an addition to it. Both drive
the vehicle, and running them together would put two nodes on
/mavros/setpoint_position/local fighting over the setpoint. Pick one.

  ros2 launch hydrone_bringup landing_sites_sim.launch.py     # sim, everything
  ros2 launch hydrone_bringup landing_sites.launch.py         # autonomy only

Nodes
-----
  pad_detector (forward)  ZED RGB+depth  -> /hydrone/pads/detections
  pad_detector (down)     belly RGB      -> /hydrone/pads/detections
  pad_map                 detections     -> /hydrone/pads/map + RViz markers
  feature_map             ZED RGB-D      -> /hydrone/map/features + coverage
  pad_mission             map + MAVROS   -> the flight itself

Like the rest of the autonomy layer this consumes ONLY the agnostic contract
buses (/zed/zed_node/*, /down_cam/*, /mavros/*), so it is identical in sim and
on the real drone. The one thing that differs is `range_topic`, because MAVROS
publishes the rangefinder on the real drone and subscribes to it in sim — see
the argument below.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "cruise_alt", default_value="2.5",
            description="Search altitude, m above takeoff. MUST clear the "
                        "arena's tallest structure (1.5 m) — there is no "
                        "forward obstacle avoidance."),
        DeclareLaunchArgument(
            "align_alt", default_value="2.0",
            description="Altitude the drone settles at over a pad before it "
                        "starts descending, m. Lower than cruise_alt so the pad "
                        "fills more of the down camera; must still clear the "
                        "1.5 m structure if one is nearby."),
        DeclareLaunchArgument(
            "search_radius", default_value="12.0",
            description="Half-width of the search box around the takeoff point, "
                        "m. Bounds the search so the drone cannot roam the "
                        "empty plane forever."),
        DeclareLaunchArgument(
            "search_step", default_value="3.0",
            description="Spacing between spiral legs, m. Keep below the down "
                        "camera's ground footprint (2*alt*tan(FOV/2)) for "
                        "overlapping coverage."),
        DeclareLaunchArgument(
            "max_pads", default_value="0",
            description="Stop and return home after this many landings. "
                        "0 = keep going until the pattern is exhausted."),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Arm and take off as soon as the FCU is ready. Set "
                        "false to hold until /hydrone/mission/start is called."),
        DeclareLaunchArgument(
            "debug_images", default_value="true",
            description="Publish annotated detector views on "
                        "/hydrone/pads/<camera>/debug_image."),
        DeclareLaunchArgument(
            "feature_map", default_value="true",
            description="Run the ZED feature/coverage mapper. Pure observer — "
                        "turn it off to save CPU."),
        DeclareLaunchArgument(
            "range_topic", default_value="/mavros/rangefinder",
            description="Downward rangefinder Range topic, used to measure pad "
                        "heights. SIM: /mavros/rangefinder (what "
                        "rangefinder_bridge feeds MAVROS). REAL: "
                        "/mavros/distance_sensor/rangefinder (what MAVROS "
                        "publishes from the VL53L1X)."),
    ]

    cruise_alt = LaunchConfiguration("cruise_alt")
    debug_images = LaunchConfiguration("debug_images")

    # ── Detectors: one per camera, same algorithm, different geometry ───────
    # The forward ZED sees pads at range and has depth to place them with.
    forward_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_forward",
        output="screen",
        parameters=[{
            "camera": "forward",
            "image_topic": "/zed/zed_node/rgb/image_rect_color",
            "camera_info_topic": "/zed/zed_node/rgb/camera_info",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "optical_frame": "zed_left_camera_optical_frame",
            "publish_debug": ParameterValue(debug_images, value_type=bool),
        }],
    )

    # The belly camera has no depth — it does not need any. It looks almost
    # straight down at a flat floor, so the ground-plane intersection is the
    # more accurate of the two routes at that geometry.
    down_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_down",
        output="screen",
        parameters=[{
            "camera": "down",
            "image_topic": "/down_cam/image_raw",
            "camera_info_topic": "/down_cam/camera_info",
            "depth_topic": "",
            "optical_frame": "down_cam_optical_frame",
            "publish_debug": ParameterValue(debug_images, value_type=bool),
        }],
    )

    pad_map = Node(
        package="hydrone_nav",
        executable="pad_map_node",
        name="pad_map",
        output="screen",
        parameters=[{
            "range_topic": LaunchConfiguration("range_topic"),
        }],
    )

    feature_map = Node(
        package="hydrone_nav",
        executable="feature_map_node",
        name="feature_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("feature_map")),
    )

    mission = Node(
        package="hydrone_mission",
        executable="pad_mission_node",
        name="pad_mission",
        output="screen",
        parameters=[{
            "cruise_alt": ParameterValue(cruise_alt, value_type=float),
            "align_alt": ParameterValue(
                LaunchConfiguration("align_alt"), value_type=float),
            "search_radius": ParameterValue(
                LaunchConfiguration("search_radius"), value_type=float),
            "search_step": ParameterValue(
                LaunchConfiguration("search_step"), value_type=float),
            "max_pads": ParameterValue(
                LaunchConfiguration("max_pads"), value_type=int),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool),
        }],
    )

    return LaunchDescription(args + [
        forward_detector,
        down_detector,
        pad_map,
        feature_map,
        mission,
    ])
