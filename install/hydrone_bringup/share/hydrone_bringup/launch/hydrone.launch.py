"""
hydrone_bringup/launch/hydrone.launch.py

Main launch file for the Hydrone competition stack.

Usage examples:
  # Phase 1 with open hardware:
  ros2 launch hydrone_bringup hydrone.launch.py phase:=1 open_hardware:=true

  # Phase 3 with two drones:
  ros2 launch hydrone_bringup hydrone.launch.py phase:=3 use_two_drones:=true

  # Phase 4, commercial drone:
  ros2 launch hydrone_bringup hydrone.launch.py phase:=4 open_hardware:=false
"""

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument, GroupAction
from launch.substitutions              import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions                import Node, PushRosNamespace
from launch_ros.substitutions          import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("phase",          default_value="1",
                              description="Competition phase: 1, 2, 3 or 4"),
        DeclareLaunchArgument("open_hardware",  default_value="false",
                              description="Using open-hardware drone? (2x score)"),
        DeclareLaunchArgument("use_two_drones", default_value="false",
                              description="Phase 3 only: use two drones"),
        DeclareLaunchArgument("debug_vision",   default_value="true",
                              description="Publish annotated debug image"),
        DeclareLaunchArgument("camera_topic",
                              default_value="/zed2/zed_node/rgb/image_rect_color",
                              description="Main camera image topic"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/zed2/zed_node/depth/depth_registered",
                              description="Depth image topic"),
    ]

    phase          = LaunchConfiguration("phase")
    open_hw        = LaunchConfiguration("open_hardware")
    two_drones     = LaunchConfiguration("use_two_drones")
    debug_vision   = LaunchConfiguration("debug_vision")
    camera_topic   = LaunchConfiguration("camera_topic")
    depth_topic    = LaunchConfiguration("depth_topic")

    config_dir = PathJoinSubstitution(
        [FindPackageShare("hydrone_bringup"), "config"])

    # ── Nodes ─────────────────────────────────────────────────────────────

    vision_node = Node(
        package    = "hydrone_vision",
        executable = "vision_node",
        name       = "hydrone_vision",
        output     = "screen",
        parameters = [
            {"camera_topic": camera_topic},
            {"depth_topic":  depth_topic},
            {"debug_image":  debug_vision},
        ],
    )

    controller_node = Node(
        package    = "hydrone_controller",
        executable = "controller_node",
        name       = "hydrone_controller",
        output     = "screen",
        parameters = [
            {"takeoff_height":       1.2},
            {"position_tolerance":   0.12},
            {"setpoint_hz":          20},
        ],
    )

    nav_node = Node(
        package    = "hydrone_nav",
        executable = "nav_node",
        name       = "hydrone_nav",
        output     = "screen",
        parameters = [
            {"cruise_altitude":  1.5},
            {"xy_tolerance":     0.15},
            {"descent_vel":      0.25},
        ],
    )

    mission_node = Node(
        package    = "hydrone_mission",
        executable = "mission_node",
        name       = "hydrone_mission",
        output     = "screen",
        parameters = [
            {"phase":           phase},
            {"open_hardware":   open_hw},
            {"use_two_drones":  two_drones},
        ],
    )

    return LaunchDescription(args + [
        vision_node,
        controller_node,
        nav_node,
        mission_node,
    ])
