"""
hydrone_biguasim/launch/sim.launch.py

Lança tudo de uma vez:
  1. BiguaSim (simulador)
  2. hydrone_biguasim_bridge (adaptador de tópicos)
  3. Stack Hydrone completa (vision, controller, nav, mission)

Uso:
  ros2 launch hydrone_biguasim sim.launch.py phase:=1
  ros2 launch hydrone_biguasim sim.launch.py phase:=1 open_hardware:=true
"""

from launch                   import LaunchDescription
from launch.actions           import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions     import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions       import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Args ──────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("phase",          default_value="1"),
        DeclareLaunchArgument("open_hardware",  default_value="false"),
        DeclareLaunchArgument("use_two_drones", default_value="false"),
        DeclareLaunchArgument("debug_vision",   default_value="true"),
    ]

    phase        = LaunchConfiguration("phase")
    open_hw      = LaunchConfiguration("open_hardware")
    two_drones   = LaunchConfiguration("use_two_drones")
    debug_vision = LaunchConfiguration("debug_vision")

    # ── Paths ─────────────────────────────────────────────────────────────
    biguasim_config = PathJoinSubstitution(
        [FindPackageShare("hydrone_biguasim"), "config", "biguasim_drone.yaml"])

    hydrone_launch = PathJoinSubstitution(
        [FindPackageShare("hydrone_bringup"), "launch", "hydrone.launch.py"])

    # ── 1. BiguaSim node ─────────────────────────────────────────────────
    biguasim_node = Node(
        package    = "biguasim_main",
        executable = "biguasim_node",
        name       = "biguasim_node",
        namespace  = "biguasim",
        output     = "screen",
        emulate_tty= True,
        parameters = [{"params_file": biguasim_config}],
    )

    # ── 2. Bridge (starts 2 s after BiguaSim to let simulator init) ──────
    bridge_node = TimerAction(
        period=2.0,
        actions=[Node(
            package    = "hydrone_biguasim",
            executable = "biguasim_bridge",
            name       = "hydrone_biguasim_bridge",
            output     = "screen",
            parameters = [
                {"agent_name": "drone_id0"},
                {"publish_tf": True},
            ],
        )],
    )

    # ── 3. Hydrone stack (starts 4 s after BiguaSim) ─────────────────────
    # vision_node recebe câmera do bridge; controller recebe pose do bridge
    hydrone_stack = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hydrone_launch),
            launch_arguments={
                "phase":          phase,
                "open_hardware":  open_hw,
                "use_two_drones": two_drones,
                "debug_vision":   debug_vision,
                # Apontar câmera para os tópicos publicados pelo bridge
                "camera_topic": "/zed2/zed_node/rgb/image_rect_color",
                "depth_topic":  "/zed2/zed_node/depth/depth_registered",
            }.items(),
        )],
    )

    return LaunchDescription(args + [
        biguasim_node,
        bridge_node,
        hydrone_stack,
    ])
