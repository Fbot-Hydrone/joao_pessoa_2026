"""
hydrone_bringup/launch/hydrone_mission_sim.launch.py

Run a full mission in simulation: the sim/vehicle bring-up + MAVROS + the
autonomy stack, in one launch. This is the missing "closed-loop in sim" entry
point — hydrone_sim.launch.py and hydrone.launch.py were previously disjoint.

  ros2 launch hydrone_bringup hydrone_mission_sim.launch.py phase:=1

Pieces:
  1. hydrone_sim.launch.py  — BiguaSim bridge + ArduPilot SITL + micro-ROS
     agent + MAVProxy + zed_mimic.
  2. mavros_node            — bridges ArduPilot MAVLink to /mavros/*, the API
     controller_node speaks. Connects to MAVProxy's udp:127.0.0.1:14551 output
     (the QGC output stays free on its own port).
  3. hydrone.launch.py      — vision / controller / nav / mission.

MAVROS is defined here (not in hydrone_sim.launch.py) so the pure sim bring-up
stays GCS-only; on the real drone you'd run zed_wrapper + mavros against the
real FCU instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("hydrone_bringup")
    launch_dir = os.path.join(pkg, "launch")

    phase          = LaunchConfiguration("phase")
    open_hardware  = LaunchConfiguration("open_hardware")
    use_two_drones = LaunchConfiguration("use_two_drones")

    args = [
        DeclareLaunchArgument("phase",          default_value="1"),
        DeclareLaunchArgument("open_hardware",  default_value="false"),
        DeclareLaunchArgument("use_two_drones", default_value="false"),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "hydrone_sim.launch.py")
        )
    )

    # MAVROS <-> ArduPilot over MAVProxy's second output (udp 14551). The empty
    # remote after '@' lets MAVROS auto-learn MAVProxy's source and reply.
    # The apm_pluginlists.yaml denylist is what makes the vision_pose plugin
    # load (mavros_node's built-in default set omits it) — required for
    # VISION_POSITION_ESTIMATE / GPS-denied external nav.
    mavros_share = get_package_share_directory("mavros")
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "apm_config.yaml"),
            {
                "fcu_url": "udp://:14551@",
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
    )

    # Feed ZED visual odometry to ArduPilot as external nav (GPS-denied flight).
    # Pairs with the EK3_SRC*/GPS1_TYPE=0 params in holybro_sitl.parm.
    vision_odom = Node(
        package="hydrone_bringup",
        executable="vision_odom_bridge",
        output="screen",
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "hydrone.launch.py")
        ),
        launch_arguments={
            "phase": phase,
            "open_hardware": open_hardware,
            "use_two_drones": use_two_drones,
        }.items(),
    )

    return LaunchDescription(args + [sim, mavros, vision_odom, autonomy])
