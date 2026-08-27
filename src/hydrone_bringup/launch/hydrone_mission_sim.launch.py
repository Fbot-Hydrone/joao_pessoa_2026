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
        # Sim-only VO drift logger; see sources_sim.launch.py. Re-declared and
        # forwarded so it is settable from this file's command line too.
        DeclareLaunchArgument("odom_error",       default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir",   default_value="/ws/logs"),
    ]

    # Base sim now brings up MAVROS + the vision_odom_bridge itself (the vehicle
    # needs the ZED-VO external nav to hold position with GPS disabled), so we
    # just add the autonomy stack on top.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "hydrone_sim.launch.py")
        ),
        launch_arguments={
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
        }.items(),
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

    return LaunchDescription(args + [sim, autonomy])
