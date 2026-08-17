"""
hydrone_bringup/launch/landing_sites_sim.launch.py

ONE COMMAND to run the whole landing-site mission in simulation:

    BS_SIM_DIR=<path-to>/bs-drone-competition ./scripts/docker_up.sh   # or
    ros2 launch hydrone_bringup landing_sites_sim.launch.py

  = sources_sim.launch.py   (BiguaSim + SITL + MAVROS + zed_mimic + down_cam)
  + landing_sites.launch.py (detectors + pad map + feature map + mission)

It is the landing-site counterpart of hydrone_mission_sim.launch.py, and it
deliberately does NOT include hydrone.launch.py: the two mission stacks both
publish position setpoints and must not run at once.

Give it ~30 s after start before expecting movement. With GPS disabled the EKF
needs the vision pose and a global origin before it will accept a takeoff — see
docs/DEVELOP-PIPELINES.md. pad_mission waits for exactly that on its own and
logs what it is waiting for.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    # Re-declared so they are settable from THIS file's command line; an
    # argument declared only inside an included file is silently ignored here.
    args = [
        DeclareLaunchArgument("cruise_alt", default_value="2.5"),
        DeclareLaunchArgument("align_alt", default_value="2.0"),
        DeclareLaunchArgument("search_radius", default_value="12.0"),
        DeclareLaunchArgument("search_step", default_value="3.0"),
        DeclareLaunchArgument("max_pads", default_value="0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("debug_images", default_value="true"),
        DeclareLaunchArgument("feature_map", default_value="true"),
        # The real visual odometry flies the vehicle, as it will on the
        # drone. odom_source:=ground_truth swaps in BiguaSim dynamics, but
        # only as a debugging tool — see sources_sim.launch.py.
        DeclareLaunchArgument("odom_source", default_value="vo"),
        # Sim-only VO drift logger; see sources_sim.launch.py.
        DeclareLaunchArgument("odom_error", default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir", default_value=""),
    ]

    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_sim.launch.py")),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
        }.items(),
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "landing_sites.launch.py")),
        launch_arguments={
            "cruise_alt": LaunchConfiguration("cruise_alt"),
            "align_alt": LaunchConfiguration("align_alt"),
            "search_radius": LaunchConfiguration("search_radius"),
            "search_step": LaunchConfiguration("search_step"),
            "max_pads": LaunchConfiguration("max_pads"),
            "auto_start": LaunchConfiguration("auto_start"),
            "debug_images": LaunchConfiguration("debug_images"),
            "feature_map": LaunchConfiguration("feature_map"),
            # SIM: the Range lives where rangefinder_bridge puts it, which is
            # what this MAVROS build's distance_sensor plugin subscribes to.
            "range_topic": "/mavros/rangefinder",
        }.items(),
    )

    return LaunchDescription(args + [sources, autonomy])
