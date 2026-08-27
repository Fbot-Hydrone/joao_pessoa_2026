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

A PURE WRAPPER, on purpose: it passes the autonomy layer NOTHING — no topic is
remapped, no parameter is set differently because this is a simulator, and it
does not even restate the mission's own arguments (see the note by `args`:
doing so shadowed them). landing_sites.launch.py cannot tell
which world it is in, which is the only way a green sim run says anything about
the drone. Everything simulated is a SOURCE: sources_sim.launch.py stands in for
hardware that is not plugged in (BiguaSim's cameras in place of the ZED, its
rangefinder in place of the VL53L1X, SITL in place of the Pixhawk) and publishes
the exact topics that hardware publishes.

If you ever find yourself wanting to add an override here, the fix belongs on
the sources side: make the sim produce what the real drone produces.

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

    # ONLY the arguments this file owns — the ones it forwards to sources_sim.
    # The mission's arguments are deliberately NOT declared here.
    #
    # They used to be, mirrored from landing_sites.launch.py and passed straight
    # down, and that silently broke tuning: a re-declared argument carries THIS
    # file's default, and forwarding it overwrote whatever the inner file
    # declared, so editing a default there did nothing whenever the sim wrapper
    # was the entry point. Found on the phase1 pair 2026-08-22 (a takeoff
    # altitude edited to 4 m flew at the wrapper's 1.0 m) and fixed here too —
    # the two files' defaults happened to be identical, so this changes nothing
    # today and unbreaks every edit from now on.
    #
    # Not declaring them is enough: launch configurations are INHERITED by an
    # included description, so landing_sites.launch.py's own defaults apply, and
    # `cruise_alt:=3.0` on the command line still reaches it. The one cost is
    # that `ros2 launch -s` on THIS file lists only the arguments below.
    # test_launch_arguments.py keeps the duplication from coming back.
    args = [
        # The real visual odometry flies the vehicle, as it will on the
        # drone. odom_source:=ground_truth swaps in BiguaSim dynamics, but
        # only as a debugging tool — see sources_sim.launch.py.
        DeclareLaunchArgument("odom_source", default_value="vo"),
        # Sim-only VO drift logger; see sources_sim.launch.py.
        DeclareLaunchArgument("odom_error", default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir", default_value="/ws/logs"),
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

    # No launch_arguments: every mission argument reaches
    # landing_sites.launch.py by inheritance, keeping its defaults in one file.
    # See the note above `args`.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "landing_sites.launch.py")),
    )

    return LaunchDescription(args + [sources, autonomy])
