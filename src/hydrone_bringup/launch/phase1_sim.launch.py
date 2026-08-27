"""
hydrone_bringup/launch/phase1_sim.launch.py

ONE COMMAND to run the Phase 1 mission in simulation:

    ros2 launch hydrone_bringup phase1_sim.launch.py

  = sources_sim.launch.py (BiguaSim + SITL + MAVROS + zed_mimic + down_cam)
  + phase1.launch.py      (detectors + pad map + feature map + the mission)

It is the Phase 1 counterpart of landing_sites_sim.launch.py and it deliberately
does NOT include it, nor hydrone.launch.py: each of those carries a mission node
of its own, and two nodes publishing position setpoints fight over the vehicle.

A PURE WRAPPER, on purpose: it passes the autonomy layer NOTHING — no topic is
remapped, no parameter is set differently because this is a simulator, and it
does not even restate the mission's own arguments (see the note by `args`: doing
so shadowed them). phase1.launch.py cannot tell which
world it is in, which is the only way a green sim run says anything about the
drone. Everything simulated is a SOURCE: sources_sim.launch.py stands in for
hardware that is not plugged in (BiguaSim's cameras in place of the ZED, its
rangefinder in place of the VL53L1X, SITL in place of the Pixhawk) and publishes
the exact topics that hardware publishes.

If you ever find yourself wanting to add an override here, the fix belongs on
the sources side: make the sim produce what the real drone produces.

Give it ~30 s after start before expecting movement. With GPS disabled the EKF
needs the vision pose and a global origin before it will accept a takeoff — see
docs/DEVELOP-PIPELINES.md. phase1_mission waits for exactly that on its own and
logs what it is waiting for.

Tuning lives in phase1.launch.py — edit a default there, or pass it on the
command line, and it reaches the mission either way:

    ros2 launch hydrone_bringup phase1_sim.launch.py takeoff_alt:=4.0

Read docs/PHASE1-MISSION.md before the first run — in particular §"What to watch
on the first flight", which lists what has and has not been observed.
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

    # ONLY the arguments this file actually owns — the ones it forwards to
    # sources_sim. The mission's arguments are deliberately NOT declared here.
    #
    # They used to be, mirrored from phase1.launch.py and passed straight down,
    # and that silently broke tuning: a re-declared argument carries THIS file's
    # default, and forwarding it overwrote whatever phase1.launch.py declared.
    # Editing a default in phase1.launch.py then did nothing whenever the sim
    # wrapper was the entry point, which is almost always. (Measured 2026-08-22:
    # takeoff_alt edited to 4 in phase1.launch.py, vehicle climbed to the
    # wrapper's 1.0 m.)
    #
    # Not declaring them is enough, and it is better than mirroring them,
    # because launch configurations are INHERITED by an included description:
    #
    #   nothing on the command line -> phase1.launch.py's own default applies
    #   takeoff_alt:=4 on the command line -> reaches phase1.launch.py unchanged
    #
    # so phase1.launch.py is the single place a default lives, and overriding
    # from the command line still works. The one cost is that `ros2 launch -s`
    # on THIS file lists only the arguments below; run it against
    # phase1.launch.py to see the mission's.
    args = [
        # The real visual odometry flies the vehicle, as it will on the drone.
        # odom_source:=ground_truth swaps in BiguaSim dynamics, but only as a
        # debugging tool — see sources_sim.launch.py and LANDING-SITES.md §10.
        # It never demonstrates that anything works.
        DeclareLaunchArgument("odom_source", default_value="vo"),
        # Sim-only VO drift logger; see sources_sim.launch.py. Worth leaving on
        # for this mission in particular: turning on the spot is the motion the
        # VO handles worst, and this is what measures how badly.
        DeclareLaunchArgument("odom_error", default_value="true"),
        DeclareLaunchArgument("odom_error_print", default_value="false"),
        DeclareLaunchArgument("odom_error_dir", default_value="/ws/logs"),
        # Stereo vs depth image for ODOMETRY. See sources_sim.launch.py.
        DeclareLaunchArgument("vo_stereo", default_value="true"),
    ]

    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_sim.launch.py")),
        launch_arguments={
            "odom_source": LaunchConfiguration("odom_source"),
            "odom_error": LaunchConfiguration("odom_error"),
            "odom_error_print": LaunchConfiguration("odom_error_print"),
            "odom_error_dir": LaunchConfiguration("odom_error_dir"),
            "vo_stereo": LaunchConfiguration("vo_stereo"),
        }.items(),
    )

    # No launch_arguments at all: every mission argument reaches
    # phase1.launch.py by inheritance, keeping its defaults in one file. See the
    # note above `args`.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1.launch.py")),
    )

    return LaunchDescription(args + [sources, autonomy])
