"""
hydrone_bringup/launch/phase1_real.launch.py

ONE COMMAND to run the Phase 1 mission on the drone:

    ros2 launch hydrone_bringup phase1_real.launch.py

  = sources_real.launch.py (ZED SDK + belly camera + MAVROS + vision bridge)
  + phase1.launch.py       (detectors + pad map + feature map + the mission)

The real-hardware counterpart of phase1_sim.launch.py, and its exact mirror:
both are pure wrappers that include the SAME UNCHANGED phase1.launch.py and
differ only in which sources file they bring up. That is the whole design —
phase1.launch.py cannot tell which world it is in, which is the only reason a
green simulator run says anything about the drone.

Argument handling
-----------------
This file declares NOTHING — neither the hardware arguments nor the mission's.

A wrapper that re-declares an argument carries its own default, and forwarding
that default overwrites whatever the inner file declared — so editing the file
that documents an argument silently does nothing. That cost a flight session on
2026-08-22 (takeoff_alt edited to 4 m in phase1.launch.py, vehicle climbed to
the wrapper's 1.0 m, nothing warned). Launch configurations are inherited by an
included description, so saying nothing here is both sufficient and correct:

    nothing on the command line   -> phase1.launch.py's own default applies
    takeoff_alt:=1.5 on the CLI   -> reaches phase1.launch.py unchanged

`hydrone_bringup/test/test_launch_arguments.py` fails if the duplication comes
back, for this file against BOTH of its includes. The one cost is that
`ros2 launch -s` on THIS file lists nothing; run it against sources_real for the
hardware arguments and phase1.launch.py for the mission's.

    ros2 launch hydrone_bringup phase1_real.launch.py \\
        takeoff_alt:=1.0 target_bases:=1 down_cam_calibrated:=true \\
        down_cam_fx:=... down_cam_fy:=... down_cam_cx:=... down_cam_cy:=...

BEFORE THE FIRST FLIGHT — read docs/JETSON-REAL-STACK.md
--------------------------------------------------------
  * The belly camera must be calibrated. Uncalibrated, its camera_info is a
    nominal pinhole and the pad positions it produces carry that error straight
    into the landing.
  * The belly camera's mount (down_cam_mount_xyz / _rpy_deg) defaults to the
    SIMULATED mount. Measure yours.
  * The slow-flight limits (WP_SPD, WP_ACC, ATC_RATE_WPY_MAX) live in
    holybro_sitl.parm and are SIM-ONLY. Nothing here applies them to the real
    airframe, and on ArduCopter 4.6.x those parameters have different names
    (WPNAV_SPEED in cm/s, WPNAV_ACCEL, ATC_SLEW_YAW in cdeg/s). Check what the
    FCU is actually running before assuming the vehicle will fly gently.
  * feature_map defaults ON in phase1.launch.py but the ZED point cloud
    defaults OFF here, because on this board it is the most expensive thing in
    the stack. Pass zed_point_cloud:=true AND leave feature_map on if you want
    the coverage map; otherwise pass feature_map:=false so a node is not
    waiting on a topic nobody publishes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    # This file declares NOTHING. Both halves keep their own defaults and both
    # are still settable from this file's command line, because launch
    # configurations are inherited by an included description:
    #
    #   hardware arguments -> sources_real.launch.py declares them
    #   mission arguments  -> phase1.launch.py declares them
    #
    # Restating either set here would give this file's default the last word
    # and silently kill edits to the file that documents the argument. See the
    # module docstring.
    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_real.launch.py")),
    )
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1.launch.py")),
    )

    return LaunchDescription([sources, autonomy])
