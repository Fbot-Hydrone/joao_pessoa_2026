"""
hydrone_bringup/launch/phase1_zed_detect_real.launch.py

ONE COMMAND to run the ZED-DETECTS mission (phase1_zed_detect.launch.py) on
the real drone:

    ros2 launch hydrone_bringup phase1_zed_detect_real.launch.py

  = sources_real.launch.py       (ZED SDK + belly camera + MAVROS + vision bridge)
  + phase1_zed_detect.launch.py  (the older division: forward ZED detects and
                                  places, belly camera only confirms)

The real-hardware counterpart of phase1_zed_detect_sim.launch.py, exactly the
way phase1_real.launch.py is the real-hardware counterpart of phase1_sim
.launch.py — same two-halves structure, same reason: phase1_zed_detect
.launch.py cannot tell which world it is in, and that is the only thing that
makes a green simulator run say anything about the drone.

Until this file existed, running the zed-detect mission on real hardware
meant typing all eight of phase1_zed_detect.launch.py's argument flips by hand
on top of sources_real.launch.py — easy to get one wrong and fly the OTHER
mission's assumptions (e.g. the belly camera creating map entries again).
This file is that combination, pinned once.

Argument handling
-----------------
Same rule as phase1_real.launch.py: this file declares NOTHING. Both halves
keep their own defaults and both are still settable from the command line,
because launch configurations are inherited by an included description:

    nothing on the command line        -> phase1_zed_detect.launch.py's own
                                          eight-argument flip applies, and
                                          everything ELSE falls through to
                                          phase1.launch.py's defaults
    down_detector_backend:=yolo        -> reaches phase1.launch.py unchanged
    down_yolo_weights:=/path/to.pt        (phase1_zed_detect.launch.py does not
                                          mention either name, so neither is
                                          shadowed)

    ros2 launch hydrone_bringup phase1_zed_detect_real.launch.py \\
        down_detector_backend:=yolo \\
        down_yolo_weights:=/home/<usuario>/hydrone_ws/models/pad_seg_yolo11.pt

Wanting the FORWARD camera on YOLO too is the same idea, one camera over:

    ros2 launch hydrone_bringup phase1_zed_detect_real.launch.py \\
        down_detector_backend:=yolo \\
        down_yolo_weights:=/home/<usuario>/hydrone_ws/models/pad_seg_yolo11.pt \\
        forward_detector_backend:=yolo \\
        forward_yolo_weights:=/home/<usuario>/hydrone_ws/models/pad_seg_yolo11_zed.pt

(a DIFFERENT weights file for the forward camera on purpose — see this
mission's own docs on why the ZED's view of the base needs its own training
data, not the belly camera's.)

BEFORE THE FIRST FLIGHT
-----------------------
Same checklist as phase1_real.launch.py: the belly camera must be calibrated
and its mount measured on the actual airframe. Read that file's docstring and
docs/JETSON-REAL-STACK.md before arming.
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
    #   mission arguments  -> phase1.launch.py declares them (through the
    #                         eight-argument flip phase1_zed_detect.launch.py
    #                         applies on top)
    #
    # Restating either set here would give this file's default the last word
    # and silently kill edits to the file that documents the argument — the
    # exact bug phase1_real.launch.py's docstring measured on 2026-08-22.
    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_real.launch.py")),
    )
    # ONE exception to "declares nothing", and it is not a mission setting:
    # field_mode is a fact about the PAD IN FRONT OF THE CAMERA, which is the
    # one thing a real-hardware wrapper knows and phase1.launch.py cannot —
    # same exemption phase1_real.launch.py carries, forwarded here for the
    # same reason. See phase1.launch.py's own docstring on field_mode.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1_zed_detect.launch.py")),
        launch_arguments={"field_mode": "dark_blue"}.items(),
    )

    return LaunchDescription([sources, autonomy])
