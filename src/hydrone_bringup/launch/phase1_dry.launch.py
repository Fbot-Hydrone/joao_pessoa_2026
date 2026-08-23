"""
hydrone_bringup/launch/phase1_dry.launch.py

THE PHASE 1 MISSION, REHEARSED ON FOOT. NOTHING IS SENT TO THE FLIGHT
CONTROLLER.

    ros2 launch hydrone_bringup phase1_dry.launch.py
    scripts/jetson_up.sh --dry                     # on the drone

Everything phase1_real.launch.py starts, starts here: both cameras, MAVROS, the
two detectors, the pad map and its markers, the feature map, the TF join, and
phase1_mission_node itself. The one difference is that the mission commands
nothing. A person carries the drone and is the actuator — the mission prints

    >>> RAISE the drone to 1.50 m above what it is standing on — 0.42 m of 1.50 m
    >>> TURN the drone RIGHT (clockwise) 41 deg, on the spot — to heading 315 deg
    >>> CARRY the drone 2.31 m to (2.41, -0.88) — 19 deg to the left of where
        the nose points. Keep it at 1.50 m and do not turn it.
    >>> HOLD THE DRONE OVER THE PAD, lens down — 2/3 good looks, 18 s before
        this candidate is rejected.
    >>> PUT THE DRONE DOWN on the pad and let go

and every transition still waits on the MEASURED pose, so the rehearsal only
advances when the drone has physically been moved. What you are debugging is
therefore the real thing: the real detector against the real pads, the real
pad_map fusing real projections through the real position estimate, and the
real state machine deciding on all of it.

WHAT `dry_run` GUARANTEES, AND WHAT IT DOES NOT
-----------------------------------------------
phase1_mission_node in dry run never CREATES its arm, mode or takeoff service
clients, nor its setpoint publisher. That is structural rather than a flag
checked at each call site: there is no object for a bug, or for code added
later, to send through. Nothing this stack runs asks the vehicle to arm, change
mode, take off, land, or go anywhere.

It is not a guarantee about the vehicle. MAVROS is started normally and still
offers /mavros/cmd/arming, so a `ros2 service call` typed in another terminal,
or any other node someone starts, can still arm — and so can the transmitter,
whose receiver is wired to the Pixhawk and never passed through MAVROS anyway.
THE THING THAT KEEPS HANDS SAFE IS AT THE FLIGHT CONTROLLER: set an arming
check the vehicle cannot pass before a rehearsal, and take the props off.

THE ONE THING THAT IS STILL SENT (and why the Pixhawk beeps)
------------------------------------------------------------
`dry_run` silences the MISSION. It does not silence `vision_odom_bridge`, which
runs here as in every other launch and publishes two things the FCU acts on:
the ZED's pose as /mavros/vision_pose/pose, and — at 1 Hz until the FCU
confirms home — a fixed global origin on
/mavros/global_position/set_gp_origin. ArduPilot sets its EKF origin and home
from that, and announces it. Start and stop the stack a few times in a row and
the FCU tones on every origin/home set and every GCS-heartbeat-lost, which
sounds alarming and is not.

It is deliberate and it has to stay: EVERY transition in the rehearsal is
measured from /mavros/local_position/pose, and without vision reaching the EKF
there is no local position, so the mission never leaves WAIT_FCU. Neither
publication can arm anything.

WHY THIS IS A WRAPPER THAT FORWARDS
-----------------------------------
Every other wrapper here declares and forwards NOTHING, and
test_launch_arguments.py enforces it — a wrapper's default silently overwrites
the default of the file that documents the argument, which cost a flight
session on 2026-08-22. This file is the deliberate exception: overriding its
includes is the entire reason it exists, and neither value below is tuning that
anyone would edit elsewhere and expect to win. `dry_run` is pinned by
`test_the_dry_launch_really_is_dry`, which is stricter than "does not forward":
it asserts the value actually arrives.

  dry_run=true          phase1.launch.py       -> the mission node
  zed_point_cloud=true  sources_real.launch.py -> the ZED SDK

The point cloud is ON here and OFF in phase1_real, which is the one place the
two disagree. feature_map_node has no other geometry source, and a rehearsal is
exactly when you want the coverage map and its markers to look at; the CPU it
costs on the Tegra X1 is not competing with a flight. It is a hard override
rather than a default — launch offers no way for a wrapper to suggest a value
without winning — so `zed_point_cloud:=false` on this file's command line does
nothing. Run phase1.launch.py directly if you need that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("hydrone_bringup"), "launch")

    sources = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "sources_real.launch.py")),
        launch_arguments={
            # feature_map_node defaults on in phase1.launch.py and has nothing
            # to read without this. Off in phase1_real because a flight cannot
            # spare the Tegra X1 cycles; a rehearsal can.
            "zed_point_cloud": "true",
        }.items(),
    )

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "phase1.launch.py")),
        launch_arguments={
            # Same reason phase1_real sets it: a fact about the PAD IN FRONT OF
            # THE CAMERA, which the hardware wrapper knows and the shared
            # autonomy file cannot.
            "field_mode": "dark_blue",
            # The whole point of this file.
            "dry_run": "true",
        }.items(),
    )

    banner = LogInfo(msg=(
        "\n"
        "════════════════════════════════════════════════════════════════\n"
        "  PHASE 1 DRY RUN — the mission sends NOTHING to the FCU.\n"
        "  No arm, no mode change, no takeoff, no setpoint: the mission\n"
        "  node does not even create the clients. You are the actuator —\n"
        "  follow the >>> lines.\n"
        "  This stops the STACK commanding the vehicle. It does not stop\n"
        "  the vehicle arming: set an arming check it cannot pass, and\n"
        "  take the props off.\n"
        "════════════════════════════════════════════════════════════════"))

    return LaunchDescription([banner, sources, autonomy])
