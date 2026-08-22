"""
hydrone_bringup/launch/phase1.launch.py

AUTONOMY layer for the Phase 1 mission: take off, turn on the spot until a
landing base is in the map, fly over it, confirm it on the belly camera, land,
repeat, then come home to the base we started on.

This is an ALTERNATIVE to hydrone.launch.py and to landing_sites.launch.py, not
an addition to either. All three drive the vehicle, and running two of them puts
two nodes on /mavros/setpoint_position/local fighting over the setpoint. Pick
one.

  ros2 launch hydrone_bringup phase1_sim.launch.py     # sim, everything
  ros2 launch hydrone_bringup phase1.launch.py         # autonomy only

Nodes
-----
  pad_detector (forward)  ZED RGB+depth  -> /hydrone/pads/detections
  pad_detector (down)     belly RGB      -> /hydrone/pads/detections
  pad_map                 detections     -> /hydrone/pads/map + RViz markers
  feature_map             ZED point cloud-> /hydrone/map/cloud + coverage
  map_odom_tf             measured map -> odom (joins TF's two trees)
  phase1_mission          map + MAVROS   -> the flight itself

How this differs from landing_sites.launch.py
---------------------------------------------
The sensing half is identical — same two detectors, same map, same tuning — and
that is deliberate: the detector is the part that is partially validated and it
should not be forked. What changes is above it.

  * `phase1_mission_node` replaces `pad_mission_node`. The old one flies +X in
    steps and lands on whatever the belly camera happens to see; this one never
    translates without a target and searches by turning in place.
  * **Both cameras now feed the decision.** The old mission threw away every
    forward-camera detection. This one takes its leads from the MAP, which fuses
    both, so the ZED identifies bases across the arena and the belly camera
    validates them from overhead.
  * `pad_map` maps nothing until the vehicle first arms, and the base the drone
    starts on is REGISTERED rather than detected — see docs/PHASE1-MISSION.md.
  * Altitude is 1 m, not 2.5 m. This is test code and a fall from 1 m is cheap.
    Note that it therefore does NOT clear the 1.5 m structure the landing_sites
    cruise altitude was chosen for: this launch assumes the Phase 1 arena is
    clear, which is the arena being flown.

Like the rest of the autonomy layer this consumes ONLY the agnostic contract
buses (/zed/zed_node/*, /down_cam/*, /mavros/*), so it is identical in sim and
on the real drone. phase1_sim adds the sources that produce those buses from
BiguaSim and passes no overrides.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "takeoff_alt", default_value="1.0",
            description="Altitude for everything: takeoff, turning, travelling "
                        "and the confirmation hover, m above the top of the "
                        "base the drone starts on. Low on purpose — this is "
                        "test code and a fall from 1 m is cheap. At 1 m the "
                        "REAL belly camera (640x480, measured fx 814.6, so "
                        "about 43 deg horizontal) covers roughly 0.8 x 0.6 m "
                        "of floor — NARROWER than the 90 deg simulated one "
                        "this text used to describe, and narrower than a 1 m "
                        "base. The base will overfill the frame at this "
                        "height; the detector needs the whole ring to score "
                        "well, so if confirmation fails at 1 m that is the "
                        "first thing to raise. There is NO obstacle "
                        "avoidance: raise this only for an arena you know is "
                        "clear at the new height."),
        DeclareLaunchArgument(
            "target_bases", default_value="1",
            description="How many landing sites to visit before returning to "
                        "the takeoff base. The takeoff base is not one of "
                        "them. ONE while the mission has never been flown: the "
                        "first thing worth knowing is whether a single "
                        "find-confirm-land-return cycle closes, and a second "
                        "base only adds a leg on a position estimate that has "
                        "already been through a landing and a takeoff. Raise "
                        "it to 2 (the competition number) once one cycle has "
                        "been watched end to end."),
        DeclareLaunchArgument(
            "rotation_step_deg", default_value="45.0",
            description="Size of each search turn, degrees clockwise."),
        DeclareLaunchArgument(
            "max_rotations", default_value="8",
            description="Turns to make before giving up and running the "
                        "fallback. 8 x 45 deg is one full circle; past that "
                        "the drone is re-examining scenery it already "
                        "rejected."),
        DeclareLaunchArgument(
            "settle_s", default_value="10.0",
            description="Time held stationary after each turn before the map "
                        "is believed, s. Detections taken while yaw is slewing "
                        "are projected through a moving estimate and land in "
                        "the map metres out. Keep this short — it is there to "
                        "let the estimate stop, not to loiter."),
        DeclareLaunchArgument(
            "confirm_detections", default_value="3",
            description="Belly-camera looks above confirm_confidence needed "
                        "before committing to a landing. One frame can be a "
                        "glint on something blue."),
        DeclareLaunchArgument(
            "confirm_confidence", default_value="0.20",
            description="Confidence that counts as a look. Raise it if the "
                        "drone lands on things that are merely blue."),
        DeclareLaunchArgument(
            "confirm_timeout_s", default_value="25.0",
            description="How long to hover over a candidate before declaring "
                        "it is not a landing site, blacklisting it and "
                        "resuming the search. Wall-clock, and BiguaSim runs "
                        "well below real time — see config/timeouts.yaml for "
                        "the same trap."),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Arm and take off as soon as the FCU is ready. Set "
                        "false to hold until /hydrone/mission/start is "
                        "called."),
        DeclareLaunchArgument(
            "debug_images", default_value="true",
            description="Publish annotated detector views on "
                        "/hydrone/pads/<camera>/debug_image."),
        DeclareLaunchArgument(
            "feature_map", default_value="true",
            description="Run the world/coverage mapper over the ZED's point "
                        "cloud. Pure observer — turn it off to save CPU."),
        DeclareLaunchArgument(
            "map_odom_tf", default_value="true",
            description="Publish the measured map -> odom that joins TF's two "
                        "trees. Set false if something else takes over that "
                        "transform — on the real drone, zed_wrapper does "
                        "unless publish_map_tf is false."),
        DeclareLaunchArgument(
            "require_armed", default_value="true",
            description="Map nothing until the vehicle first arms. On the "
                        "ground both cameras are looking at the base the drone "
                        "is standing on, from the grazing angles that detect "
                        "it best; mapping then opens the run with a candidate "
                        "the mission has to fly out and rule out."),
        DeclareLaunchArgument(
            "range_topic", default_value="/mavros/distance_sensor/rangefinder",
            description="Downward rangefinder Range topic, used to measure pad "
                        "heights. The SAME topic in sim and on the drone: "
                        "MAVROS publishes it from the VL53L1X on real, and "
                        "rangefinder_bridge mimics that publication in sim."),
        DeclareLaunchArgument(
            "ground_z", default_value="0.0",
            description="Height of the arena FLOOR above the takeoff plane, m. "
                        "The takeoff plane is the top of the base the drone "
                        "starts on, so if that base is raised this is negative "
                        "— every belly-camera projection intersects this plane "
                        "and is biased by getting it wrong. 0.0 is correct "
                        "while everything in the arena is at ground level; set "
                        "it to minus the start base's height once that stops "
                        "being true."),
    ]

    takeoff_alt = LaunchConfiguration("takeoff_alt")
    debug_images = LaunchConfiguration("debug_images")
    ground_z = LaunchConfiguration("ground_z")

    # HSV thresholds, shared by both detectors. Carried over UNCHANGED from
    # landing_sites.launch.py — this mission changes what is done with
    # detections, never how they are made. The measurement behind these numbers
    # (blue S 37-75, yellow S 38-59 on a lossless /down_cam frame at 3 m hover,
    # 2026-08-18, against a library floor of S >= 110 that admitted zero pixels
    # of either) is written out in full there and in docs/LANDING-SITES.md §3.
    #
    # SIM VALUES. Retune against the real arena's light before flying it.
    blue_hsv_low = [95, 30, 50]
    yellow_hsv_low = [18, 30, 90]

    # ── Detectors: one per camera, same algorithm, different geometry ───────
    # The forward ZED sees pads across the arena and has depth to place them
    # with. In this mission it is the IDENTIFIER: everything the search finds,
    # it finds here first.
    forward_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_forward",
        output="screen",
        parameters=[{
            "camera": "forward",
            "image_topic": "/zed/zed_node/rgb/image_rect_color",
            "camera_info_topic": "/zed/zed_node/rgb/camera_info",
            "depth_topic": "/zed/zed_node/depth/depth_registered",
            "optical_frame": "zed_left_camera_optical_frame",
            "publish_debug": ParameterValue(debug_images, value_type=bool),
            "blue_hsv_low": blue_hsv_low,
            "yellow_hsv_low": yellow_hsv_low,
            "ground_z": ParameterValue(ground_z, value_type=float),
        }],
    )

    # The belly camera has no depth — it does not need any. It looks almost
    # straight down at a flat floor, so the ground-plane intersection is the
    # more accurate of the two routes at that geometry. In this mission it is
    # the VALIDATOR: nothing is landed on until this camera has seen it from
    # directly above, where the ring and cross are hundreds of pixels across.
    down_detector = Node(
        package="hydrone_vision",
        executable="pad_detector_node",
        name="pad_detector_down",
        output="screen",
        parameters=[{
            "camera": "down",
            "image_topic": "/down_cam/image_raw",
            "camera_info_topic": "/down_cam/camera_info",
            "depth_topic": "",
            "optical_frame": "down_cam_optical_frame",
            "publish_debug": ParameterValue(debug_images, value_type=bool),
            "blue_hsv_low": blue_hsv_low,
            "yellow_hsv_low": yellow_hsv_low,
            "ground_z": ParameterValue(ground_z, value_type=float),
        }],
    )

    pad_map = Node(
        package="hydrone_nav",
        executable="pad_map_node",
        name="pad_map",
        output="screen",
        parameters=[{
            "range_topic": LaunchConfiguration("range_topic"),
            "require_armed": ParameterValue(
                LaunchConfiguration("require_armed"), value_type=bool),
            # The default 20 s is wall-clock, and BiguaSim runs ~5-8x below
            # real time — so a candidate had ~3 FLIGHT-seconds to be re-seen
            # before being dropped as a false positive. It matters more here
            # than it did for the forward run: this mission's whole search is
            # "turn away, turn back", and a base sighted on one heading has to
            # survive in the map until the drone has finished looking at the
            # other seven.
            "provisional_ttl_s": 120.0,
        }],
    )

    # Accumulates the ZED's own point cloud into a persistent voxel map plus a
    # coverage grid. Pure observer; nothing in this mission reads it.
    feature_map = Node(
        package="hydrone_nav",
        executable="feature_map_node",
        name="feature_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("feature_map")),
    )

    # Joins TF's two disconnected trees, by MEASURING map -> odom rather than
    # assuming identity: map_T_odom = map_T_base . (odom_T_base)^-1. The full
    # argument, including the 2026-08-20 measurement that showed identity to be
    # wrong by 90 degrees, is in landing_sites.launch.py — it is the same node
    # doing the same job here.
    map_odom_tf = Node(
        package="hydrone_bringup",
        executable="map_odom_node",
        name="map_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("map_odom_tf")),
    )

    mission = Node(
        package="hydrone_mission",
        executable="phase1_mission_node",
        name="phase1_mission",
        output="screen",
        parameters=[{
            "takeoff_alt": ParameterValue(takeoff_alt, value_type=float),
            "target_bases": ParameterValue(
                LaunchConfiguration("target_bases"), value_type=int),
            "rotation_step_deg": ParameterValue(
                LaunchConfiguration("rotation_step_deg"), value_type=float),
            "max_rotations": ParameterValue(
                LaunchConfiguration("max_rotations"), value_type=int),
            "settle_s": ParameterValue(
                LaunchConfiguration("settle_s"), value_type=float),
            "confirm_detections": ParameterValue(
                LaunchConfiguration("confirm_detections"), value_type=int),
            "confirm_confidence": ParameterValue(
                LaunchConfiguration("confirm_confidence"), value_type=float),
            "confirm_timeout_s": ParameterValue(
                LaunchConfiguration("confirm_timeout_s"), value_type=float),
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool),
        }],
    )

    return LaunchDescription(args + [
        forward_detector,
        down_detector,
        pad_map,
        feature_map,
        map_odom_tf,
        mission,
    ])
