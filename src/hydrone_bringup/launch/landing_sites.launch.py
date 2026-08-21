"""
hydrone_bringup/launch/landing_sites.launch.py

AUTONOMY layer for the landing-site mission: fly forward, land on each pad the
down camera finds, take off again, keep going.

This is an ALTERNATIVE to hydrone.launch.py, not an addition to it. Both drive
the vehicle, and running them together would put two nodes on
/mavros/setpoint_position/local fighting over the setpoint. Pick one.

  ros2 launch hydrone_bringup landing_sites_sim.launch.py     # sim, everything
  ros2 launch hydrone_bringup landing_sites.launch.py         # autonomy only

Nodes
-----
  pad_detector (forward)  ZED RGB+depth  -> /hydrone/pads/detections
  pad_detector (down)     belly RGB      -> /hydrone/pads/detections
  pad_map                 detections     -> /hydrone/pads/map + RViz markers
  feature_map             ZED point cloud-> /hydrone/map/cloud + coverage
  map_odom_tf             static identity map -> odom (joins TF's two trees)
  pad_mission             down cam + MAVROS -> the flight itself

Like the rest of the autonomy layer this consumes ONLY the agnostic contract
buses (/zed/zed_node/*, /down_cam/*, /mavros/*), so it is identical in sim and
on the real drone — every topic below is the one the real hardware publishes,
and NOTHING here is configured differently for the simulator. landing_sites_sim
adds the sources that produce those buses from BiguaSim and passes no overrides.

It also PRODUCES only what no sensor does: pad detections and the maps built out
of them. It does not back-project depth into a point cloud any more — the ZED
publishes `/zed/zed_node/point_cloud/cloud_registered` itself, so feature_map
consumes that and spends its time on the part that is ours, the accumulation.
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
            "cruise_alt", default_value="2.5",
            description="Cruise altitude, m above takeoff. MUST clear the "
                        "arena's tallest structure (1.5 m) — there is no "
                        "forward obstacle avoidance."),
        DeclareLaunchArgument(
            "forward_step", default_value="1.0",
            description="How far ahead the position setpoint is placed, m. Each "
                        "step is a position error the FCU answers with "
                        "acceleration, so a big step is an aggressive demand — "
                        "which is what flips the vehicle under the sim's "
                        "actuation lag. Keep it small."),
        DeclareLaunchArgument(
            "forward_limit_m", default_value="0.0",
            description="Land and finish after covering this much ground, m. "
                        "0 = fly forward until aborted."),
        DeclareLaunchArgument(
            "rearm_distance_m", default_value="3.0",
            description="Ignore the down camera for this much ground after each "
                        "takeoff, m. Without it the drone lands again on the "
                        "pad it just left, forever."),
        DeclareLaunchArgument(
            "min_confidence", default_value="0.60",
            description="Down-camera confidence that counts as 'a pad is "
                        "below'. Raise it if the drone lands on things that are "
                        "merely blue."),
        DeclareLaunchArgument(
            "auto_start", default_value="true",
            description="Arm and take off as soon as the FCU is ready. Set "
                        "false to hold until /hydrone/mission/start is called."),
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
                        "trees, computed as map_T_base . (odom_T_base)^-1. Set "
                        "false if something else (loop closure, a fiducial) "
                        "takes over that transform."),
        DeclareLaunchArgument(
            "range_topic", default_value="/mavros/distance_sensor/rangefinder",
            description="Downward rangefinder Range topic, used to measure pad "
                        "heights. The SAME topic in sim and on the drone: MAVROS "
                        "publishes it from the VL53L1X on real, and "
                        "rangefinder_bridge mimics that publication in sim. An "
                        "argument only so a bench setup with a different driver "
                        "can point it elsewhere."),
    ]

    cruise_alt = LaunchConfiguration("cruise_alt")
    debug_images = LaunchConfiguration("debug_images")

    # HSV thresholds, shared by both detectors. The library's floor for both
    # colours is S >= 110, but UE's tonemapping/bloom pushes the whole pad
    # toward white (V saturates at ~251-254), so nothing on it reaches that.
    #
    # MEASURED inside the pad's bounding box on a lossless /down_cam frame at
    # 3 m hover (2026-08-18):
    #
    #     blue field        S 37-75, mean 56    ->      0 px survive S >= 110
    #     yellow ring+cross S 38-59, mean 48    ->      0 px survive S >= 110
    #
    # BOTH floors have to come down. Lowering only yellow cannot work: detect()
    # runs findContours on the BLUE mask and iterates over blue contours, so an
    # empty blue mask means zero candidates and the yellow, ring, cross and
    # concentricity checks never execute at all.
    #
    # An earlier grab over the spawn pad put the sim's yellow at S ~ 82-109,
    # well above what it measures at hover — the pad's saturation varies a lot
    # with altitude and local lighting inside the map. S >= 30 covers the whole
    # observed range with margin (it admits every one of the pad's blue and
    # yellow pixels in both grabs) and leaves the discrimination to the
    # structural checks, which is where it belongs: on a confirmed detection
    # ring coverage is 1.0, arms 4, concentricity offset 0.005.
    #
    # SIM VALUES. The library defaults and its tests are unchanged; retune
    # against the real arena lighting per docs/LANDING-SITES.md §3.
    blue_hsv_low = [95, 30, 50]
    yellow_hsv_low = [18, 30, 90]

    # ── Detectors: one per camera, same algorithm, different geometry ───────
    # The forward ZED sees pads at range and has depth to place them with.
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
        }],
    )

    # The belly camera has no depth — it does not need any. It looks almost
    # straight down at a flat floor, so the ground-plane intersection is the
    # more accurate of the two routes at that geometry.
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
        }],
    )

    pad_map = Node(
        package="hydrone_nav",
        executable="pad_map_node",
        name="pad_map",
        output="screen",
        parameters=[{
            "range_topic": LaunchConfiguration("range_topic"),
            # The default 20 s is wall-clock, and BiguaSim runs ~5-8x below
            # real time — so a candidate had ~3 FLIGHT-seconds to be re-seen
            # before being dropped as a false positive (measured 2026-08-18:
            # the spawn pad was sighted once on climb-out and pruned 20 s
            # later while the drone was still crawling to its first search
            # waypoint). Same wall-vs-sim trap as config/timeouts.yaml.
            "provisional_ttl_s": 120.0,
        }],
    )

    # Accumulates the ZED's own point cloud into a persistent voxel map plus a
    # coverage grid. It does NOT create the cloud: cloud_registered comes from
    # the camera (zed_wrapper on the drone, zed_mimic_node in sim), which is
    # where back-projection belongs. Coverage has no counterpart on the ZED, so
    # it lives here.
    feature_map = Node(
        package="hydrone_nav",
        executable="feature_map_node",
        name="feature_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("feature_map")),
    )

    # Joins TF's two disconnected trees.
    #
    # Nobody publishes a link between `map` and `odom`, in sim or on the real
    # drone, so TF comes up as two separate trees:
    #
    #   odom -> base_link -> {zed_camera_link, down_cam_link, ...}   (VO / zed_wrapper)
    #   map  -> map_ned                                              (MAVROS static)
    #
    # Everything this launch file MAPS is published in `map`, because it is
    # built from /mavros/local_position/pose and MAVROS stamps that with
    # frame_id "map" (apm_config.yaml: local_position.frame_id). Everything it
    # SENSES lives under base_link. With no edge between them, RViz can render
    # the map or the vehicle but never both, and `tf2_echo map base_link` fails
    # with "Tf has two or more unconnected trees".
    #
    # This USED to be a static_transform_publisher broadcasting identity, on the
    # theory that `map` and `odom` coincide at boot. They do not, and the error
    # is not small: BiguaSim's odometry is NWU, vision_odom_bridge rotates it
    # +90 deg to ENU before MAVROS sees it, so `map` is ENU while the
    # odom->base_link TF stays NWU. MEASURED 2026-08-20 at two different
    # headings, /mavros/local_position/pose was Rz(+90 deg) x ground truth to
    # 0.012 m and 0.55 deg — RViz drew the vehicle 90 deg away from its own
    # point cloud, origin rotated to match.
    #
    # Identity is not even right in principle: visual_odometry_node fixes its
    # odom origin to identity at the FIRST FRAME, so `odom` is aligned to
    # whatever attitude the drone booted at. The real ZED wrapper does the same,
    # so identity would only ever hold if the vehicle happened to boot facing
    # map-east.
    #
    # map_odom_node computes the transform instead of assuming it:
    #   map_T_odom = map_T_base . (odom_T_base)^-1
    # which is how AMCL and robot_localization do it, is correct for any boot
    # heading, and needs no per-setup constant. The residual it publishes is the
    # drift between the FCU's estimate and the odometry origin — the thing
    # identity was silently declaring to be zero.
    #
    # NOT done by turning on MAVROS local_position.tf.send, which would
    # broadcast map->base_link while the VO already broadcasts odom->base_link.
    # Two parents for one frame is not a tree, and TF rejects it.
    #
    # ON THE REAL DRONE: zed_wrapper publishes map->odom ITSELF when
    # pos_tracking.publish_map_tf is true (its default), and its `map` is the
    # ZED's loop-closed frame, not the FCU's. Two broadcasters, one edge, two
    # different meanings — set publish_map_tf:=false there, or set this argument
    # false and let the ZED own it. See sources_real.launch.py.
    map_odom_tf = Node(
        package="hydrone_bringup",
        executable="map_odom_node",
        name="map_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("map_odom_tf")),
    )

    mission = Node(
        package="hydrone_mission",
        executable="pad_mission_node",
        name="pad_mission",
        output="screen",
        parameters=[{
            "cruise_alt": ParameterValue(cruise_alt, value_type=float),
            "forward_step": ParameterValue(
                LaunchConfiguration("forward_step"), value_type=float),
            "forward_limit_m": ParameterValue(
                LaunchConfiguration("forward_limit_m"), value_type=float),
            "rearm_distance_m": ParameterValue(
                LaunchConfiguration("rearm_distance_m"), value_type=float),
            "min_confidence": ParameterValue(
                LaunchConfiguration("min_confidence"), value_type=float),
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
        # mission,
    ])
