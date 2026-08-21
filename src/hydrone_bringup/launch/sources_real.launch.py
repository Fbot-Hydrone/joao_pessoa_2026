"""
hydrone_bringup/launch/sources_real.launch.py

REAL SOURCES layer (STUB — hardware drivers not integrated yet).

This is the real-drone counterpart of sources_sim.launch.py. It MUST produce the
SAME agnostic output contract so the autonomy stack (hydrone.launch.py) runs
unchanged when the flag flips:

  OUTPUT CONTRACT (must match sources_sim.launch.py 1:1):
    /zed/zed_node/rgb/image_rect_color, /zed/zed_node/rgb/camera_info,
    /zed/zed_node/depth/depth_registered, /zed/zed_node/depth/camera_info,
    /zed/zed_node/point_cloud/cloud_registered,
    /zed/zed_node/imu/data, /zed/zed_node/odom,
    /down_cam/image_raw, /down_cam/camera_info,
    /mavros/*  (incl. /mavros/vision_pose/pose, /mavros/distance_sensor/rangefinder)
    TF: base_link -> zed_camera_link -> zed_left_camera_frame
                                     -> zed_left_camera_optical_frame
        base_link -> down_cam_link   -> down_cam_optical_frame

Difference vs sim (only the SOURCES change, never the autonomy):
  - NO SITL, NO ardubridge, NO zed_mimic, NO down_cam_mimic,
    NO visual_odometry_node, NO rangefinder_bridge.
  - zed_wrapper (ZED SDK on the Jetson) publishes /zed/zed_node/* INCLUDING
    /zed/zed_node/odom natively (its own VIO) — that is why visual_odometry_node
    is sim-only — AND /zed/zed_node/point_cloud/cloud_registered, which is why
    zed_mimic_node grew a point cloud: feature_map_node consumes the camera's
    cloud in both worlds instead of back-projecting depth itself.
  - The VL53L1X rangefinder is wired over I2C into the Pixhawk; ArduPilot reads
    it natively and MAVROS PUBLISHES /mavros/distance_sensor/* (publisher mode) —
    that is why rangefinder_bridge is sim-only.
  - The belly camera is a real USB/CSI camera. Its driver publishes
    /down_cam/image_raw + /down_cam/camera_info directly, replacing
    down_cam_mimic_node — plus a static_transform_publisher for the mount,
    which the mimic was providing in sim.
  - vision_odom_bridge is AGNOSTIC and runs here too (same node as sim).

NOTHING ABOVE THE SOURCES IS TOLD APART. The autonomy layer takes no sim/real
argument at all — landing_sites.launch.py runs with identical defaults in both
worlds. The rangefinder used to be the exception (MAVROS PUBLISHES the
natively-read VL53L1X on /mavros/distance_sensor/rangefinder here, while in sim
rangefinder_bridge FEEDS MAVROS on /mavros/rangefinder); the bridge now mimics
MAVROS's real publication on the real topic name too, so both worlds read
/mavros/distance_sensor/rangefinder and the override is gone.

TWO THINGS TO SET ON zed_wrapper HERE, both of which it gets wrong for us by
default:
  - point cloud ON (`point_cloud_freq` > 0, `depth.depth_mode` not NONE) —
    feature_map_node has no other source of geometry now.
  - `pos_tracking.publish_map_tf:=false`. The wrapper broadcasts map->odom by
    default, where `map` means the ZED's loop-closed frame. In this stack `map`
    is the FCU's frame and map_odom_node owns that edge (it computes
    map_T_base . (odom_T_base)^-1). Two broadcasters of one edge, meaning two
    different things, is a corrupt TF tree. Alternatively launch the autonomy
    with map_odom_tf:=false and let the ZED own it — but then `map` is no longer
    where the pad map lives, so prefer disabling it on the wrapper.

The nodes below are intentionally left COMMENTED (stub) until the ZED SDK / FCU
are integrated: a launch pointing at drivers that don't exist yet would fail and
teach us nothing. This file is the scaffold + checklist — uncomment and fill in
the real device parameters when the hardware arrives, keeping the output topics
above exactly as-is.
"""

from launch import LaunchDescription
from launch.actions import LogInfo
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


def generate_launch_description():
    # ── REAL SOURCES (stub — uncomment when hardware/drivers are integrated) ──
    #
    # mavros_share = get_package_share_directory("mavros")
    # bringup_pkg  = get_package_share_directory("hydrone_bringup")
    #
    # 1) ZED SDK wrapper — publishes /zed/zed_node/* incl. /zed/zed_node/odom (VIO).
    #    Replaces zed_mimic_node AND visual_odometry_node from the sim side.
    # zed_wrapper = Node(
    #     package="zed_wrapper", executable="zed_wrapper", output="screen",
    #     parameters=[{"general.camera_model": "zed2i",
    #                  "pos_tracking.pos_tracking_enabled": True,
    #                  # map->odom is map_odom_node's edge here; see above.
    #                  "pos_tracking.publish_map_tf": False,
    #                  # feature_map_node's only input.
    #                  "point_cloud_freq": 10.0}],
    #     # Ensure it publishes under /zed/zed_node/* (namespace/remap as needed).
    # )
    #
    # 2) MAVROS against the real FCU. Same plugins as sim, but:
    #    - fcu_url points at the physical Pixhawk (serial or companion UDP), and
    #    - NO mavros_distance_sensor.yaml: the distance_sensor plugin PUBLISHES
    #      /mavros/distance_sensor/* from the natively-read VL53L1X (I2C).
    # mavros = Node(
    #     package="mavros", executable="mavros_node", output="screen",
    #     parameters=[
    #         os.path.join(mavros_share, "launch", "apm_pluginlists.yaml"),
    #         os.path.join(mavros_share, "launch", "apm_config.yaml"),
    #         {"fcu_url": "/dev/ttyACM0:921600",   # <- real FCU (placeholder)
    #          "gcs_url": "", "tgt_system": 1, "tgt_component": 1,
    #          "fcu_protocol": "v2.0"},
    #     ],
    # )
    #
    # 3) Belly camera for the landing-pad detector. Any driver will do as long
    #    as it publishes /down_cam/image_raw (bgr8/rgb8/mono8) and a
    #    /down_cam/camera_info with real intrinsics — the projection is only as
    #    good as fx/fy/cx/cy, so calibrate it, do not assume the nominal FOV.
    # down_cam = Node(
    #     package="usb_cam", executable="usb_cam_node_exe", output="screen",
    #     parameters=[{"video_device": "/dev/video0",
    #                  "image_width": 640, "image_height": 480,
    #                  "pixel_format": "yuyv", "frame_id": "down_cam_optical_frame",
    #                  "camera_info_url": "file:///.../down_cam.yaml"}],
    #     remappings=[("image_raw", "/down_cam/image_raw"),
    #                 ("camera_info", "/down_cam/camera_info")],
    # )
    #
    # 4) The belly camera's mount, in place of down_cam_mimic's static TFs.
    #    Measure the offset on the airframe. The rotation below is the standard
    #    optical convention for a lens pointing straight down (x y z yaw pitch
    #    roll, radians): base_link -> down_cam_optical_frame in one hop.
    # down_cam_tf = Node(
    #     package="tf2_ros", executable="static_transform_publisher",
    #     arguments=["0", "0", "-0.12", "-1.5707963", "0", "-3.1415927",
    #                "base_link", "down_cam_optical_frame"],
    # )
    #
    # 5) AGNOSTIC plumbing — identical node to sim: /zed/zed_node/odom ->
    #    /mavros/vision_pose/pose.
    # vision_odom = Node(
    #     package="hydrone_bringup", executable="vision_odom_bridge", output="screen",
    # )
    #
    # return LaunchDescription([zed_wrapper, mavros, down_cam, down_cam_tf,
    #                           vision_odom])

    return LaunchDescription([
        LogInfo(msg=("[sources_real] STUB — real drivers not integrated. "
                     "Uncomment zed_wrapper + mavros(FCU) + down_cam + its "
                     "static TF + vision_odom_bridge; they must publish "
                     "/zed/zed_node/* (incl. point_cloud/cloud_registered), "
                     "/down_cam/* and /mavros/* exactly as sources_sim does. "
                     "No autonomy changes, and no autonomy arguments.")),
    ])
