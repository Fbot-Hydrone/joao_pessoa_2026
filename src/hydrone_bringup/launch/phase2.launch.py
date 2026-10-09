"""
hydrone_bringup/launch/phase2.launch.py — Fase 2 (transporte de pacotes).

AUTONOMY ONLY, world-agnostic like phase1.launch.py: it consumes /down_cam/*,
/zed/zed_node/*, /mavros/* and /hydrone/gripper/command, and never knows
whether a simulator or the drone produces them. phase2_sim.launch.py puts the
simulator sources under it.

  hydrone_controller   the only node that commands the FCU
  hydrone_nav          NavigateTo: A* over the octomap
  cloud_filter+octomap the occupancy map — built in the `map` frame here, the
                       frame the Navigator queries (in phase 1 it is `odom`,
                       which the Navigator does not transform; see the memory
                       'Octomap está em odom, não em map')
  map_odom             map -> odom TF
  pad_detector_down    the BASES (YOLO base_pouso) — delivery descent
  kit_detector         the KITS (YOLO 'lipo') — pickup descent + verify
  phase2_mission_node  the strategy

Positions: hydrone_bringup/config/phase2_bases.yaml (arena frame, Figure 7).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

MODELS = PathJoinSubstitution([FindPackageShare("hydrone_vision"), "models"])


def generate_launch_description():
    bringup = get_package_share_directory("hydrone_bringup")
    args = [
        DeclareLaunchArgument("layout_file", default_value=os.path.join(
            bringup, "config", "phase2_bases.yaml"),
            description="Posicoes das bases, frame da arena (Figura 7)."),
        DeclareLaunchArgument("arena_yaw_deg", default_value="-90.0",
                              description="Rotacao arena->map. Sim: -90."),
        DeclareLaunchArgument("cruise_alt", default_value="2.5"),
        DeclareLaunchArgument("mission_budget_s", default_value="600.0"),
        DeclareLaunchArgument("debug", default_value="false"),
    ]
    ignore = [0.75, 0.0, 1.0, 0.22, 0.0, 0.78, 0.16, 1.0]   # airframe in frame

    controller = Node(package="hydrone_controller",
                      executable="controller_node", name="hydrone_controller",
                      output="screen")
    nav = Node(package="hydrone_nav", executable="nav_node", name="hydrone_nav",
               output="screen",
               parameters=[{"plan_bounds": [-5.0, -5.0, 0.3, 5.0, 5.0, 3.0]}])
    cloud_filter = Node(package="hydrone_map", executable="cloud_filter_node",
                        name="cloud_filter", output="screen")
    octomap = Node(
        package="octomap_server", executable="octomap_server_node",
        name="octomap_server", namespace="octomap", output="screen",
        parameters=[{
            "resolution": 0.15, "frame_id": "map", "base_frame_id": "base_link",
            "point_cloud_min_z": -0.5, "point_cloud_max_z": 3.0,
            "sensor_model.max_range": 12.0, "sensor_model.hit": 0.7,
            "sensor_model.miss": 0.4, "filter_ground_plane": False,
            "filter_speckles": True, "occupancy_min_z": 0.25,
            "occupancy_max_z": 2.5, "latch": True}],
        remappings=[("cloud_in", "/hydrone/map/cloud_filtered")])
    map_odom = Node(package="hydrone_localization", executable="map_odom_node",
                    name="map_odom", output="screen")

    def detector(name, camera, weights, classes, out, imgsz, conf):
        return Node(
            package="hydrone_vision", executable="pad_detector_node",
            name=name, output="screen",
            parameters=[{
                "camera": camera,
                "image_topic": "/down_cam/image_raw",
                "camera_info_topic": "/down_cam/camera_info",
                "depth_topic": "",
                "optical_frame": "down_cam_optical_frame",
                "project_position": False,       # positions are KNOWN here
                "out_topic": out,
                "publish_debug": True,
                "ignore_regions": ignore,
                "min_confidence": conf,
                "detector_backend": "yolo",
                "yolo_weights_path": PathJoinSubstitution([MODELS, weights]),
                "yolo_target_classes": classes,
                "yolo_imgsz": imgsz,
                "yolo_conf_threshold": conf,
            }])

    bases = detector("pad_detector_down", "down", "pad_seg_yolo11.pt",
                     ["base_pouso"], "/hydrone/pads/down/detections", 640, 0.30)
    kits = detector("kit_detector", "kit", "lipo_seg_yolo11.pt",
                    ["lipo"], "/hydrone/kits/detections", 960, 0.50)

    mission = Node(
        package="hydrone_mission", executable="phase2_mission_node",
        name="phase2_mission", output="screen",
        parameters=[{
            "layout_file": LaunchConfiguration("layout_file"),
            "arena_yaw_deg": ParameterValue(
                LaunchConfiguration("arena_yaw_deg"), value_type=float),
            "cruise_alt": ParameterValue(
                LaunchConfiguration("cruise_alt"), value_type=float),
            "mission_budget_s": ParameterValue(
                LaunchConfiguration("mission_budget_s"), value_type=float),
        }])

    rqt = Node(package="rqt_image_view", executable="rqt_image_view",
               arguments=["/hydrone/pads/kit/debug_image"],
               condition=IfCondition(LaunchConfiguration("debug")))

    return LaunchDescription(args + [controller, nav, cloud_filter, octomap,
                                     map_odom, bases, kits, mission, rqt])
