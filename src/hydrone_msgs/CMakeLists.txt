cmake_minimum_required(VERSION 3.8)
project(hydrone_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingBase.msg"
  "msg/MissionState.msg"
  "msg/HumanGesture.msg"
  "msg/QRCode.msg"
  "srv/SetPhase.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
