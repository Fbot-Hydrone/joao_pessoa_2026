cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)

# Messages only. The real driver (Livox-SDK2 + this same package name) replaces
# this on the drone; the sim just needs the types to exist.
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
