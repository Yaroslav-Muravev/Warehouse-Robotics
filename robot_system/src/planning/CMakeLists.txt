cmake_minimum_required(VERSION 3.5)
project(planning)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LaneChange.srv"
  DEPENDENCIES
    std_msgs
    builtin_interfaces
    nav_msgs
    geometry_msgs
)

# ← ONLY ONE of these!
ament_python_install_package(planning_nodes)

ament_package()

