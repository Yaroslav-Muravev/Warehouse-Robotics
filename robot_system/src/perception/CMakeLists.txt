cmake_minimum_required(VERSION 3.5)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(perception_nodes)
install(PROGRAMS
  perception_nodes/fiducial_detector.py
  perception_nodes/localization_node.py
  DESTINATION lib/perception_nodes
)

ament_package()

