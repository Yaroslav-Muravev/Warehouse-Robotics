cmake_minimum_required(VERSION 3.5)
project(control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(control_nodes)
install(PROGRAMS control_nodes/velocity_controller.py
  DESTINATION lib/control_nodes
)
ament_package()

