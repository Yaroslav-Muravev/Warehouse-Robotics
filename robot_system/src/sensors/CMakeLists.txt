cmake_minimum_required(VERSION 3.5)
project(sensors)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(sensors_nodes)
install(PROGRAMS
  sensors_nodes/camera_node.py
  sensors_nodes/odom_node.py
  sensors_nodes/imu_node.py
  sensors_nodes/range_node.py
  DESTINATION lib/sensors_nodes
)

ament_package()

