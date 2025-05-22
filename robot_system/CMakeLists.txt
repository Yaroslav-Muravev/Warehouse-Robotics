cmake_minimum_required(VERSION 3.5)
project(robot_system)

find_package(ament_cmake REQUIRED)

# Install the entire launch/ directory into share/robot_system/launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

