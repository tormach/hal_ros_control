cmake_minimum_required(VERSION 3.5)
project(hal_rrbot_control)

find_package(ament_cmake REQUIRED)
find_package(hal_hw_interface REQUIRED)
find_package(ros2_control_demo_example_1 REQUIRED)

install(
  DIRECTORY config halfiles launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
