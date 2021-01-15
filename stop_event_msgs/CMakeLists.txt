cmake_minimum_required(VERSION 3.0.2)
project(stop_event_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  message_generation
  std_msgs
  trajectory_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StopEventResult.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetProbeResult.srv
  SetNextProbeMove.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  control_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  control_msgs
  message_runtime
  std_msgs
  trajectory_msgs
)

