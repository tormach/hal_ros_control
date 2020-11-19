cmake_minimum_required(VERSION 3.0.2)
project(stop_event_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs control_msgs trajectory_msgs std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StopEventResult.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetNextProbeMove.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  control_msgs
  trajectory_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime actionlib_msgs control_msgs trajectory_msgs std_msgs 
)

install(DIRECTORY rules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
