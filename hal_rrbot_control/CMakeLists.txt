cmake_minimum_required(VERSION 2.8.3)
project(hal_rrbot_control)

find_package(
  catkin REQUIRED COMPONENTS
  hal_hw_interface
  rrbot_control
  )

catkin_package(
  CATKIN_DEPENDS
  hal_hw_interface
  rrbot_control
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY halfiles
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
