cmake_minimum_required(VERSION 2.8.3)
project(hal_hw_interface)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ros_control_boilerplate
  controller_manager
  roscpp
)

# This package
catkin_package(
  INCLUDE_DIRS
  include

  CATKIN_DEPENDS
  ros_control_boilerplate
  controller_manager
  roscpp
  rospy

  LIBRARIES
  ${PROJECT_NAME}
)

# Find Machinekit HAL
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(HAL)

# Header files
# - Specify additional header locations
include_directories(
  include/
  ${HAL_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
)
# - Install our headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# Launch file
# - Add library path to help locate HAL comp
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/hal_hw_interface.in.launch
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/hal_hw_interface.launch @ONLY)
# - Install
install(FILES
  launch/hal_hw_interface.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#######################
# HAL component build definitions

# Build HAL component
add_library(
  ${PROJECT_NAME} MODULE
  src/hal_hw_interface.cpp
  src/hal_control_loop.cpp
)

# CFLAGS
target_compile_definitions(
  ${PROJECT_NAME} PRIVATE
  RTAPI=1
  THREAD_FLAVOR_ID=1
  )

# Linked libs
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

# Remove 'lib' prefix from comp name
set_target_properties(
  ${PROJECT_NAME} PROPERTIES
  PREFIX ""
  )
