cmake_minimum_required(VERSION 2.8.3)
project(hal_hw_interface)

# C++ 11
add_compile_options(-std=c++11 -Wall)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  ros_control_boilerplate
  roscpp
  rospy
)

# This package
catkin_package(
  INCLUDE_DIRS
  include

  CATKIN_DEPENDS
  controller_manager
  ros_control_boilerplate
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

#######################
# HAL component build definitions

# Build HAL component
add_library(
  ${PROJECT_NAME} MODULE
  src/hal_control_loop.cpp
  src/${PROJECT_NAME}.cpp
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

#######################
# Install

# Install libraries
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

# Install python executables
catkin_install_python(
  PROGRAMS
  src/hal_mgr.py
  src/hal_io

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install launch files
install(
  FILES launch/${PROJECT_NAME}.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
