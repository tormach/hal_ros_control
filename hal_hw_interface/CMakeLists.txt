cmake_minimum_required(VERSION 3.0.2)
project(hal_hw_interface)

# C++ 11
add_compile_options(-std=c++11 -Wall)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  ros_control_boilerplate
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  redis_store
)

# Install packages with setup.py
catkin_python_setup()

add_service_files(
  FILES
  SetUInt32.srv
  SetInt32.srv
  SetFloat64.srv
)
generate_messages()

# This package
catkin_package(
  INCLUDE_DIRS
  include

  CATKIN_DEPENDS
  controller_manager
  ros_control_boilerplate
  roscpp
  rospy
  message_runtime
  redis_store
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
  scripts/hal_mgr
  scripts/hal_io
  scripts/hal_offset_mgr

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Install header files
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)
  add_rostest(tests/tests.launch)
  # Check launch file; see http://wiki.ros.org/roslaunch
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
