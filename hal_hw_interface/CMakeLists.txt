cmake_minimum_required(VERSION 3.5)
project(hal_hw_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Machinekit HAL CMake find/use scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# Find dependencies
find_package(ament_cmake REQUIRED)
# - RT
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(HAL REQUIRED)
# - Python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# - Services
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hal_hw_interface_msgs REQUIRED)

#######################
# HAL RT component build definitions

set(hal_comp_name "hal_control_node")

# Build HAL control_node component
add_library(
  hal_control_node MODULE
  src/hal_control_node.cpp
)

target_compile_definitions(
  hal_control_node PUBLIC
  # Component name, shared btw. controller node & system interface
  CNAME="${hal_comp_name}"
  # HAL RT component
  RTAPI=1
)

# Remove 'lib' prefix from comp name
set_target_properties(hal_control_node PROPERTIES PREFIX "")

# Header files
# - Specify additional header locations
target_include_directories(
  hal_control_node
  PRIVATE
  include/
  ${HAL_INCLUDE_PATH}
)

# Libs
link_directories(${rclcpp_LIBRARY_DIRS})

# Linked libs
target_link_libraries(hal_control_node ${rclcpp_LIBRARIES})

ament_target_dependencies(
  hal_control_node
  controller_manager
  rclcpp
  HAL
)


# Build HAL system hardware plugin
add_library(
  hal_system_interface SHARED
  src/hal_system_interface.cpp
)

# Header files
# - Specify additional header locations
target_include_directories(
  hal_system_interface
  PRIVATE
  include/
  ${HAL_INCLUDE_PATH}
)

target_compile_definitions(
  hal_system_interface PUBLIC
  # Component name, shared btw. controller node & system interface
  CNAME="${hal_comp_name}"
  # HAL RT component
  RTAPI=1
)

ament_target_dependencies(
  hal_system_interface
  pluginlib
  rclcpp
  # rclcpp_lifecycle
  hardware_interface
  HAL
)

pluginlib_export_plugin_description_file(
  hardware_interface hal_hw_interface.xml
)

# Install libraries
install(
  TARGETS hal_control_node hal_system_interface
  LIBRARY
  DESTINATION lib
)

# Install C++ headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Exports
ament_export_include_directories(
  include
)

#######################
# Python

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python scripts
install(PROGRAMS
  scripts/hal_mgr
  scripts/hal_io
  DESTINATION lib/${PROJECT_NAME}  # or bin?
)

#######################
# Install

# Install CMake files
# FIXME:  This belongs in Machinekit HAL!
install(FILES cmake/FindHAL.cmake cmake/UseHALComp.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

# Install config
install(FILES config/fastrtps_disable_shm.xml
  DESTINATION share/${PROJECT_NAME}/config)


#######################
# Tests

if(BUILD_TESTING)

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint()
  # FIXME Broken https://github.com/tholo/pytest-flake8/issues/66
  # find_package(ament_cmake_flake8 REQUIRED)
  # ament_flake8()
  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

#######################
# Finalize package

ament_export_include_directories(
  include
)
ament_export_libraries(
  hal_system_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package(
  CONFIG_EXTRAS cmake/${PROJECT_NAME}-extras.cmake
)
