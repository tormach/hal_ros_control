cmake_minimum_required(VERSION 3.0.2)
project(machinekit_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp stop_event_msgs)

# Declare a catkin package
catkin_package(INCLUDE_DIRS include/ ${catkin_INCLUDE_DIRS} CATKIN_DEPENDS
               roscpp)

#############
## Testing ##
#############

# TODO
#if(CATKIN_ENABLE_TESTING)
#endif()

#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
