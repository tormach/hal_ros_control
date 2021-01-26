cmake_minimum_required(VERSION 2.8.3)
project(machinekit_interfaces)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  stop_event_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})


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
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
