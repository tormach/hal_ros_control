cmake_minimum_required(VERSION 3.0.2)
project(velocity_override_msgs)
add_definitions(-std=c++11)
#* Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

#**************************************
#* Declare ROS messages and services **
#**************************************

# Add IKSolverError to your message files
add_message_files(FILES MoveTypes.msg ServiceNames.msg)

#* Generate servives in the 'srv' folder
add_service_files(FILES MoveTypeService.srv FeedholdStatusService.srv)

#* Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

#**********************************
#* catkin specific configuration **
#**********************************
#* The catkin_package macro generates cmake config files for your package
#* Declare things to be passed to dependent projects
#* INCLUDE_DIRS: uncomment this if you package contains header files
#* LIBRARIES: libraries you create in this project that dependent projects also need
#* CATKIN_DEPENDS: catkin_packages dependent projects also need
#* DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
