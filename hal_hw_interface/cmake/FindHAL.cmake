###############################################################################
#
# CMake script for finding HAL headers
#
# Output variables:
#
# - HAL_FOUND:  Boolean that indicates if the package was found
# - HAL_INCLUDE_DIRS:  Absolute path to package headers
#
# Example usage:
#
#   find_package(HAL)
#
#   include_directories(
#     ${HAL_INCLUDE_DIRS}
#   )
#
###############################################################################

set(MACHINEKIT_RIP_PATH $ENV{EMC2_HOME} CACHE STRING "")

find_path(
  HAL_INCLUDE_PATH hal.h
  PATH_SUFFIXES linuxcnc
  PATHS ${MACHINEKIT_RIP_PATH}/include
  )
if(HAL_INCLUDE_PATH)
  message(STATUS "Found HAL includes:  ${HAL_INCLUDE_PATH}")
  set(HAL_FOUND TRUE)
else(HAL_INCLUDE_PATH)
  message(FATAL_ERROR "Could not find HAL includes")
endif(HAL_INCLUDE_PATH)
