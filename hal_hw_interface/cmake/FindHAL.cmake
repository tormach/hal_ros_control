###############################################################################
#
# CMake script for finding HAL headers
#
# Output variables:
#
# - HAL_FOUND:  Boolean that indicates if the package was found
# - HAL_INCLUDE_DIRS:  Absolute path to package headers
# - HAL_INSTCOMP:  Path to instcomp script
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

set(MACHINEKIT_RIP_PATH
    $ENV{MK_HOME}
    CACHE STRING "")

# HAL_INCLUDE_PATH:  Find HAL include directory
find_path(
  HAL_INCLUDE_PATH hal/hal.h
  PATH_SUFFIXES machinekit/hal
  PATHS ${MACHINEKIT_RIP_PATH}/include
  )
if(HAL_INCLUDE_PATH)
  message(STATUS "Found HAL includes:  ${HAL_INCLUDE_PATH}")
  set(HAL_FOUND TRUE)
else(HAL_INCLUDE_PATH)
  message(FATAL_ERROR "Could not find HAL includes")
endif(HAL_INCLUDE_PATH)

# HAL_EXECUTABLE:  instcomp python script path
find_program(HAL_INSTCOMP NAMES instcomp)
