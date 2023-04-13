###############################################################################
#
# CMake script for finding HAL headers
#
# Output variables:
#
# - HAL_FOUND:  Boolean that indicates if the package was found
# - HAL_INCLUDE_DIRS:  Absolute path to package headers
# - HAL_INSTCOMP:  Path to instcomp script
# - HAL_MODINC:  Path to Makefile.modinc
# - HAL_EXTRA_CFLAGS:  HAL RT comp CFLAGS
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
  HAL_INCLUDE_PATH hal.h
  PATH_SUFFIXES machinekit
  PATHS ${MACHINEKIT_RIP_PATH}/include
)

# HAL_EXECUTABLE:  instcomp python script path
find_program(HAL_INSTCOMP NAMES instcomp)

# HAL_MODINC:  Path to Makefile.modinc
execute_process(
  COMMAND ${HAL_INSTCOMP} --print-modinc
  OUTPUT_VARIABLE HAL_MODINC
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  HAL
  DEFAULT_MSG
  HAL_INCLUDE_PATH
  HAL_INSTCOMP
)
