# Define a function to create HAL comps.
#
# This file defines a CMake function to build a HAL component.
# To use it, first include this file.
#
#   include(UseHALComp)
#
# Then call `hal_add_comp_module()` to create a component; e.g. if the
# source file is `src/mycomp.icomp`:
#
#   hal_add_instcomp(src/mycomp)
#
# The function will generate the C source with `instcomp`, build the
# comp and install it.

#=============================================================================
# Copyright 2015 John Morris <john@zultron.com>
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
# =============================================================================

find_package(HAL)

function(hal_add_instcomp instcomp_path)
  get_filename_component(icomp_name ${instcomp_path} NAME)
  get_filename_component(icomp_dir ${instcomp_path} DIRECTORY)
  set(icomp_src "${icomp_name}.icomp")
  set(icomp_c "${icomp_name}.c")
  set(icomp_src_path ${CMAKE_CURRENT_SOURCE_DIR}/${icomp_dir}/${icomp_src})

  # Generate C source with `instcomp`
  add_custom_command(
    OUTPUT ${icomp_c}
    # Copy .icomp file:  instcomp generates .c in same directory
    COMMAND cp ${icomp_src_path} ${icomp_src}
    COMMAND ${HAL_INSTCOMP} -p ${icomp_src}
    DEPENDS ${icomp_src_path}
    COMMENT "Preprocessing instcomp ${instcomp_path}")

  # Add the generated .c target
  add_custom_target(${icomp_c} DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${icomp_src})

  # Add the HAL comp .so target
  add_library(${icomp_name} MODULE ${icomp_c})

  # Add CFLAGS
  target_compile_definitions(${icomp_name} PRIVATE RTAPI=1)

  # Omit the `lib` prefix
  set_target_properties(${icomp_name} PROPERTIES PREFIX "")

  # Install HAL component
  install(TARGETS ${icomp_name}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endfunction()

function(hal_add_c_comp)
  get_filename_component(target ${ARGV0} NAME_WE)

  # Generate list of C source files
  set(comp_srcs "")
  foreach(comp_src_path IN LISTS ARGN)
    list(APPEND comp_srcs ${CMAKE_CURRENT_SOURCE_DIR}/${comp_src_path})
  endforeach()

  # Run `comp --compile <sources>` to build HAL .so module
  add_custom_command(
    OUTPUT "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${target}.so"
    WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    COMMAND env MAKEFLAGS=-j1 ${HAL_COMP} --compile ${comp_srcs}
    COMMENT "Building and linking C HAL comp ${target}"
    DEPENDS ${comp_srcs}
    )

  # Hook HAL .so module into build
  add_custom_target(
    build_${target}
    ALL
    DEPENDS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${target}.so"
    COMMENT "Built C HAL component ${target}"
    )

  # Install HAL .so module
  install(
    FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${target}.so
    CONFIGURATIONS Debug Release
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
endfunction()
