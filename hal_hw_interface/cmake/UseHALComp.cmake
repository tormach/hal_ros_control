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
#   hal_comp_add_module(src/mycomp)
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

function(set_hal_modinc_var dest_var modinc_var)
  execute_process(
    COMMAND bash -c "echo $'include ${HAL_MODINC}\nprint:\n\t@echo \$(${modinc_var})'"
    COMMAND make -s -f - print
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE modinc_var_value
  )
  set(${dest_var} "${modinc_var_value}" PARENT_SCOPE)
endfunction()


# hal_add_instcomp(src/my_mod)
function(hal_add_instcomp icomp_modpath)
  get_filename_component(icomp_name ${icomp_modpath} NAME)
  get_filename_component(icomp_dir ${icomp_modpath} DIRECTORY)
  set(icomp_src "${icomp_name}.icomp")
  set(icomp_c "${icomp_name}.c")
  set(icomp_src_path ${CMAKE_CURRENT_SOURCE_DIR}/${icomp_dir}/${icomp_src})

  # Generate C source with `instcomp`
  add_custom_command(
    OUTPUT ${icomp_c}
    # Copy .icomp file:  instcomp generates .c in same directory
    COMMAND cp ${icomp_src_path} ${icomp_src}
    COMMAND ${HAL_INSTCOMP} --preprocess ${icomp_src}
    DEPENDS ${icomp_src_path}
    COMMENT "Preprocessing instcomp ${icomp_modpath}")

  # Add the generated .c target
  add_custom_target(${icomp_c} DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${icomp_src})

  # Add the HAL comp .so target
  add_library(${icomp_name} MODULE ${icomp_c})

  # Omit the `lib` prefix and add `$EXTRA_CFLAGS`
  set_hal_modinc_var(HAL_EXTRA_CFLAGS "EXTRA_CFLAGS")
  set_target_properties(${icomp_name} PROPERTIES
    PREFIX ""
    COMPILE_FLAGS "${HAL_EXTRA_CFLAGS} -DRTAPI=1"
  )

  install(TARGETS ${icomp_name}
          LIBRARY DESTINATION lib)
endfunction()
