# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ts01_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ts01_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ts01_FOUND FALSE)
  elseif(NOT ts01_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ts01_FOUND FALSE)
  endif()
  return()
endif()
set(_ts01_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ts01_FIND_QUIETLY)
  message(STATUS "Found ts01: 0.0.0 (${ts01_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ts01' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ts01_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ts01_DIR}/${_extra}")
endforeach()
