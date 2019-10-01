# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_endoscope_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED endoscope_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(endoscope_FOUND FALSE)
  elseif(NOT endoscope_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(endoscope_FOUND FALSE)
  endif()
  return()
endif()
set(_endoscope_CONFIG_INCLUDED TRUE)

# output package information
if(NOT endoscope_FIND_QUIETLY)
  message(STATUS "Found endoscope: 0.0.0 (${endoscope_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'endoscope' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(endoscope_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${endoscope_DIR}/${_extra}")
endforeach()
