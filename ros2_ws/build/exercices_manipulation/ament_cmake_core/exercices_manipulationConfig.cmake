# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_exercices_manipulation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED exercices_manipulation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(exercices_manipulation_FOUND FALSE)
  elseif(NOT exercices_manipulation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(exercices_manipulation_FOUND FALSE)
  endif()
  return()
endif()
set(_exercices_manipulation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT exercices_manipulation_FIND_QUIETLY)
  message(STATUS "Found exercices_manipulation: 0.1.0 (${exercices_manipulation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'exercices_manipulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${exercices_manipulation_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(exercices_manipulation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${exercices_manipulation_DIR}/${_extra}")
endforeach()
