# This module provides a function to find the ONNX Runtime library
#
# Usage:
#   find_package(ONNXRuntime REQUIRED)
#   target_link_libraries(mytarget PRIVATE ${ONNXRuntime_LIBRARIES})

# Set the search paths for the library and headers
find_library(ONNXRuntime_LIBRARY
  NAMES onnxruntime
  HINTS ${CMAKE_PREFIX_PATH} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64
)

find_path(ONNXRuntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${CMAKE_PREFIX_PATH} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES include include/onnxruntime  include/onnxruntime/core/session
)

# Check that we found everything we need
if (NOT ONNXRuntime_LIBRARY)
  message(FATAL_ERROR "Failed to find ONNX Runtime library")
endif()

if (NOT ONNXRuntime_INCLUDE_DIR)
  message(FATAL_ERROR "Failed to find ONNX Runtime headers")
endif()

# Export the target for downstream use
add_library(ONNXRuntime::ONNXRuntime INTERFACE IMPORTED)
set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES "${ONNXRuntime_LIBRARY}"
)

# Export variables for use in downstream projects
set(ONNXRuntime_FOUND TRUE)
set(ONNXRuntime_INCLUDE_DIRS "${ONNXRuntime_INCLUDE_DIR}")
set(ONNXRuntime_LIBRARIES "${ONNXRuntime_LIBRARY}")
mark_as_advanced(ONNXRuntime_INCLUDE_DIRS ONNXRuntime_LIBRARIES)

