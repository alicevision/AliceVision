# ==============================================================================
# AliceVision version
# ==============================================================================
file(STRINGS "aliceVision/version.hpp" _ALICEVISION_VERSION_HPP_CONTENTS REGEX "#define ALICEVISION_VERSION_")
foreach(v MAJOR MINOR REVISION)
  if("${_ALICEVISION_VERSION_HPP_CONTENTS}" MATCHES "#define ALICEVISION_VERSION_${v} ([0-9]+)")
    set(ALICEVISION_VERSION_${v} "${CMAKE_MATCH_1}")
  else()
    message(FATAL_ERROR "Failed to retrieve the AliceVision version the source code. Missing ALICEVISION_VERSION_${v}.")
  endif()
endforeach()
set(ALICEVISION_VERSION ${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}.${ALICEVISION_VERSION_REVISION})

# ==============================================================================
# Dependencies libraries:
# ==============================================================================
add_subdirectory(dependencies)

# ==============================================================================
# Include directories
# ==============================================================================
set(generatedDir "${CMAKE_CURRENT_BINARY_DIR}/generated")
message("generatedDir: ${generatedDir}")
set(ALICEVISION_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${generatedDir}
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies
  ${LEMON_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${UNCERTAINTYTE_INCLUDE_DIRS}
  ${MAGMA_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${LP_INCLUDE_DIRS}
  ${COINUTILS_INCLUDE_DIRS}
  ${CLP_INCLUDE_DIRS}
  ${OSI_INCLUDE_DIRS}
)

include_directories(${ALICEVISION_INCLUDE_DIRS})

# ==============================================================================
# aliceVision modules
# ==============================================================================
# The aliceVision library itself
add_subdirectory(aliceVision)

# ==============================================================================
# aliceVision libraries
# ==============================================================================
set(ALICEVISION_LIBRARIES
  aliceVision_dataio
  aliceVision_feature
  aliceVision_image
  aliceVision_kvld
#  aliceVision_linearProgramming
  aliceVision_lInftyComputerVision
  aliceVision_localization
  aliceVision_matching
  aliceVision_matchingImageCollection
  aliceVision_multiview
  aliceVision_numeric
  aliceVision_rig
  aliceVision_sensorDB
  aliceVision_sfm
  aliceVision_system
  aliceVision_voctree
)

if(ALICEVISION_HAVE_OPENCV)
  list(APPEND ALICEVISION_LIBRARIES aliceVision_calibration)
endif()

foreach(omvglib ${ALICEVISION_LIBRARIES})
  set_property(TARGET ${omvglib} PROPERTY FOLDER AliceVision/AliceVision)
endforeach()

# aliceVision tutorial examples
if(ALICEVISION_BUILD_EXAMPLES)
  add_subdirectory(samples)
endif()

# Complete software(s) build on aliceVision libraries
add_subdirectory(software)

# software(s) under patent or commercial licence
# Included for research purpose only
add_subdirectory(nonFree)







