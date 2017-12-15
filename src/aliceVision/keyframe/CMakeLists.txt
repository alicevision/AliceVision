# Headers
set(keyframe_files_headers
  SharpnessSelectionPreset.hpp
  KeyframeSelector.hpp
)

# Sources
set(keyframe_files_sources
  KeyframeSelector.cpp
)

add_library(aliceVision_keyframe
  ${keyframe_files_sources}
)

target_include_directories(aliceVision_keyframe
  PUBLIC ${OPENIMAGEIO_INCLUDE_DIRS}
)

target_link_libraries(aliceVision_keyframe
  PUBLIC aliceVision_feature
  aliceVision_sensorDB
  aliceVision_dataio
  aliceVision_voctree
  ${OPENIMAGEIO_LIBRARIES}
)

set_target_properties(aliceVision_keyframe
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_keyframe
  DESTINATION lib
  EXPORT aliceVision-targets
)

