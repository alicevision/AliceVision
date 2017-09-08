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
  PUBLIC aliceVision_features
  aliceVision_dataio
  aliceVision_voctree
  ${OPENIMAGEIO_LIBRARIES}
)

set_target_properties(aliceVision_keyframe
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS aliceVision_keyframe
  DESTINATION lib
  EXPORT aliceVision-targets
)

