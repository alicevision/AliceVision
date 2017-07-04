# Headers
set(keyframe_files_headers
  SharpnessSelectionPreset.hpp
  KeyframeSelector.hpp
)

# Sources
set(keyframe_files_sources
  KeyframeSelector.cpp
)

add_library(openMVG_keyframe
  ${keyframe_files_sources}
)

target_include_directories(openMVG_keyframe
  PUBLIC ${OPENIMAGEIO_INCLUDE_DIRS}
)

target_link_libraries(openMVG_keyframe
  PUBLIC openMVG_features
  openMVG_dataio
  openMVG_voctree
  ${OPENIMAGEIO_LIBRARIES}
)

set_target_properties(openMVG_keyframe
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_keyframe
  DESTINATION lib
  EXPORT openMVG-targets
)

