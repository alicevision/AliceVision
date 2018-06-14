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

set_property(TARGET aliceVision_keyframe
  PROPERTY FOLDER AliceVision
)

target_include_directories(aliceVision_keyframe
  PUBLIC ${OPENIMAGEIO_INCLUDE_DIRS}
)

target_link_libraries(aliceVision_keyframe
  PUBLIC 
    aliceVision_dataio
    aliceVision_feature
    aliceVision_voctree
    ${OPENIMAGEIO_LIBRARIES}
  PRIVATE
    aliceVision_sensorDB
    aliceVision_system
)

set_target_properties(aliceVision_keyframe
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_keyframe
  DESTINATION lib
  EXPORT aliceVision-targets
)

