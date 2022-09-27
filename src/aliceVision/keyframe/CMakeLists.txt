# Headers
set(keyframe_files_headers
  SharpnessSelectionPreset.hpp
  KeyframeSelector.hpp
)

# Sources
set(keyframe_files_sources
  KeyframeSelector.cpp
)

alicevision_add_library(aliceVision_keyframe
  SOURCES ${keyframe_files_headers} ${keyframe_files_sources}
  PUBLIC_LINKS
    aliceVision_dataio
    aliceVision_feature
    aliceVision_voctree
    OpenImageIO::OpenImageIO
  PRIVATE_LINKS
    aliceVision_sensorDB
    aliceVision_system
    Boost::filesystem
)
