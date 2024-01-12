# Headers
set(keyframe_files_headers
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
    aliceVision_sfmData
    aliceVision_sfmDataIO
    OpenImageIO::OpenImageIO
  PRIVATE_LINKS
    aliceVision_sensorDB
    aliceVision_system
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_keyframe PUBLIC ${OpenCV_LIBS})
endif()