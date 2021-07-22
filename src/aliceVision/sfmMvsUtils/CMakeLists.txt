# Headers
set(sfmMvsUtils_files_headers
  visibility.hpp
)

# Sources
set(sfmMvsUtils_files_sources
  visibility.cpp
)

alicevision_add_library(aliceVision_sfmMvsUtils
  SOURCES ${sfmMvsUtils_files_headers} ${sfmMvsUtils_files_sources}
  PUBLIC_LINKS
    aliceVision_mesh
    aliceVision_mvsData
    aliceVision_sfmData
  PRIVATE_LINKS
    aliceVision_system
)
