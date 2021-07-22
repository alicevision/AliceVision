# Headers
set(mvsUtils_files_headers
  common.hpp
  fileIO.hpp
  ImagesCache.hpp
  MultiViewParams.hpp
)

# Sources
set(mvsUtils_files_sources
  common.cpp
  fileIO.cpp
  ImagesCache.cpp
  MultiViewParams.cpp
)

alicevision_add_library(aliceVision_mvsUtils
  SOURCES ${mvsUtils_files_headers} ${mvsUtils_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    aliceVision_multiview
    aliceVision_mvsData
  PRIVATE_LINKS
    aliceVision_system
    Boost::filesystem
    Boost::boost
)
