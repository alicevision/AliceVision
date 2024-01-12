# Headers
set(mvsUtils_files_headers
  common.hpp
  fileIO.hpp
  ImagesCache.hpp
  mapIO.hpp
  MultiViewParams.hpp
  TileParams.hpp
)

# Sources
set(mvsUtils_files_sources
  common.cpp
  fileIO.cpp
  ImagesCache.cpp
  mapIO.cpp
  MultiViewParams.cpp
  TileParams.cpp
)

alicevision_add_library(aliceVision_mvsUtils
  SOURCES ${mvsUtils_files_headers} ${mvsUtils_files_sources}
  PUBLIC_LINKS
    aliceVision_camera
    aliceVision_image
    aliceVision_numeric
    aliceVision_multiview
    aliceVision_mvsData
    OpenImageIO::OpenImageIO_Util
  PRIVATE_LINKS
    aliceVision_system
    Boost::boost
)
