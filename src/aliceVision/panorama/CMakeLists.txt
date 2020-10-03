# Headers
set(panorama_files_headers
  boundingBox.hpp
  warper.hpp
)

# Sources
set(panorama_files_sources
  warper.cpp
  gaussian.cpp
  coordinatesMap.cpp
  distance.cpp
  remapBbox.cpp
  sphericalMapping.cpp
  feathering.cpp
  laplacianPyramid.cpp
  seams.cpp
  imageOps.cpp
  cachedImage.cpp
)

alicevision_add_library(aliceVision_panorama
  SOURCES ${panorama_files_headers} ${panorama_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_image
)
