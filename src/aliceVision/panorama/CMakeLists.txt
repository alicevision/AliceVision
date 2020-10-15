# Headers
set(panorama_files_headers
  alphaCompositer.hpp
  boundingBox.hpp
  boundingBoxMap.hpp
  compositer.hpp
  coordinatesMap.hpp
  distance.hpp
  feathering.hpp
  gaussian.hpp
  graphcut.hpp
  imageOps.hpp
  laplacianCompositer.hpp
  laplacianPyramid.hpp
  remapBbox.hpp
  seams.hpp
  sphericalMapping.hpp
  warper.hpp
)

# Sources
set(panorama_files_sources
  warper.cpp
  gaussian.cpp
  boundingBox.cpp
  coordinatesMap.cpp
  distance.cpp
  remapBbox.cpp
  sphericalMapping.cpp
  feathering.cpp
  laplacianPyramid.cpp
  seams.cpp
  imageOps.cpp
  cachedImage.cpp
  boundingBoxMap.cpp
)

alicevision_add_library(aliceVision_panorama
  SOURCES ${panorama_files_headers} ${panorama_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_image
)
