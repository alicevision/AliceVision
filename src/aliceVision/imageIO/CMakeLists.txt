# Headers
set(imageio_files_headers
  image.hpp
  imageScaledColors.hpp
)

# Sources
set(imageio_files_sources
  image.cpp
  imageScaledColors.cpp
)

alicevision_add_library(aliceVision_imageIO
  SOURCES ${imageio_files_headers} ${imageio_files_sources}
  PUBLIC_LINKS
    ${OPENIMAGEIO_LIBRARIES}
  PRIVATE_LINKS
    aliceVision_mvsData
    aliceVision_system
    ${Boost_FILESYSTEM_LIBRARY}
  PUBLIC_INCLUDE_DIRS
    ${OPENIMAGEIO_INCLUDE_DIRS}
)
