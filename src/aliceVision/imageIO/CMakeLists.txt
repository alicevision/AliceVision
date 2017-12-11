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

add_library(aliceVision_imageIO
  ${imageio_files_headers}
  ${imageio_files_sources}
)

target_include_directories(aliceVision_imageIO
  PUBLIC ${OPENIMAGEIO_INCLUDE_DIRS}
)

target_link_libraries(aliceVision_imageIO
  PUBLIC ${OPENIMAGEIO_LIBRARIES}
         mv_structures
)
