# Headers
set(common_files_headers
  common.hpp
  fileIO.hpp
  ImagesCache.hpp
  MultiViewParams.hpp
)

# Sources
set(common_files_sources
  common.cpp
  fileIO.cpp
  ImagesCache.cpp
  MultiViewParams.cpp
)

add_library(aliceVision_common
  ${common_files_headers}
  ${common_files_sources}
)

target_link_libraries(aliceVision_common
  PUBLIC mv_structures
         aliceVision_imageIO
)
