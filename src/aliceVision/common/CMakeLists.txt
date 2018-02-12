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

target_include_directories(aliceVision_common
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_common
  PUBLIC aliceVision_structures
         aliceVision_imageIO
)

set_property(TARGET aliceVision_common
  PROPERTY FOLDER AliceVision
)
