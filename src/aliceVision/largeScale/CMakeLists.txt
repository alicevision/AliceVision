# Headers
set(largeScale_files_headers
  largeScale.hpp
  octreeTracks.hpp
  reconstructionPlan.hpp
  voxelsGrid.hpp
  mv_fuse.hpp
)

# Sources
set(largeScale_files_sources
  largeScale.cpp
  octreeTracks.cpp
  reconstructionPlan.cpp
  voxelsGrid.cpp
  mv_fuse.cpp
)

add_library(aliceVision_largeScale
  ${largeScale_files_headers}
  ${largeScale_files_sources}
)

target_include_directories(aliceVision_largeScale
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_largeScale
  PUBLIC aliceVision_common
         aliceVision_imageIO
         aliceVision_depthMap
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_largeScale
  PROPERTY FOLDER AliceVision
)
