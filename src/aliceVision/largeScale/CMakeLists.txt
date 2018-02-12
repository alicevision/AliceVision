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

add_library(mv_largeScale
  ${largeScale_files_headers}
  ${largeScale_files_sources}
)

target_include_directories(mv_largeScale
  PUBLIC
  $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${generatedDir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv_largeScale
  PUBLIC aliceVision_common
         aliceVision_imageIO
         mv_prematching
         mv_cudaInterfaces
         ${Boost_LIBRARIES}
         Geogram::geogram
)

set_property(TARGET mv_largeScale
  PROPERTY FOLDER AliceVision
)
