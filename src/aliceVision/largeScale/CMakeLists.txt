# Headers
set(largeScale_files_headers
  Fuser.hpp
  LargeScale.hpp
  OctreeTracks.hpp
  ReconstructionPlan.hpp
  VoxelsGrid.hpp
)

# Sources
set(largeScale_files_sources
  Fuser.cpp
  LargeScale.cpp
  OctreeTracks.cpp
  ReconstructionPlan.cpp
  VoxelsGrid.cpp
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

install(TARGETS aliceVision_largeScale
  DESTINATION lib
  EXPORT aliceVision-targets
)
