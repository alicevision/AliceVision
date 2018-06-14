# Headers
set(fuseCut_files_headers
  DelaunayGraphCut.hpp
  delaunayGraphCutTypes.hpp
  Fuser.hpp
  LargeScale.hpp
  MaxFlow_CSR.hpp
  MaxFlow_AdjList.hpp
  OctreeTracks.hpp
  ReconstructionPlan.hpp
  VoxelsGrid.hpp
)

# Sources
set(fuseCut_files_sources
  DelaunayGraphCut.cpp
  Fuser.cpp
  LargeScale.cpp
  MaxFlow_CSR.cpp
  MaxFlow_AdjList.cpp
  OctreeTracks.cpp
  ReconstructionPlan.cpp
  VoxelsGrid.cpp
)

add_library(aliceVision_fuseCut
  ${fuseCut_files_headers}
  ${fuseCut_files_sources}
)

target_include_directories(aliceVision_fuseCut
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
  PRIVATE $<BUILD_INTERFACE:${NANOFLANN_INCLUDE_DIR}>
)

target_link_libraries(aliceVision_fuseCut
  PUBLIC 
    aliceVision_mvsData
    aliceVision_mvsUtils
    aliceVision_imageIO
    aliceVision_mesh
    aliceVision_system
    Geogram::geogram
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_GRAPH_LIBRARY}
    ${Boost_CONTAINER_LIBRARY}
  PRIVATE
    nanoflann
)

set_property(TARGET aliceVision_fuseCut
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_fuseCut
  DESTINATION lib
  EXPORT aliceVision-targets
)
