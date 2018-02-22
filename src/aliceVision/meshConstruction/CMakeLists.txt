# Headers
set(meshing_files_headers
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
set(meshing_files_sources
  DelaunayGraphCut.cpp
  Fuser.cpp
  LargeScale.cpp
  MaxFlow_CSR.cpp
  MaxFlow_AdjList.cpp
  OctreeTracks.cpp
  ReconstructionPlan.cpp
  VoxelsGrid.cpp
)

add_library(aliceVision_meshConstruction
  ${meshing_files_headers}
  ${meshing_files_sources}
)

target_include_directories(aliceVision_meshConstruction
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_meshConstruction
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_imageIO
         aliceVision_mesh
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshConstruction
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_meshConstruction
  DESTINATION lib
  EXPORT aliceVision-targets
)
