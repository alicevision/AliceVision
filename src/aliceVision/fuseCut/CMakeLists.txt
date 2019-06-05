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

alicevision_add_library(aliceVision_fuseCut
  SOURCES ${fuseCut_files_headers} ${fuseCut_files_sources}
  PUBLIC_LINKS
    aliceVision_mvsData
    aliceVision_mvsUtils
    aliceVision_mesh
    aliceVision_system
    Geogram::geogram
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_GRAPH_LIBRARY}
    ${Boost_CONTAINER_LIBRARY}
  PRIVATE_LINKS
    nanoflann
)
