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
    Boost::filesystem
    Boost::graph
    Boost::container
  PRIVATE_LINKS
    nanoflann
    Boost::boost
)

# Unit tests
alicevision_add_test(DelaunayGraphCut_test.cpp
  NAME "fuseCut_delaunayGraphCut"
  LINKS aliceVision_fuseCut
    aliceVision_sfm
    aliceVision_multiview
    aliceVision_multiview_test_data
)