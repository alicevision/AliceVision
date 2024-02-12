# Headers
set(fuseCut_files_headers
  DelaunayGraphCut.hpp
  delaunayGraphCutTypes.hpp
  Fuser.hpp
  MaxFlow_AdjList.hpp
  Octree.hpp
  PointCloudBuilder.hpp
  GCOutput.hpp
  Kdtree.hpp
  GraphFiller.hpp
  Intersections.hpp
  GraphCut.hpp
  InputSet.hpp
)

# Sources
set(fuseCut_files_sources
  DelaunayGraphCut.cpp
  Fuser.cpp
  MaxFlow_AdjList.cpp
  PointCloudBuilder.cpp
  GCOutput.cpp
  GraphFiller.cpp
  Intersections.cpp
  Tetrahedralization.cpp
  GraphCut.cpp
  InputSet.cpp
)

alicevision_add_library(aliceVision_fuseCut
  SOURCES ${fuseCut_files_headers} ${fuseCut_files_sources}
  PUBLIC_LINKS
    aliceVision_mvsData
    aliceVision_mvsUtils
    aliceVision_mesh
    aliceVision_sfmData
    aliceVision_system
    Geogram::geogram
    Boost::graph
    Boost::container
  PRIVATE_LINKS
    nanoflann
    Boost::boost
    Boost::json
)

# Unit tests
alicevision_add_test(DelaunayGraphCut_test.cpp
  NAME "fuseCut_delaunayGraphCut"
  LINKS aliceVision_fuseCut
    aliceVision_sfm
    aliceVision_multiview
    aliceVision_multiview_test_data
)
