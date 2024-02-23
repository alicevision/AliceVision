# Headers
set(fuseCut_files_headers
  delaunayGraphCutTypes.hpp
  Fuser.hpp
  MaxFlow_AdjList.hpp
  Octree.hpp
  InputSet.hpp
  BoundingBox.hpp
  Intersections.hpp
  PointCloud.hpp
  GraphFiller.hpp
  Mesher.hpp
)

# Sources
set(fuseCut_files_sources

  Fuser.cpp
  MaxFlow_AdjList.cpp
  InputSet.cpp
  Tetrahedralization.cpp
  Intersections.cpp
  PointCloud.cpp
  GraphFiller.cpp
  Mesher.cpp
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

