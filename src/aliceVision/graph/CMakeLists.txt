# Headers
set(graph_files_headers
  graph.hpp
  connectedComponent.hpp
  IndexedGraph.hpp
  indexedGraphGraphvizExport.hpp
  Triplet.hpp
)

alicevision_add_interface(aliceVision_graph
  SOURCES ${graph_files_headers}
  LINKS
    aliceVision_system
    ${LEMON_LIBRARY}
)

# Unit tests
alicevision_add_test(connectedComponent_test.cpp NAME "graph_connectedComponent" LINKS aliceVision_graph)
alicevision_add_test(triplet_test.cpp            NAME "graph_triplet"            LINKS aliceVision_graph)
