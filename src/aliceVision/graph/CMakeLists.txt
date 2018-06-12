# Headers
set(graph_files_headers
  graph.hpp
  connectedComponent.hpp
  IndexedGraph.hpp
  indexedGraphGraphvizExport.hpp
  Triplet.hpp
)

# Sources
set(graph_files_test
  connectedComponent_test.cpp
  triplet_test.cpp
)

add_library(aliceVision_graph INTERFACE)

target_link_libraries(aliceVision_graph
  INTERFACE
    aliceVision_system 
    ${LEMON_LIBRARY}
)

install(TARGETS aliceVision_graph 
  EXPORT aliceVision-targets 
)

UNIT_TEST(aliceVision connectedComponent "aliceVision_graph")
UNIT_TEST(aliceVision triplet            "aliceVision_graph")

add_custom_target(aliceVision_graph_ide SOURCES ${graph_files_headers} ${graph_files_test})

set_property(TARGET aliceVision_graph_ide
  PROPERTY FOLDER AliceVision
)
