# Headers
set(graph_files_headers
  connectedComponent.hpp
  graph.hpp
  graph_builder.hpp
  graph_graphviz_export.hpp
  triplet_finder.hpp
)

# Sources
set(graph_files_test
  connectedComponent_test.cpp
  triplet_finder_test.cpp
)

add_library(aliceVision_graph INTERFACE)

target_link_libraries(aliceVision_graph
  INTERFACE ${LEMON_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(aliceVision connectedComponent "aliceVision_graph")
UNIT_TEST(aliceVision triplet_finder     "aliceVision_graph")

add_custom_target(aliceVision_graph_ide SOURCES ${graph_files_headers} ${graph_files_test})
