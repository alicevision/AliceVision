add_library(openMVG_graph INTERFACE)

target_link_libraries(openMVG_graph
  INTERFACE ${LEMON_LIBRARY}
            ${LOG_LIB}
)

UNIT_TEST(openMVG_graph connectedComponent "openMVG_graph")
UNIT_TEST(openMVG_graph triplet_finder     "openMVG_graph")

