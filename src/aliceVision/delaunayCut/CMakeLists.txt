# Headers
set(delaunayCut_files_headers
  DelaunayGraphCut.hpp
  delaunayGraphCutTypes.hpp
  MaxFlow_CSR.hpp
  MaxFlow_AdjList.hpp
)

# Sources
set(delaunayCut_files_sources
  DelaunayGraphCut.cpp
  MaxFlow_CSR.cpp
  MaxFlow_AdjList.cpp
)

add_library(aliceVision_delaunayCut
  ${delaunayCut_files_headers}
  ${delaunayCut_files_sources}
)

target_include_directories(aliceVision_delaunayCut
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_delaunayCut
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_delaunayCut
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_delaunayCut
  DESTINATION lib
  EXPORT aliceVision-targets
)
