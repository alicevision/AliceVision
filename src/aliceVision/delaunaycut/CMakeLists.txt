# Headers
set(delaunayCut_files_headers
  mv_delaunay_GC.hpp
  mv_delaunay_helpers.hpp
  mv_delaunay_meshSmooth.hpp
  mv_delaunay_types.hpp
  hallucinations.hpp
  MaxFlow_CSR.hpp
  MaxFlow_AdjList.hpp
)

# Sources
set(delaunayCut_files_sources
  mv_delaunay_GC.cpp
  mv_delaunay_helpers.cpp
  mv_delaunay_meshSmooth.cpp
  hallucinations.cpp
  MaxFlow_CSR.cpp
  MaxFlow_AdjList.cpp
)

add_library(aliceVision_delaunaycut
  ${delaunayCut_files_headers}
  ${delaunayCut_files_sources}
)

target_include_directories(aliceVision_delaunaycut
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_delaunaycut
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_largeScale
         aliceVision_depthMap
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_delaunaycut
  PROPERTY FOLDER AliceVision
)
