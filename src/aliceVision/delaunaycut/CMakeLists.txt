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

add_library(mv_delaunaycut
  ${delaunayCut_files_headers}
  ${delaunayCut_files_sources}
)

target_include_directories(mv_delaunaycut 
  PUBLIC
  $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${generatedDir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mv_delaunaycut
  PUBLIC mv_largeScale
         mv_structures
         aliceVision_common
         mv_cudaInterfaces
         mv_mesh
         ${Boost_LIBRARIES}
         Geogram::geogram
)

set_property(TARGET mv_delaunaycut
  PROPERTY FOLDER AliceVision
)
