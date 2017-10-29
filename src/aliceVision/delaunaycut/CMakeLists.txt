
set(SOURCES_files_delaunaycut
  mv_delaunay_GC.cpp
  mv_delaunay_GC.hpp
  mv_delaunay_helpers.cpp
  mv_delaunay_helpers.hpp
  mv_delaunay_meshSmooth.cpp
  mv_delaunay_meshSmooth.hpp
  mv_delaunay_types.hpp
)
source_group("delaunaycut" FILES ${SOURCES_files_delaunaycut})

add_library(mv_delaunaycut ${SOURCES_files_delaunaycut})
target_include_directories(mv_delaunaycut 
         PUBLIC
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_delaunaycut PUBLIC mv_largeScale mv_structures mv_cudaInterfaces mv_hallucinations mv_mesh ${Boost_LIBRARIES} Geogram::geogram)
