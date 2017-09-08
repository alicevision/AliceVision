set(SOURCES_files_largeScale
  largeScale.cpp
  largeScale.h
  octreeTracks.cpp
  octreeTracks.h
  reconstructionPlan.cpp
  reconstructionPlan.h
  voxelsGrid.cpp
  voxelsGrid.h
)
source_group("largeScale" FILES ${SOURCES_files_largeScale})

add_library(mv_largeScale ${SOURCES_files_largeScale})
target_include_directories(mv_largeScale
         PUBLIC
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_largeScale PUBLIC mv_prematching mv_output3D mv_filter mv_delaunaycut mv_rply mv_cudaInterfaces ${Boost_LIBRARIES})
