set(SOURCES_files_output3D
  mv_output3D.cpp
  mv_output3D.h
)
source_group("output3D" FILES ${SOURCES_files_output3D})


add_library(mv_output3D ${SOURCES_files_output3D})
target_include_directories(mv_output3D
         PUBLIC 
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_output3D PUBLIC mv_structures)

