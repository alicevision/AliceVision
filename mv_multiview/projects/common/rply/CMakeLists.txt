set(SOURCES_files_rply
  mv_plyloader.cpp
  mv_plyloader.h
  mv_plysaver.cpp
  mv_plysaver.h
  rply.c
  rply.h
)
source_group("rply" FILES ${SOURCES_files_rply})


add_library(mv_rply ${SOURCES_files_rply})
target_include_directories(mv_rply
         PUBLIC 
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_rply PUBLIC mv_mesh)
