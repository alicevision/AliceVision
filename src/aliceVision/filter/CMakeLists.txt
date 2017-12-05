set(SOURCES_files_filter
  mv_fuse.cpp
  mv_fuse.hpp
)
source_group("filter" FILES ${SOURCES_files_filter})


add_library(mv_filter ${SOURCES_files_filter})
target_include_directories(mv_filter         
         PUBLIC 
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_filter PUBLIC aliceVision_common aliceVision_imageIO mv_cudaInterfaces mv_output3D mv_prematching)

