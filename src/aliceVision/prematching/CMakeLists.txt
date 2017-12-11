set(SOURCES_files_prematching
  mv_prematch_cams.hpp
  mv_prematch_cams.cpp
)
source_group("prematching" FILES ${SOURCES_files_prematching})

set(SOURCES_prematching
  ${SOURCES_files_prematching}
)

add_library(mv_prematching ${SOURCES_prematching})

target_include_directories(mv_prematching
         PUBLIC 
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${Boost_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

target_link_libraries(mv_prematching PUBLIC aliceVision_common ${Boost_FILESYSTEM_LIBRARIES})

