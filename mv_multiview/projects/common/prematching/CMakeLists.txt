set(SOURCES_files_prematching
  mv_prematch_cams.h
  mv_prematch_cams.cpp
)
source_group("prematching" FILES ${SOURCES_files_prematching})

set(SOURCES_prematching
  ${SOURCES_files_prematching}
)

add_library(mv_prematching ${SOURCES_prematching})

target_include_directories(mv_prematching
         PUBLIC 
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

target_link_libraries(mv_prematching PUBLIC ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARIES})

