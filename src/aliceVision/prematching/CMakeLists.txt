# Headers
set(prematching_files_headers
  mv_prematch_cams.hpp
)

# Sources
set(prematching_files_sources
  mv_prematch_cams.cpp
)

add_library(aliceVision_prematching
  ${prematching_files_headers}
  ${prematching_files_sources}
)

target_include_directories(aliceVision_prematching
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${ZLIB_INCLUDE_DIR}
         ${Boost_INCLUDE_DIR}
)

target_link_libraries(aliceVision_prematching
  PUBLIC aliceVision_common
         ${Boost_FILESYSTEM_LIBRARIES}
)

set_property(TARGET aliceVision_prematching
  PROPERTY FOLDER AliceVision
)
