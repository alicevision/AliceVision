# Headers
set(common_files_headers
  common.hpp
  fileIO.hpp
  ImagesCache.hpp
  MultiViewParams.hpp
  PreMatchCams.hpp
)

# Sources
set(common_files_sources
  common.cpp
  fileIO.cpp
  ImagesCache.cpp
  MultiViewParams.cpp
  PreMatchCams.cpp
)

add_library(aliceVision_mvsUtils
  ${common_files_headers}
  ${common_files_sources}
)

target_include_directories(aliceVision_mvsUtils
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_mvsUtils
  PUBLIC 
    aliceVision_mvsData
    aliceVision_imageIO
  PRIVATE 
    aliceVision_system
    ${Boost_FILESYSTEM_LIBRARY}
)

set_property(TARGET aliceVision_mvsUtils
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_mvsUtils
  DESTINATION lib
  EXPORT aliceVision-targets
)
