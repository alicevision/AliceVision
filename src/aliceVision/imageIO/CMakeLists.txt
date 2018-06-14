# Headers
set(imageio_files_headers
  image.hpp
  imageScaledColors.hpp
)

# Sources
set(imageio_files_sources
  image.cpp
  imageScaledColors.cpp
)

add_library(aliceVision_imageIO
  ${imageio_files_headers}
  ${imageio_files_sources}
)

target_include_directories(aliceVision_imageIO
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${OPENIMAGEIO_INCLUDE_DIRS}
)

target_link_libraries(aliceVision_imageIO
  PUBLIC 
    ${OPENIMAGEIO_LIBRARIES}
  PRIVATE 
    aliceVision_mvsData
    aliceVision_system
    ${Boost_FILESYSTEM_LIBRARY}
)

set_property(TARGET aliceVision_imageIO
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_imageIO
  DESTINATION lib
  EXPORT aliceVision-targets
)
