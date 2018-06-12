add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(image_files_headers
  all.hpp
  Image.hpp
  concat.hpp
  convertion.hpp
  convolutionBase.hpp
  convolution.hpp
  diffusion.hpp
  drawing.hpp
  filtering.hpp
  io.hpp
  resampling.hpp
  warping.hpp
  pixelTypes.hpp
  Sampler.hpp
)

# Sources
set(image_files_sources
  convolution.cpp
  filtering.cpp
  io.cpp
)

add_library(aliceVision_image
  ${image_files_headers}
  ${image_files_sources}
)

set_property(TARGET aliceVision_image
  PROPERTY FOLDER AliceVision
)

target_include_directories(aliceVision_image
  PUBLIC ${OPENIMAGEIO_INCLUDE_DIRS}
  PRIVATE ${OPENEXR_INCLUDE_DIR}
)

target_link_libraries(aliceVision_image
  PUBLIC 
    aliceVision_numeric
    ${OPENIMAGEIO_LIBRARIES}
  PRIVATE 
    aliceVision_system  
    ${OPENEXR_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
)

set_target_properties(aliceVision_image
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_image
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision image      "aliceVision_image")
UNIT_TEST(aliceVision drawing    "aliceVision_image")
UNIT_TEST(aliceVision io         "aliceVision_image")
UNIT_TEST(aliceVision filtering  "aliceVision_image")
UNIT_TEST(aliceVision resampling "aliceVision_image")

