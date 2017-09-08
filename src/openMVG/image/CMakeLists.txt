add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(image_files_headers
  image.hpp
  image_concat.hpp
  image_container.hpp
  image_converter.hpp
  image_convolution_base.hpp
  image_convolution.hpp
  image_diffusion.hpp
  image_drawing.hpp
  image_filtering.hpp
  image_io.hpp
  image_resampling.hpp
  image_warping.hpp
  pixel_types.hpp
  sample.hpp
)

# Sources
set(image_files_sources
  image_io.cpp
)

add_library(aliceVision_image
  ${image_files_headers}
  ${image_files_sources}
)

target_link_libraries(aliceVision_image
  aliceVision_numeric
  ${PNG_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${TIFF_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(aliceVision_image
  PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
  VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
)

install(TARGETS aliceVision_image
  DESTINATION lib
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision image            "aliceVision_image")
UNIT_TEST(aliceVision image_drawing    "aliceVision_image")
UNIT_TEST(aliceVision image_io         "aliceVision_image")
UNIT_TEST(aliceVision image_filtering  "aliceVision_image")
UNIT_TEST(aliceVision image_resampling "aliceVision_image")

