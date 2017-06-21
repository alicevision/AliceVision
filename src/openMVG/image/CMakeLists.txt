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

add_library(openMVG_image
  ${image_files_headers}
  ${image_files_sources}
)

target_link_libraries(openMVG_image
  openMVG_numeric
  ${PNG_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${TIFF_LIBRARIES}
  ${LOG_LIB}
)

set_target_properties(openMVG_image
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS openMVG_image
  DESTINATION lib
  EXPORT openMVG-targets
)

UNIT_TEST(openMVG image            "openMVG_image")
UNIT_TEST(openMVG image_drawing    "openMVG_image")
UNIT_TEST(openMVG image_io         "openMVG_image")
UNIT_TEST(openMVG image_filtering  "openMVG_image")
UNIT_TEST(openMVG image_resampling "openMVG_image")

