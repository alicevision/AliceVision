add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(image_files_headers
  all.hpp
  Image.hpp
  imageAlgo.hpp
  colorspace.hpp
  concat.hpp
  convertion.hpp
  convolutionBase.hpp
  convolution.hpp
  dcp.hpp
  diffusion.hpp
  drawing.hpp
  filtering.hpp
  io.hpp
  resampling.hpp
  warping.hpp
  pixelTypes.hpp
  Rgb.hpp
  Sampler.hpp
  cache.hpp
  caching.hpp
)

# Sources
set(image_files_sources
  colorspace.cpp
  convolution.cpp
  dcp.cpp
  filtering.cpp
  io.cpp
  imageAlgo.cpp
  cache.cpp
  caching.cpp
)

alicevision_add_library(aliceVision_image
  SOURCES ${image_files_headers} ${image_files_sources}
  PUBLIC_LINKS
    aliceVision_numeric
    OpenImageIO::OpenImageIO
    OpenImageIO::OpenImageIO_Util
  PRIVATE_LINKS
    aliceVision_system
    ${OPENEXR_LIBRARIES}
    Boost::filesystem
  PRIVATE_INCLUDE_DIRS
    ${OPENEXR_INCLUDE_DIR}
)

# Install config.ocio
install(FILES ./share/aliceVision/config.ocio DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision)

# Unit tests
alicevision_add_test(image_test.cpp      NAME "image"            LINKS aliceVision_image)
alicevision_add_test(io_test.cpp         NAME "image_io"         LINKS aliceVision_image)
alicevision_add_test(drawing_test.cpp    NAME "image_drawing"    LINKS aliceVision_image)
alicevision_add_test(filtering_test.cpp  NAME "image_filtering"  LINKS aliceVision_image)
alicevision_add_test(resampling_test.cpp NAME "image_resampling" LINKS aliceVision_image)
alicevision_add_test(caching_test.cpp    NAME "caching"          LINKS aliceVision_image)
