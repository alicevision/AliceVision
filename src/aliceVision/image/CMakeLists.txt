add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# Headers
set(image_files_headers
    all.hpp
    Image.hpp
    imageAlgo.hpp
    colorspace.hpp
    concat.hpp
    conversion.hpp
    convolutionBase.hpp
    convolution.hpp
    dcp.hpp
    diffusion.hpp
    drawing.hpp
    filtering.hpp
    io.hpp
    jetColorMap.cpp
    resampling.hpp
    warping.hpp
    pixelTypes.hpp
    Rgb.hpp
    Sampler.hpp
    cache.hpp
)

# Sources
set(image_files_sources
    colorspace.cpp
    convolution.cpp
    dcp.cpp
    filtering.cpp
    io.cpp
    imageAlgo.cpp
    jetColorMap.cpp
    cache.cpp
)

alicevision_add_library(aliceVision_image
    SOURCES ${image_files_headers} ${image_files_sources}
    PUBLIC_LINKS
        aliceVision_numeric
        OpenImageIO::OpenImageIO
        OpenImageIO::OpenImageIO_Util
        Imath::Imath
    PRIVATE_LINKS
        aliceVision_system
        OpenEXR::OpenEXR
)

# Install config.ocio
install(FILES ./share/aliceVision/config.ocio DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision)
install(DIRECTORY ./share/aliceVision/luts DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision)

# Unit tests
alicevision_add_test(image_test.cpp        NAME "image"            LINKS aliceVision_image)
alicevision_add_test(io_test.cpp           NAME "image_io"         LINKS aliceVision_image)
alicevision_add_test(drawing_test.cpp      NAME "image_drawing"    LINKS aliceVision_image)
alicevision_add_test(filtering_test.cpp    NAME "image_filtering"  LINKS aliceVision_image)
alicevision_add_test(resampling_test.cpp   NAME "image_resampling" LINKS aliceVision_image)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(image
        SOURCES image.i
        PUBLIC_LINKS
            aliceVision_image
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()