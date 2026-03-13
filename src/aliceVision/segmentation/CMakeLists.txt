# Headers
set(segmentation_files_headers
    segmentation.hpp
)

# Sources
set(segmentation_files_sources
    segmentation.cpp
)

set(SEGMENTATION_PRIVATE_LINKS "")
if (ALICEVISION_HAVE_CUDA)
    set(SEGMENTATION_PRIVATE_LINKS ${ALICEVISION_CUDA_LIBRARIES})
endif()

alicevision_add_library(aliceVision_segmentation
    SOURCES ${segmentation_files_headers} ${segmentation_files_sources}
    PUBLIC_LINKS
        aliceVision_system
        aliceVision_numeric
        aliceVision_image
        ONNXRuntime::ONNXRuntime
    PRIVATE_LINKS
        ${SEGMENTATION_PRIVATE_LINKS}
)
