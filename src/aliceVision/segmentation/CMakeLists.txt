# Headers
set(segmentation_files_headers
    segmentation.hpp
)

# Sources
set(segmentation_files_sources
    segmentation.cpp
)

alicevision_add_library(aliceVision_segmentation
  SOURCES ${segmentation_files_headers} ${segmentation_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_image
    ONNXRuntime::ONNXRuntime
    #penImageIO::OpenImageIO
)





