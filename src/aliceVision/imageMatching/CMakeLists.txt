# Headers
set(imageMatching_files_headers
    ImageMatching.hpp
)

# Sources
set(imageMatching_files_sources
    ImageMatching.cpp
)

alicevision_add_library(aliceVision_imageMatching
    SOURCES ${imageMatching_files_headers} ${imageMatching_files_sources}
    PRIVATE_LINKS
        aliceVision_image
        aliceVision_voctree
)
