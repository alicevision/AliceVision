# Headers
set(imageMasking_files_headers
  imageMasking.hpp
)

# Sources
set(imageMasking_files_sources
  imageMasking.cpp
)

alicevision_add_library(aliceVision_imageMasking
  SOURCES ${imageMasking_files_headers} ${imageMasking_files_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
  PRIVATE_LINKS
    aliceVision_image
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
