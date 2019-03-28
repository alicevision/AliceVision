
# Headers
set(hdr_files_headers
 rgbCurve.hpp
 RobertsonCalibrate.hpp
 RobertsonMerge.hpp
)

# Sources
set(hdr_files_sources
 rgbCurve.cpp
 RobertsonCalibrate.cpp
 RobertsonMerge.cpp
)

alicevision_add_library(aliceVision_hdr
  SOURCES ${hdr_files_headers} ${hdr_files_sources}
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_image
    ${Boost_FILESYSTEM_LIBRARY}
)

# Unit tests

