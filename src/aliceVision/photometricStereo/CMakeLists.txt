# Headers
set(photometric_stereo_headers
    photometricDataIO.hpp
)

# Sources
set(photometric_stereo_sources
    photometricDataIO.cpp
)

alicevision_add_library(aliceVision_photometricStereo
  SOURCES ${photometric_stereo_headers} ${photometric_stereo_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_image
    aliceVision_sfmData
    Boost::filesystem
)
