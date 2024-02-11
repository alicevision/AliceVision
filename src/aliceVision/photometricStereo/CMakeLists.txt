# Headers
set(photometric_stereo_headers
    photometricDataIO.hpp
    normalIntegration.hpp
    photometricStereo.hpp
)

# Sources
set(photometric_stereo_sources
    photometricDataIO.cpp
    normalIntegration.cpp
    photometricStereo.cpp
)

alicevision_add_library(aliceVision_photometricStereo
  SOURCES ${photometric_stereo_headers} ${photometric_stereo_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
    aliceVision_system
    aliceVision_image
    aliceVision_sfmData
    aliceVision_sfmDataIO
    aliceVision_mvsData
)
