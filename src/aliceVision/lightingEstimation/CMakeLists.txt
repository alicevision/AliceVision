# Headers
set(lightingEstimation_files_headers
    augmentedNormals.hpp
    lightingEstimation.hpp
    lightingCalibration.hpp
    ellipseGeometry.hpp
)

# Sources
set(lightingEstimation_files_sources
    augmentedNormals.cpp
    lightingEstimation.cpp
    lightingCalibration.cpp
    ellipseGeometry.cpp
)

alicevision_add_library(aliceVision_lightingEstimation
  SOURCES ${lightingEstimation_files_headers} ${lightingEstimation_files_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
    aliceVision_image
    aliceVision_system
    aliceVision_photometricStereo
)


# Unit tests

alicevision_add_test(lightingEstimation_test.cpp
  NAME "lightingEstimation_lightingEstimation"
  LINKS aliceVision_lightingEstimation
)
