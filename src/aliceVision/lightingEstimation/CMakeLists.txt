# Headers
set(lightingEstimation_files_headers
    augmentedNormals.hpp
    lightingEstimation.hpp
    lightingCalibration.hpp
)

# Sources
set(lightingEstimation_files_sources
    augmentedNormals.cpp
    lightingEstimation.cpp
    lightingCalibration.cpp
)

alicevision_add_library(aliceVision_lightingEstimation
  SOURCES ${lightingEstimation_files_headers} ${lightingEstimation_files_sources}
  PUBLIC_LINKS
    aliceVision_image
    aliceVision_system
)


# Unit tests

alicevision_add_test(lightingEstimation_test.cpp
  NAME "lightingEstimation_lightingEstimation"
  LINKS aliceVision_lightingEstimation
)
