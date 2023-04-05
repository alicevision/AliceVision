# Headers
set(sphereDetection_headers
    sphereDetection.hpp
)

# Sources
set(sphereDetection_sources
    sphereDetection.cpp
)

alicevision_add_library(aliceVision_sphereDetection
  SOURCES ${sphereDetection_headers} ${sphereDetection_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
    aliceVision_image
    ONNXRuntime::ONNXRuntime
)
