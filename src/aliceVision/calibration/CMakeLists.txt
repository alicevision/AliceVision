# Headers
set(calibration_files_headers
  calibration.hpp
  bestImages.hpp
  patternDetect.hpp
  exportData.hpp
  distortionEstimation.hpp
  checkerDetector.hpp
)

# Sources
set(calibration_files_sources
  calibration.cpp
  bestImages.cpp
  patternDetect.cpp
  exportData.cpp
  distortionEstimation.cpp
  checkerDetector.cpp
)

alicevision_add_library(aliceVision_calibration
  SOURCES ${calibration_files_headers} ${calibration_files_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
  PRIVATE_LINKS
    aliceVision_image
    aliceVision_system
    aliceVision_dataio
    Boost::filesystem
    Boost::program_options
    Boost::boost
    ${CERES_LIBRARIES}
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_calibration PRIVATE CCTag::CCTag)
endif()
