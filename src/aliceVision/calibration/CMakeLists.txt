# Headers
set(calibration_files_headers
  calibration.hpp
  bestImages.hpp
  patternDetect.hpp
  exportData.hpp
  distortionEstimation.hpp
  checkerDetector.hpp
  checkerDetector_io.hpp
)

# Sources
set(calibration_files_sources
  distortionEstimation.cpp
  checkerDetector.cpp
  checkerDetector_io.cpp
)

if(ALICEVISION_HAVE_OPENCV)
  list(APPEND calibration_files_sources 
    bestImages.cpp
    calibration.cpp
    patternDetect.cpp
    exportData.cpp
  )
endif()

alicevision_add_library(aliceVision_calibration
  SOURCES ${calibration_files_headers} ${calibration_files_sources}
  PRIVATE_LINKS
    aliceVision_image
    aliceVision_system
    aliceVision_dataio
    Boost::program_options
    Boost::boost
    Boost::json
    ${CERES_LIBRARIES}
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_calibration PUBLIC ${OpenCV_LIBS})
endif()

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_calibration PRIVATE CCTag::CCTag)
endif()
