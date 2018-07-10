# Headers
set(calibration_files_headers
  calibration.hpp
  bestImages.hpp
  patternDetect.hpp
  exportData.hpp
)

# Sources
set(calibration_files_sources
  calibration.cpp
  bestImages.cpp
  patternDetect.cpp
  exportData.cpp
)

alicevision_add_library(aliceVision_calibration
  SOURCES ${calibration_files_headers} ${calibration_files_sources}
  PUBLIC_LINKS
    ${OpenCV_LIBS}
  PRIVATE_LINKS
    aliceVision_image
    aliceVision_system
    aliceVision_dataio
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_calibration PRIVATE CCTag::CCTag)
endif()
