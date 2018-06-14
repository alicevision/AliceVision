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

add_library(aliceVision_calibration
  ${calibration_files_headers}
  ${calibration_files_sources}
)

target_link_libraries(aliceVision_calibration
  PUBLIC 
    ${OpenCV_LIBS}
  PRIVATE 
    aliceVision_image
    aliceVision_system
    aliceVision_dataio
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_calibration
    PRIVATE CCTag::CCTag
  )
endif()

set_property(TARGET aliceVision_calibration
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_calibration
  DESTINATION lib
  EXPORT aliceVision-targets
)
