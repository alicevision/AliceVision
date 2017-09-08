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

add_library(openMVG_calibration
  ${calibration_files_headers}
  ${calibration_files_sources}
)

target_link_libraries(openMVG_calibration
  PUBLIC ${OpenCV_LIBS}
  PRIVATE openMVG_image
          openMVG_system
          openMVG_dataio
          ${BOOST_LIBRARIES}
          ${LOG_LIB}
)

if(OPENMVG_HAVE_CCTAG)
  target_link_libraries(openMVG_calibration
    PRIVATE CCTag::CCTag
  )
endif()

set_property(TARGET openMVG_calibration
  PROPERTY FOLDER AliceVision
)

install(TARGETS openMVG_calibration
  DESTINATION lib
  EXPORT openMVG-targets
)
