set(calibration_files_headers
  calibration.hpp
  bestImages.hpp
  patternDetect.hpp
  exportData.hpp
)
set(calibration_files_sources
  calibration.cpp
  bestImages.cpp
  patternDetect.cpp
  exportData.cpp
)

ADD_LIBRARY(openMVG_calibration
  ${calibration_files_headers}
  ${calibration_files_sources})
TARGET_LINK_LIBRARIES(openMVG_calibration 
        PUBLIC ${OpenCV_LIBS} 
        PRIVATE openMVG_image 
                openMVG_system 
                openMVG_dataio
                ${BOOST_LIBRARIES}
                ${LOG_LIB})
IF(OpenMVG_USE_CCTAG)
  TARGET_LINK_LIBRARIES(openMVG_calibration PRIVATE CCTag::CCTag)
ENDIF(OpenMVG_USE_CCTAG)

SET_PROPERTY(TARGET openMVG_calibration PROPERTY FOLDER OpenMVG)

INSTALL(TARGETS openMVG_calibration DESTINATION lib EXPORT openMVG-targets)
