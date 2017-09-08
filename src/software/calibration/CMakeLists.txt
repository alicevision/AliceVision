###
# Calibrate a rig based on CCTag localizers
###
add_executable(aliceVision_main_rigCalibration main_rigCalibration.cpp)
install(TARGETS aliceVision_main_rigCalibration DESTINATION bin/)
target_link_libraries(aliceVision_main_rigCalibration
  aliceVision_rig
  aliceVision_localization
  aliceVision_dataio
  aliceVision_image
  aliceVision_features
  vlsift  #TODO: delete vlsift
  ${Boost_LIBRARIES}
)

if(OPENMVG_HAVE_CCTAG)
  target_link_libraries(aliceVision_main_rigCalibration CCTag::CCTag)
endif()

if(OPENMVG_HAVE_OPENCV)

  add_executable(aliceVision_main_cameraCalibration main_cameraCalibration.cpp)
  # target_include_directories(aliceVision_main_cameraCalibration PUBLIC ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(aliceVision_main_cameraCalibration # PUBLIC
    aliceVision_dataio
    aliceVision_image
    aliceVision_calibration
    aliceVision_system
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})

  if(OPENMVG_HAVE_CCTAG)
    target_link_libraries(aliceVision_main_cameraCalibration CCTag::CCTag)
  endif()

  install(TARGETS aliceVision_main_cameraCalibration DESTINATION bin/)

endif()
