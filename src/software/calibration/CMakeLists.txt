# Rig calibration
if(OpenMVG_USE_BOOST)

  ###
  # Calibrate a rig based on CCTag localizers
  ###
  add_executable(openMVG_main_rigCalibration main_rigCalibration.cpp)
  install(TARGETS openMVG_main_rigCalibration DESTINATION bin/)
  target_link_libraries(openMVG_main_rigCalibration 
    openMVG_rig 
    openMVG_localization  
    openMVG_dataio 
    openMVG_image 
    openMVG_features 
    vlsift 
    ${Boost_LIBRARIES}) #todo: delete vlsift
  if(OpenMVG_USE_CCTAG)
    target_link_libraries(openMVG_main_rigCalibration CCTag::CCTag)
  endif(OpenMVG_USE_CCTAG)

  if(OpenMVG_USE_OPENCV) 

    add_executable(openMVG_main_cameraCalibration main_cameraCalibration.cpp)
    # target_include_directories(openMVG_main_cameraCalibration PUBLIC ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(openMVG_main_cameraCalibration # PUBLIC
      openMVG_dataio
      openMVG_image
      openMVG_calibration
      openMVG_system
      ${OpenCV_LIBRARIES}
      ${Boost_LIBRARIES})
    if(OpenMVG_USE_CCTAG)
      target_link_libraries(openMVG_main_cameraCalibration CCTag::CCTag)
    endif()

    install(TARGETS openMVG_main_cameraCalibration DESTINATION bin/)

  endif()

endif(OpenMVG_USE_BOOST)
