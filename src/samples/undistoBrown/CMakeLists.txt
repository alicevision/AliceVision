add_executable(aliceVision_samples_undistoBrown main_undistoBrown.cpp)

target_link_libraries(aliceVision_samples_undistoBrown
  aliceVision_system
  aliceVision_image
  aliceVision_multiview
  stlplus
  ${CERES_LIBRARIES}
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_samples_undistoBrown
  PROPERTY FOLDER AliceVision/Samples
)
