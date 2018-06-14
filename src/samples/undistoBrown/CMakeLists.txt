add_executable(aliceVision_samples_undistoBrown main_undistoBrown.cpp)

target_link_libraries(aliceVision_samples_undistoBrown
  aliceVision_system
  aliceVision_image
  aliceVision_multiview
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_undistoBrown
  PROPERTY FOLDER Samples
)
