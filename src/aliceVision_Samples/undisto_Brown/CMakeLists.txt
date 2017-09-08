add_executable(aliceVision_sample_undistoBrown undistoBrown.cpp)

target_link_libraries(aliceVision_sample_undistoBrown
  aliceVision_image
  aliceVision_multiview
  stlplus
  ${CERES_LIBRARIES}
)

set_property(TARGET aliceVision_sample_undistoBrown
  PROPERTY FOLDER AliceVision/Samples
)
