add_executable(openMVG_sample_undistoBrown undistoBrown.cpp)

target_link_libraries(openMVG_sample_undistoBrown
  openMVG_image
  openMVG_multiview
  stlplus
  ${CERES_LIBRARIES}
)

set_property(TARGET openMVG_sample_undistoBrown
  PROPERTY FOLDER AliceVision/Samples
)
