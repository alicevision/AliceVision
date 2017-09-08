add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustHomography robust_homography.cpp)

target_link_libraries(aliceVision_sample_robustHomography
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  vlsift
  stlplus
)

set_property(TARGET aliceVision_sample_robustHomography
  PROPERTY FOLDER AliceVision/Samples
)
