add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustHomography_guided robust_homography_guided.cpp)

target_link_libraries(aliceVision_sample_robustHomography_guided
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  vlsift
  stlplus)

set_property(TARGET aliceVision_sample_robustHomography_guided
  PROPERTY FOLDER AliceVision/Samples
)
