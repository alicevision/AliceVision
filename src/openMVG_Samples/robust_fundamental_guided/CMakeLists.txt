add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustFundamental_guided robust_fundamental_guided.cpp)

target_link_libraries(aliceVision_sample_robustFundamental_guided
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  stlplus
  vlsift
)

set_property(TARGET aliceVision_sample_robustFundamental_guided
  PROPERTY FOLDER AliceVision/Samples
)
