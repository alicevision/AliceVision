add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustFundamental robust_fundamental.cpp)

target_link_libraries(aliceVision_sample_robustFundamental
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  stlplus
  vlsift
)

set_property(TARGET aliceVision_sample_robustFundamental
  PROPERTY FOLDER AliceVision/Samples
)
