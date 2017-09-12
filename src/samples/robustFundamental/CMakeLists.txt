add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustFundamental main_robustFundamental.cpp)

target_link_libraries(aliceVision_samples_robustFundamental
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  stlplus
  vlsift
)

set_property(TARGET aliceVision_samples_robustFundamental
  PROPERTY FOLDER AliceVision/Samples
)
