add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustEssential_ba robust_essential_ba.cpp)

target_link_libraries(aliceVision_sample_robustEssential_ba
  aliceVision_image
  aliceVision_features
  aliceVision_matching
  aliceVision_system
  aliceVision_sfm
  stlplus
  vlsift)

set_property(TARGET aliceVision_sample_robustEssential_ba
  PROPERTY FOLDER AliceVision/Samples
)
