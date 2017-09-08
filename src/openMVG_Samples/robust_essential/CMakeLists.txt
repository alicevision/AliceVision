add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustEssential robust_essential.cpp)

target_link_libraries(aliceVision_sample_robustEssential
  aliceVision_image
  aliceVision_multiview
  aliceVision_system
  aliceVision_features
  aliceVision_matching
  aliceVision_sfm
  vlsift
)

set_property(TARGET aliceVision_sample_robustEssential
  PROPERTY FOLDER AliceVision/Samples
)
