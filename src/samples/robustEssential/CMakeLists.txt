add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustEssential main_robustEssential.cpp)

target_link_libraries(aliceVision_samples_robustEssential
  aliceVision_image
  aliceVision_multiview
  aliceVision_system
  aliceVision_feature
  aliceVision_matching
  aliceVision_sfm
  vlsift
)

set_property(TARGET aliceVision_samples_robustEssential
  PROPERTY FOLDER Samples
)
