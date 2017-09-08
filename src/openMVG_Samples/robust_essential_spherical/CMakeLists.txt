add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_sample_robustEssential_spherical robust_essential_spherical.cpp)

target_link_libraries(aliceVision_sample_robustEssential_spherical
  aliceVision_image
  aliceVision_multiview
  aliceVision_features
  aliceVision_matching
  aliceVision_system
  aliceVision_sfm
  stlplus
  vlsift
)

set_property(TARGET aliceVision_sample_robustEssential_spherical PROPERTY FOLDER AliceVision/Samples)

