add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustFundamentalGuided main_robustFundamentalGuided.cpp)

target_link_libraries(aliceVision_samples_robustFundamentalGuided
  aliceVision_image
  aliceVision_multiview
  aliceVision_feature
  aliceVision_matching
  vlsift
)

set_property(TARGET aliceVision_samples_robustFundamentalGuided
  PROPERTY FOLDER Samples
)
