add_executable(aliceVision_sample_main_repeatability_dataset main_repeatability_dataset.cpp)

target_link_libraries(aliceVision_sample_main_repeatability_dataset
  aliceVision_image
  aliceVision_features
  aliceVision_matching
  aliceVision_system
  aliceVision_multiview
  vlsift
  stlplus)

set_property(TARGET aliceVision_sample_main_repeatability_dataset
  PROPERTY FOLDER AliceVision/Samples
)
