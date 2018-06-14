add_executable(aliceVision_samples_repeatabilityDataset main_repeatabilityDataset.cpp)

target_link_libraries(aliceVision_samples_repeatabilityDataset
  aliceVision_image
  aliceVision_feature
  aliceVision_matching
  aliceVision_system
  aliceVision_multiview
  vlsift
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_repeatabilityDataset
  PROPERTY FOLDER Samples
)
