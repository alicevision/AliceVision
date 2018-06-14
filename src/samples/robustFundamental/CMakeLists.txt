add_executable(aliceVision_samples_robustFundamental main_robustFundamental.cpp)

target_link_libraries(aliceVision_samples_robustFundamental
  aliceVision_system
  aliceVision_image
  aliceVision_multiview
  aliceVision_feature
  aliceVision_matching
  vlsift
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_robustFundamental
  PROPERTY FOLDER Samples
)
