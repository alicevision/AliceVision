alicevision_add_software(aliceVision_samples_robustFundamentalF10
  SOURCE main_robustFundamentalF10.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_multiview
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
        vlsift
        ${Boost_LIBRARIES}
)
