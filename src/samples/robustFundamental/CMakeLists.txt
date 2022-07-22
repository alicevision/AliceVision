alicevision_add_software(aliceVision_samples_robustFundamental
  SOURCE main_robustFundamental.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_multiview
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
        aliceVision_vfs
        vlsift
        Boost::program_options
)
