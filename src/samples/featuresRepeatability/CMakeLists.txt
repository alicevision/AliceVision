alicevision_add_software(aliceVision_samples_repeatabilityDataset
  SOURCE main_repeatabilityDataset.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
        aliceVision_multiview
        vlsift
        Boost::program_options
        Boost::regex
        Boost::filesystem
)
