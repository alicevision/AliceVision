add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_describeAndMatch
  SOURCE main_describeAndMatch.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_matching
        vlsift
        Boost::program_options
)
