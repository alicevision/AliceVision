add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_robustHomographyGrowing
  SOURCE main_robustHomographyGrowing.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_matching
        aliceVision_matchingImageCollection
        vlsift
        Boost::program_options
)
