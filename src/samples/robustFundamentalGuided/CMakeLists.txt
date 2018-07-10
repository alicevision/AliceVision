add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_robustFundamentalGuided
  SOURCE main_robustFundamentalGuided.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_multiview
        aliceVision_feature
        aliceVision_matching
        vlsift
)
