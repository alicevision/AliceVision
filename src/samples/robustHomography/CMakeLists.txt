add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_robustHomography
  SOURCE main_robustHomography.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_multiview
        aliceVision_feature
        aliceVision_matching
        vlsift
)
