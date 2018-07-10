add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_siftMatching
  SOURCE main_siftMatching.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_feature
        aliceVision_matching
        vlsift
)
