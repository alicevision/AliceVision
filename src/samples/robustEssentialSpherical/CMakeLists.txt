add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_robustEssentialSpherical
  SOURCE main_robustEssentialSpherical.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_multiview
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        vlsift
)
