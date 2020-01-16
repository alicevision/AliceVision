add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_robustEssentialBA
  SOURCE main_robustEssentialBA.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_feature
        aliceVision_matching
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        vlsift
        Boost::filesystem
)
