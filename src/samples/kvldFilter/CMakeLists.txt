add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_kvldFilter
  SOURCE main_kvldFilter.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_multiview
        aliceVision_kvld
        aliceVision_feature
        aliceVision_matching
        vlsift
)
