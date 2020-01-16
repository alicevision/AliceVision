add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

alicevision_add_software(aliceVision_samples_demoAKAZE
  SOURCE main_demoAKAZE.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_feature
        ${ALICEVISION_LIBRARY_DEPENDENCIES}
)

alicevision_add_software(aliceVision_samples_computeAKAZE
  SOURCE main_computeAKAZE.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_image
        aliceVision_feature
        aliceVision_system
        ${ALICEVISION_LIBRARY_DEPENDENCIES}
        Boost::filesystem
)
