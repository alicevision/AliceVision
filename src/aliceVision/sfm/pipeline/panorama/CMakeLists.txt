
# The tests are split into multiple executables so that it's possible to easily run the tests
# in parallel using ctest -jX

alicevision_add_test(panoramaSfM_radial3_test.cpp
  NAME "sfm_panorama_radial3"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
)

alicevision_add_test(panoramaSfM_radial3_outliers_test.cpp
  NAME "sfm_panorama_radial3_outliers"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
)

alicevision_add_test(panoramaSfM_equidistant_test.cpp
  NAME "sfm_panorama_equidistant"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
)

alicevision_add_test(panoramaSfM_equidistant_outliers_test.cpp
  NAME "sfm_panorama_equidistant_outliers"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
)

