alicevision_add_test(globalSfM_test.cpp
    NAME "sfm_globalSfM"
    LINKS aliceVision_sfm
          aliceVision_multiview_test_data
          aliceVision_feature
)
