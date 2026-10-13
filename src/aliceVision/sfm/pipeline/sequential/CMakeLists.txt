alicevision_add_test(sequentialSfM_test.cpp
    NAME "sfm_sequentialSfM"
    LINKS aliceVision_sfm
          aliceVision_multiview_test_data
          aliceVision_feature
)
