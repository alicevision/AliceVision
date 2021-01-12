alicevision_add_test(panoramaSfM_test.cpp
  NAME "sfm_panorama"
  LINKS aliceVision_sfm
        aliceVision_feature
        aliceVision_matching
        aliceVision_system
)
