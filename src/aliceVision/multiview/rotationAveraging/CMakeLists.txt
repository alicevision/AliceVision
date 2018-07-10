alicevision_add_test(rotationAveraging_test.cpp
  NAME "multiview_rotationAveraging"
  LINKS aliceVision_multiview
        aliceVision_multiview_test_data
        ${CERES_LIBRARIES}
)
