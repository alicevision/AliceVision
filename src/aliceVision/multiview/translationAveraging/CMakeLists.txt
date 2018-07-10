alicevision_add_test(translationAveraging_test.cpp
  NAME "multiview_translationAveraging"
  LINKS aliceVision_multiview
        aliceVision_multiview_test_data
        ${CERES_LIBRARIES}
)

