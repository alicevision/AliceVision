alicevision_add_test(p4pfSolver_test.cpp      NAME "multiview_p4pfSolver"      LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(p5pfrSolver_test.cpp     NAME "multiview_p5pfrSolver"     LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(Resection6PSolver_test.cpp NAME "multiview_resection6PSolver" LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(resectionKernel_test.cpp NAME "multiview_resectionKernel" LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(resectionLORansac_test.cpp
  NAME "multiview_resectionLORansac"
  LINKS aliceVision_multiview
        aliceVision_multiview_test_data
        aliceVision_feature
        aliceVision_sfm
        ${CERES_LIBRARIES}
)
