# Kernels
alicevision_add_test(fundamentalKernel_test.cpp        NAME "multiview_relativePose_fundamentalKernel"          LINKS aliceVision_multiview)
alicevision_add_test(homographyKernel_test.cpp         NAME "multiview_relativePose_homographyKernel"           LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(essentialKernel_test.cpp          NAME "multiview_relativePose_essentialKernel"            LINKS aliceVision_multiview aliceVision_multiview_test_data)

# Solvers
alicevision_add_test(fundamental10PSolver_test.cpp     NAME "multiview_relativePose_fundamental10Solver"        LINKS aliceVision_multiview aliceVision_multiview_test_data)
alicevision_add_test(essential5PSolver_test.cpp        NAME "multiview_relativePose_essential5PSolver"          LINKS aliceVision_multiview aliceVision_multiview_test_data)


