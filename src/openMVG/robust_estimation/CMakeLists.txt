add_library(openMVG_robust_estimation INTERFACE)
target_link_libraries(openMVG_robust_estimation INTERFACE ${LOG_LIB})

UNIT_TEST(openMVG rand_sampling "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_lineKernel "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_MaxConsensus "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_Ransac "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_LORansac "openMVG_robust_estimation")
#UNIT_TEST(openMVG robust_estimator_LMeds "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_ACRansac "openMVG_robust_estimation")

