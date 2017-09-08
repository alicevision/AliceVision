# Headers
set(robust_estimation_files_headers
  guided_matching.hpp
  lineTestGenerator.hpp
  rand_sampling.hpp
  rand_sampling_test.cpp
  robust_estimator_ACRansac.hpp
  robust_estimator_ACRansacKernelAdaptator.hpp
  robust_estimator_LMeds.hpp
  robust_estimator_LORansac.hpp
  robust_estimator_LORansacKernelAdaptor.hpp
  robust_estimator_MaxConsensus.hpp
  robust_estimator_MaxConsensus_test.cpp
  robust_estimator_Ransac.hpp
  robust_estimators.hpp
  robust_ransac_tools.hpp
  score_evaluator.hpp
)

# Sources
set(robust_estimation_files_test
  robust_estimator_ACRansac_test.cpp
  robust_estimator_lineKernel_test.cpp
  robust_estimator_lineKernel_test.hpp
  robust_estimator_LMeds_test.cpp
  robust_estimator_LORansac_test.cpp
  robust_estimator_Ransac_test.cpp
)

add_library(aliceVision_robust_estimation INTERFACE)

target_link_libraries(aliceVision_robust_estimation
  INTERFACE ${LOG_LIB}
)

UNIT_TEST(aliceVision rand_sampling                 "aliceVision_robust_estimation")
UNIT_TEST(aliceVision robust_estimator_lineKernel   "aliceVision_robust_estimation")
UNIT_TEST(aliceVision robust_estimator_MaxConsensus "aliceVision_robust_estimation")
UNIT_TEST(aliceVision robust_estimator_Ransac       "aliceVision_robust_estimation")
UNIT_TEST(aliceVision robust_estimator_LORansac     "aliceVision_robust_estimation")
#UNIT_TEST(aliceVision robust_estimator_LMeds        "aliceVision_robust_estimation")
UNIT_TEST(aliceVision robust_estimator_ACRansac     "aliceVision_robust_estimation")


add_custom_target(aliceVision_robust_estimation_ide SOURCES ${robust_estimation_files_headers} ${robust_estimation_files_test})
