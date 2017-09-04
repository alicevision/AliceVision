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
set(robust_estimation_files_test
  robust_estimator_ACRansac_test.cpp
  robust_estimator_lineKernel_test.cpp
  robust_estimator_lineKernel_test.hpp
  robust_estimator_LMeds_test.cpp
  robust_estimator_LORansac_test.cpp
  robust_estimator_Ransac_test.cpp
  )

add_library(openMVG_robust_estimation INTERFACE)
target_sources(openMVG_robust_estimation INTERFACE
    ${robust_estimation_files_headers}
    )

target_link_libraries(openMVG_robust_estimation
  INTERFACE ${LOG_LIB}
)

UNIT_TEST(openMVG rand_sampling                 "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_lineKernel   "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_MaxConsensus "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_Ransac       "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_LORansac     "openMVG_robust_estimation")
#UNIT_TEST(openMVG robust_estimator_LMeds        "openMVG_robust_estimation")
UNIT_TEST(openMVG robust_estimator_ACRansac     "openMVG_robust_estimation")


add_custom_target(openMVG_robust_estimation_ide SOURCES ${robust_estimation_files_headers} ${robust_estimation_files_test})
