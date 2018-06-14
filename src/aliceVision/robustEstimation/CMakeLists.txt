# Headers
set(robustEstimation_files_headers
  guidedMatching.hpp
  lineTestGenerator.hpp
  randSampling.hpp
  LineKernel.hpp
  Ransac.hpp
  ACRansac.hpp
  ACRansacKernelAdaptator.hpp
  LORansac.hpp
  LORansacKernelAdaptor.hpp
  ransacTools.hpp
  estimators.hpp
  maxConsensus.hpp
  leastMedianOfSquares.hpp
  ScoreEvaluator.hpp
)

# Sources
set(robustEstimation_files_test
  ""
)

add_library(aliceVision_robustEstimation INTERFACE)

target_link_libraries(aliceVision_robustEstimation
  INTERFACE
    aliceVision_camera
    aliceVision_feature
    aliceVision_matching
    aliceVision_numeric
    aliceVision_system
)

install(TARGETS aliceVision_robustEstimation 
  EXPORT aliceVision-targets
)

UNIT_TEST(aliceVision randSampling "aliceVision_robustEstimation")
UNIT_TEST(aliceVision lineKernel   "aliceVision_robustEstimation")
UNIT_TEST(aliceVision ransac       "aliceVision_robustEstimation")
UNIT_TEST(aliceVision acRansac     "aliceVision_robustEstimation")
UNIT_TEST(aliceVision loRansac     "aliceVision_robustEstimation")
UNIT_TEST(aliceVision maxConsensus "aliceVision_robustEstimation")
#UNIT_TEST(aliceVision leastMedianOfSquares        "aliceVision_robustEstimation")

add_custom_target(aliceVision_robustEstimation_ide SOURCES ${robustEstimation_files_headers} ${robustEstimation_files_test})

set_property(TARGET aliceVision_robustEstimation_ide
  PROPERTY FOLDER AliceVision
)
