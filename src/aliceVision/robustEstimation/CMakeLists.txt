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

alicevision_add_interface(aliceVision_robustEstimation
  SOURCES ${robustEstimation_files_headers}
  LINKS
    aliceVision_camera
    aliceVision_feature
    aliceVision_matching
    aliceVision_numeric
    aliceVision_system
)

# Unit tests
alicevision_add_test(randSampling_test.cpp NAME "robustEstimation_randSampling" LINKS aliceVision_robustEstimation)
alicevision_add_test(lineKernel_test.cpp   NAME "robustEstimation_lineKernel"   LINKS aliceVision_robustEstimation)
alicevision_add_test(ransac_test.cpp       NAME "robustEstimation_ransac"       LINKS aliceVision_robustEstimation)
alicevision_add_test(acRansac_test.cpp     NAME "robustEstimation_acRansac"     LINKS aliceVision_robustEstimation)
alicevision_add_test(loRansac_test.cpp     NAME "robustEstimation_loRansac"     LINKS aliceVision_robustEstimation)
alicevision_add_test(maxConsensus_test.cpp NAME "robustEstimation_maxConsensus" LINKS aliceVision_robustEstimation)
# alicevision_add_test(leastMedianOfSquares_test.cpp NAME "robustEstimation_leastMedianOfSquares" LINKS aliceVision_robustEstimation)
