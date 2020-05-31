# Headers
set(robustEstimation_files_headers 
  conditioning.hpp
  estimators.hpp
  LineKernel.hpp
  lineTestGenerator.hpp
  PointFittingKernel.hpp
  PointFittingRansacKernel.hpp
  ISolver.hpp
  IRansacKernel.hpp
  Ransac.hpp 
  ACRansac.hpp
  LORansac.hpp
  ransacTools.hpp
  randSampling.hpp
  leastMedianOfSquares.hpp
  ScoreEvaluator.hpp
  maxConsensus.hpp
)

# Sources
set(robustEstimation_files_sources
  conditioning.cpp
)

alicevision_add_library(aliceVision_robustEstimation
  SOURCES ${robustEstimation_files_headers} ${robustEstimation_files_sources}
  PUBLIC_LINKS
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
