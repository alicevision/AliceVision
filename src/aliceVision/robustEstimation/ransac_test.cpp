// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/robustEstimation/LineKernel.hpp"
#include "aliceVision/robustEstimation/Ransac.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"

#include "aliceVision/numeric/numeric.hpp"

#include "testing/testing.h"
#include "lineTestGenerator.hpp"

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// Test without outlier
TEST(MaxConsensusLineFitter, OutlierFree) {

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  // The base estimator
  LineKernel kernel(xy);

  // Check the best model that fit the most of the data
  //  in a robust framework (Ransac).
  std::vector<size_t> vec_inliers;
  Vec2 model = RANSAC(kernel, ScoreEvaluator<LineKernel>(0.3), &vec_inliers);
  EXPECT_NEAR(2.0, model[1], 1e-9);
  EXPECT_NEAR(1.0, model[0], 1e-9);
  CHECK_EQUAL(5, vec_inliers.size());
}

// Test efficiency of MaxConsensus to find (inlier/outlier) in contamined data
TEST(MaxConsensusLineFitter, OneOutlier) {

  Mat2X xy(2, 6);
  // y = 2x + 1 with an outlier
  xy << 1, 2, 3, 4,  5, 100, // outlier!
        3, 5, 7, 9, 11, -123; // outlier!

  LineKernel kernel(xy);

  std::vector<size_t> vec_inliers;
  Vec2 model = RANSAC(kernel, ScoreEvaluator<LineKernel>(0.3), &vec_inliers);
  EXPECT_NEAR(2.0, model[1], 1e-9);
  EXPECT_NEAR(1.0, model[0], 1e-9);
  CHECK_EQUAL(5, vec_inliers.size());
}

// Critical test:
// Test if the robust estimator do not return inlier if too few point
// was given for an estimation.
TEST(MaxConsensusLineFitter, TooFewPoints) {

  Mat2X xy(2, 1);
  xy << 1,
        3;   // y = 2x + 1 with x = 1
  LineKernel kernel(xy);
  std::vector<size_t> vec_inliers;
  Vec2 model = RANSAC(kernel, ScoreEvaluator<LineKernel>(0.3), &vec_inliers);
  CHECK_EQUAL(0, vec_inliers.size());
}

// From a GT model :
//  Compute a list of point that fit the model.
//  Add white noise to given amount of points in this list.
//  Check that the number of inliers and the model are correct.
TEST(MaxConsensusLineFitter, RealisticCase) {

  const int NbPoints = 30;
  const double outlierRatio = .3; //works with 40
  Mat2X xy(2, NbPoints);
  std::mt19937 gen;

  Vec2 GTModel; // y = 2x + 1
  GTModel <<  -2.0, 6.3;

  //-- Add some noise (for the asked percentage amount)
  const std::size_t nbPtToNoise = (size_t) NbPoints*outlierRatio;
  std::vector<std::size_t> vec_inliersGT;
  generateLine(NbPoints, outlierRatio, 0.0, GTModel, gen, xy, vec_inliersGT);


  LineKernel kernel(xy);
  std::vector<size_t> vec_inliers;
  Vec2 model = RANSAC(kernel, ScoreEvaluator<LineKernel>(0.3), &vec_inliers);
  CHECK_EQUAL(NbPoints-nbPtToNoise, vec_inliers.size());
  EXPECT_NEAR(-2.0, model[0], 1e-9);
  EXPECT_NEAR( 6.3, model[1], 1e-9);
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
