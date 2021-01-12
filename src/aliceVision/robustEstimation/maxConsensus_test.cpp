// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/robustEstimation/LineKernel.hpp"
#include "aliceVision/robustEstimation/maxConsensus.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"

#include "aliceVision/numeric/numeric.hpp"

#define BOOST_TEST_MODULE maxConsensus

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// Test without outlier
BOOST_AUTO_TEST_CASE(MaxConsensusLineFitter_OutlierFree)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  // The base estimator
  LineKernel kernel(xy);

  // Check the best model that fit the most of the data
  //  in a robust framework (Max-consensus).
  std::vector<std::size_t> inliers;
  LineKernel::ModelT model = maxConsensus(kernel, ScoreEvaluator<LineKernel>(0.3), randomNumberGenerator, &inliers);
  BOOST_CHECK_SMALL(2.0 - model.getMatrix()[1], 1e-9);
  BOOST_CHECK_SMALL(1.0 - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_EQUAL(5, inliers.size());
}

// Test without getting back the model
BOOST_AUTO_TEST_CASE(MaxConsensusLineFitter_OutlierFree_DoNotGetBackModel)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  LineKernel kernel(xy);
  std::vector<std::size_t> inliers;
  LineKernel::ModelT model = maxConsensus(kernel, ScoreEvaluator<LineKernel>(0.3), randomNumberGenerator, &inliers);
  BOOST_CHECK_EQUAL(5, inliers.size());
}

// Test efficiency of MaxConsensus to find (inlier/outlier) in contamined data
BOOST_AUTO_TEST_CASE(MaxConsensusLineFitter_OneOutlier)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 6);
  // y = 2x + 1 with an outlier
  xy << 1, 2, 3, 4,  5, 100, // outlier!
        3, 5, 7, 9, 11, -123; // outlier!

  LineKernel kernel(xy);

  std::vector<std::size_t> inliers;
  LineKernel::ModelT model = maxConsensus(kernel, ScoreEvaluator<LineKernel>(0.3), randomNumberGenerator, &inliers);
  BOOST_CHECK_SMALL(2.0 - model.getMatrix()[1], 1e-9);
  BOOST_CHECK_SMALL(1.0 - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_EQUAL(5, inliers.size());
}

// Critical test:
// Test if the robust estimator do not return inlier if too few point
// was given for an estimation.
BOOST_AUTO_TEST_CASE(MaxConsensusLineFitter_TooFewPoints)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 1);
  xy << 1,
        3;   // y = 2x + 1 with x = 1
  LineKernel kernel(xy);
  std::vector<std::size_t> inliers;
  const LineKernel::ModelT model = maxConsensus(kernel, ScoreEvaluator<LineKernel>(0.3), randomNumberGenerator, &inliers);
  BOOST_CHECK_EQUAL(0, inliers.size());
}

// From a GT model :
//  Compute a list of point that fit the model.
//  Add white noise to given amount of points in this list.
//  Check that the number of inliers and the model are correct.
BOOST_AUTO_TEST_CASE(MaxConsensusLineFitter_RealisticCase)
{
  std::mt19937 randomNumberGenerator;
  const int numPoints = 30;
  const float outlierRatio = .3; //works with .4
  Mat2X xy(2, numPoints);

  Vec2 GTModel; // y = 2x + 1
  GTModel <<  -2.0, 6.3;

  //-- Build the point list according the given model
  for(Mat::Index i = 0; i < numPoints; ++i)
  {
    xy.col(i) << i, (double)i*GTModel[1] + GTModel[0];
  }

  //-- Add some noise (for the asked percentage amount)
  int nbPtToNoise = (int) numPoints * outlierRatio;
  vector<std::size_t> vec_samples; // Fit with unique random index
  uniformSample(randomNumberGenerator, nbPtToNoise, numPoints, vec_samples);
  for(std::size_t i = 0; i <vec_samples.size(); ++i)
  {
    const std::size_t randomIndex = vec_samples[i];
    //Additive random noise
    xy.col(randomIndex) << xy.col(randomIndex)(0)+rand()%2-3,
                           xy.col(randomIndex)(1)+rand()%8-6;
  }

  LineKernel kernel(xy);
  std::vector<std::size_t> inliers;
  LineKernel::ModelT model = maxConsensus(kernel, ScoreEvaluator<LineKernel>(0.3), randomNumberGenerator, &inliers);
  BOOST_CHECK_EQUAL(numPoints-nbPtToNoise, inliers.size());
  BOOST_CHECK_SMALL((-2.0) - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_SMALL( 6.3 - model.getMatrix()[1], 1e-9);
}
