// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/robustEstimation/LineKernel.hpp"
#include "aliceVision/robustEstimation/leastMedianOfSquares.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"

#include "aliceVision/numeric/numeric.hpp"

#define BOOST_TEST_MODULE leastMedianOfSquares

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>


using namespace aliceVision;
using namespace aliceVision::robustEstimation;

static const double dExpectedPrecision = 1e-9;

template<typename Kernel>
void EvalInlier(const Kernel & kernel, const typename Kernel::Model & model,
   double dThreshold, std::vector<size_t> * vec_inliers)
{
  ScoreEvaluator<Kernel> scorer(dThreshold);
  std::vector<size_t> vec_index(kernel.NumSamples());
  for (size_t i = 0; i < kernel.NumSamples(); ++i)
    vec_index[i] = i;

  scorer.Score(kernel, model, vec_index, &(*vec_inliers));
}

// Test without outlier
BOOST_AUTO_TEST_CASE(LMedsLineFitter_OutlierFree) {

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;

  // The base estimator
  LineKernel kernel(xy);

  // Check the best model that fit the most of the data
  //  in a robust framework (LMeds).
  Vec2 model;
  double dThreshold = std::numeric_limits<double>::infinity();
  double dBestMedian = leastMedianOfSquares(kernel, &model, &dThreshold);
  BOOST_CHECK_CLOSE(2.0, model[1], dExpectedPrecision);
  BOOST_CHECK_CLOSE(1.0, model[0], dExpectedPrecision);
  BOOST_CHECK_CLOSE(0.0, dBestMedian, dExpectedPrecision);
  BOOST_CHECK_CLOSE(0.0, dThreshold, dExpectedPrecision);
  //Compute which point are inliers (error below dThreshold)
  std::vector<size_t> vec_inliers;
  EvalInlier(kernel, model, dExpectedPrecision, &vec_inliers);
  BOOST_CHECK_EQUAL(5, vec_inliers.size());
}

// Test efficiency of LMeds to find (inlier/outlier) in contamined data
BOOST_AUTO_TEST_CASE(LMedsLineFitter_OneOutlier) {

  Mat2X xy(2, 6);
  // y = 2x + 1 with an outlier
  xy << 1, 2, 3, 4,  5, 100, // outlier!
        3, 5, 7, 9, 11, -123; // outlier!

  LineKernel kernel(xy);

  Vec2 model;
  double dThreshold = std::numeric_limits<double>::infinity();
  double dBestMedian = leastMedianOfSquares(kernel, &model, &dThreshold);
  BOOST_CHECK_CLOSE(2.0, model[1], dExpectedPrecision);
  BOOST_CHECK_CLOSE(1.0, model[0], dExpectedPrecision);
  BOOST_CHECK_CLOSE(0.0, dBestMedian, dExpectedPrecision);
  BOOST_CHECK_CLOSE(0.0, dThreshold, dExpectedPrecision);
  //Compute which point are inliers (error below dThreshold)
  std::vector<size_t> vec_inliers;
  EvalInlier(kernel, model, dExpectedPrecision, &vec_inliers);
  BOOST_CHECK_EQUAL(5, vec_inliers.size());
}

// Critical test:
// Test if the robust estimator do not return inlier if too few point
// was given for an estimation.
BOOST_AUTO_TEST_CASE(LMedsLineFitter_TooFewPoints) {

  Mat2X xy(2, 1);
  xy << 1,
        3;   // y = 2x + 1 with x = 1
  LineKernel kernel(xy);

  Vec2 model;
  double dThreshold = std::numeric_limits<double>::infinity();
  double dBestMedian = leastMedianOfSquares(kernel, &model, &dThreshold);
  //No inliers
  BOOST_CHECK_EQUAL( dBestMedian, std::numeric_limits<double>::max());
}

// From a GT model :
//  Compute a list of point that fit the model.
//  Add white noise to given amount of points in this list.
//  Check that the number of inliers and the model are correct.
BOOST_AUTO_TEST_CASE(LMedsLineFitter_RealisticCase) {

  const int NbPoints = 30;
  const float outlierRatio = .3; //works with .4
  Mat2X xy(2, NbPoints);

  Vec2 GTModel; // y = 2x + 1
  GTModel <<  -2.0, 6.3;

  //-- Build the point list according the given model
  for(Mat::Index i = 0; i < NbPoints; ++i)
  {
    xy.col(i) << i, (double)i*GTModel[1] + GTModel[0];
  }

  //-- Add some noise (for the asked percentage amount)
  int nbPtToNoise = (int) NbPoints * outlierRatio;
  vector<size_t> vec_samples; // Fit with unique random index
  uniformSample(nbPtToNoise, NbPoints, vec_samples);
  for(size_t i = 0; i <vec_samples.size(); ++i)
  {
    const size_t randomIndex = vec_samples[i];
    //Additive random noise
    xy.col(randomIndex) << xy.col(randomIndex)(0)+rand()%2-3,
                           xy.col(randomIndex)(1)+rand()%8-6;
  }

  LineKernel kernel(xy);

  Vec2 model;
  double dThreshold = std::numeric_limits<double>::infinity();
  double dBestMedian = leastMedianOfSquares(kernel, &model, &dThreshold);
  BOOST_CHECK_CLOSE(-2.0, model[0], dExpectedPrecision);
  BOOST_CHECK_CLOSE(6.3, model[1], dExpectedPrecision);
  //Compute which point are inliers (error below dThreshold)
  std::vector<size_t> vec_inliers;
  EvalInlier(kernel, model, dThreshold, &vec_inliers);
  BOOST_CHECK(vec_inliers.size()>0);
  BOOST_CHECK_EQUAL(NbPoints-nbPtToNoise, vec_inliers.size());
}
