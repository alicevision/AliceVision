// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/robustEstimation/LineKernel.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/randSampling.hpp>
#include <aliceVision/robustEstimation/lineTestGenerator.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#define BOOST_TEST_MODULE robustEstimation_AC_Ransac

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <iterator>
#include <random>

using namespace svg;

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// test ACRANSAC with the AC-adapted Line kernel in a noise/outlier free dataset
BOOST_AUTO_TEST_CASE(RansacLineFitter_OutlierFree)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 5);

  // y = 2x + 1
  xy << 1, 2, 3, 4, 5,
        3, 5, 7, 9, 11;

  // The base estimator
  LineKernel lineKernel(xy, 12, 12);

  // check the best model that fit the most of the data in a robust framework (ACRANSAC).
  std::vector<std::size_t> inliers;

  robustEstimation::MatrixModel<Vec2> model;
  ACRANSAC(lineKernel, randomNumberGenerator, inliers, 300, &model);

  BOOST_CHECK_SMALL(2.0 - model.getMatrix()[1], 1e-9);
  BOOST_CHECK_SMALL(1.0 - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_EQUAL(5, inliers.size());
}

// test without getting back the model
BOOST_AUTO_TEST_CASE(RansacLineFitter_OutlierFree_DoNotGetBackModel)
{
  std::mt19937 randomNumberGenerator;

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4, 5,
          3, 5, 7, 9, 11;

  LineKernel lineKernel(xy, 12, 12);
  std::vector<std::size_t> inliers;

  ACRANSAC(lineKernel, randomNumberGenerator, inliers);

  BOOST_CHECK_EQUAL(5, inliers.size());
}

BOOST_AUTO_TEST_CASE(RansacLineFitter_OneOutlier)
{
  std::mt19937 randomNumberGenerator;
  Mat2X xy(2, 6);
  // y = 2x + 1 with an outlier
  xy << 1, 2, 3, 4, 5, 100, // (100,-123) is the outlier
          3, 5, 7, 9, 11, -123;

  // The base estimator
  LineKernel lineKernel(xy, 12, 12);

  // Check the best model that fit the most of the data
  //  in a robust framework (ACRANSAC).
  std::vector<std::size_t> inliers;
  robustEstimation::MatrixModel<Vec2> model;

  ACRANSAC(lineKernel, randomNumberGenerator, inliers, 300, &model);

  BOOST_CHECK_SMALL(2.0 - model.getMatrix()[1], 1e-9);
  BOOST_CHECK_SMALL(1.0 - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_EQUAL(5, inliers.size());
}


// test if the robust estimator do not return inlier if too few point
// was given for an estimation.
BOOST_AUTO_TEST_CASE(RansacLineFitter_TooFewPoints)
{
  std::mt19937 randomNumberGenerator;
  Vec2 xy;
  // y = 2x + 1
  xy << 1, 2;
  LineKernel lineKernel(xy, 12, 12);
  std::vector<std::size_t> inliers;

  ACRANSAC(lineKernel, randomNumberGenerator, inliers);

  BOOST_CHECK_EQUAL(0, inliers.size());
}

// from a GT model :
//  Compute a list of point that fit the model.
//  Add white noise to given amount of points in this list.
//  Check that the number of inliers and the model are correct.
BOOST_AUTO_TEST_CASE(RansacLineFitter_RealisticCase)
{
  std::mt19937 randomNumberGenerator;
  const int NbPoints = 100;
  const float outlierRatio = .3;
  Mat2X xy(2, NbPoints);

  Vec2 GTModel; // y = 6.3 x + (-2.0)
  GTModel << -2.0, 6.3;

  //-- Build the point list according the given model
  for(Mat::Index i = 0; i < NbPoints; ++i)
  {
    xy.col(i) << i, (double) i * GTModel[1] + GTModel[0];
  }

  // Setup a normal distribution in order to make outlier not aligned
  std::mt19937 gen;
  std::normal_distribution<> d(0, 5); // More or less 5 units

  //-- Simulate outliers (for the asked percentage amount of the datum)
  const int nbPtToNoise = (int) NbPoints * outlierRatio;
  std::vector<std::size_t> vec_samples(nbPtToNoise); // Fit with unique random index
  std::iota(vec_samples.begin(), vec_samples.end(), 0);
  for(std::size_t i = 0; i < vec_samples.size(); ++i)
  {
    const std::size_t randomIndex = vec_samples[i];
    // Start from a outlier point (0,0)
    // and move it in a given small range (since it must remains in an outlier area)
    xy.col(randomIndex) << d(gen), d(gen);
  }

  // The base estimator
  LineKernel lineKernel(xy, 12, 12);

  // Check the best model that fit the most of the data
  //  in a robust framework (ACRANSAC).
  std::vector<std::size_t> inliers;
  robustEstimation::MatrixModel<Vec2> model;

  ACRANSAC(lineKernel, randomNumberGenerator, inliers, 300, &model);

  BOOST_CHECK_EQUAL(NbPoints - nbPtToNoise, inliers.size());
  BOOST_CHECK_SMALL(GTModel(0) - model.getMatrix()[0], 1e-9);
  BOOST_CHECK_SMALL(GTModel(1) - model.getMatrix()[1], 1e-9);
}

// generate nbPoints along a line and add gaussian noise.
// move some point in the dataset to create outlier contamined data
void generateLine(Mat & points, std::size_t nbPoints, int W, int H, float noise, float outlierRatio)
{
  points = Mat(2, nbPoints);

  Vec2 lineEq(50, 0.3);

  // Setup a normal distribution of mean 0 and amplitude equal to noise
  std::mt19937 gen;
  std::normal_distribution<> normDistribution(0, noise);

  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    const float x = i + normDistribution(gen);
    float y =  normDistribution(gen) + (lineEq[1] * x + lineEq[0]);
    points.col(i) = Vec2(x, y);
  }

  // generate outlier
  std::normal_distribution<> d_outlier(0, 0.2);
  std::size_t count = outlierRatio * nbPoints;
  const auto vec_indices = randSample<std::size_t>(gen, 0, nbPoints, count);
//  std::copy(vec_indices.begin(), vec_indices.end(), std::ostream_iterator<int>(std::cout, " "));
//  std::cout  << "\n";
  assert(vec_indices.size() == count);
//  std::cout << nbPoints << " " << count << "\n";
  for(const auto &idx : vec_indices)
  {
    std::cout << idx << " ";
    assert(idx < points.cols());
    points.col(idx) = Vec2(rand() % W + d_outlier(gen), rand() % H - d_outlier(gen));
  }
}

/// Structure used to avoid repetition in a given series
struct IndMatchd
{

  IndMatchd(double i = 0, double j = 0) : _i(i), _j(j) { }

  friend bool operator==(const IndMatchd& m1, const IndMatchd& m2)
  {
    return (m1._i == m2._i && m1._j == m2._j);
  }

  /// Lexicographical ordering of matches. Used to remove duplicates.
  friend bool operator<(const IndMatchd& m1, const IndMatchd& m2)
  {
    if(m1._i < m2._i) return true;
    if(m1._i > m2._i) return false;

    if(m1._j < m2._j) return true;
    else
      return false;

  }

  double _i, _j;
};

// test ACRANSAC adaptability to noise
// set a line with a increasing gaussian noise
// see if the AContrario RANSAC is able to label the good point as inlier
//  by having it's estimated confidence threshold growing.
BOOST_AUTO_TEST_CASE(RansacLineFitter_ACRANSACSimu)
{

  const int S = 100;
  const int W = S, H = S;
  const float outlierRatio = .3f;
  Vec2 GTModel; // y = 2x + 1
  GTModel << -2, .3;
  std::mt19937 gen;

  std::vector<double> vec_gaussianValue;
  for(std::size_t i = 0; i < 10; ++i)
  {
    vec_gaussianValue.push_back(i / 10. * 5.);
  }

  for(const auto& iter : vec_gaussianValue)
  {
    const double gaussianNoiseLevel = iter;

    std::size_t numPoints = 2.0 * S * sqrt(2.0);

    Mat2X points(2, numPoints);
    std::vector<std::size_t> vec_inliersGT;
    generateLine(numPoints, outlierRatio, gaussianNoiseLevel, GTModel, gen, points, vec_inliersGT);
    // robust line estimation
    robustEstimation::MatrixModel<Vec2> model;

    // The base estimator
    LineKernel lineKernel(points, W, H);

    // Check the best model that fit the most of the data
    //  in a robust framework (ACRANSAC).
    std::vector<std::size_t> vec_inliers;
    const std::pair<double,double> ret = ACRANSAC(lineKernel, gen, vec_inliers, 1000, &model);
    const double errorMax = ret.first;

    std::cout << "gaussianNoiseLevel " << gaussianNoiseLevel << " \tsqrt(errorMax) " << sqrt(errorMax) << std::endl;
    std::cout << "line " << model.getMatrix() << std::endl;
    const std::size_t expectedInliers = vec_inliersGT.size();

    // ACRansac is not really repeatable so we can check at least it does not
    // find more inliers than expected.
    BOOST_CHECK(vec_inliers.size() <= expectedInliers);

  }
}
