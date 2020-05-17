// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/multiview/relativePose/HomographyKernel.hpp>

#define BOOST_TEST_MODULE homographyKernelSolver
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::multiview;

BOOST_AUTO_TEST_CASE(NormalizedHomography4PKernel_Fitting)
{
  // define 3 knows homographies (Use as GT).
  std::vector<Mat3> H_gt(3);

  H_gt[0] = Mat3::Identity();
  H_gt[1] << 1,  0, -4,
             0,  1,  5,
             0,  0,  1;
  H_gt[2] << 1, -2,  3,
             4,  5, -6,
            -7,  8,  1;

  // define a set of points.
  Mat x(2, 9), xh;
  x << 0, 0, 0, 1, 1, 1, 2, 2, 2,
       0, 1, 2, 0, 1, 2, 0, 1, 2;

  euclideanToHomogeneous(x, &xh);

  for(int i = 0; i < H_gt.size(); ++i)
  {
    // transform points by the ground truth homography.
    Mat y, yh = H_gt[i] * xh;
    homogeneousToEuclidean(yh, &y);

    const relativePose::NormalizedHomography4PKernel kernel(x, y);

    std::size_t samples_[5] = { 0, 1, 2, 3, 4 };
    std::vector<std::size_t> samples(samples_,samples_+5);
    for(std::size_t j = 4; samples.size() < x.cols(); samples.push_back(j++))
    {
      std::vector<robustEstimation::Mat3Model> Hs;
      kernel.fit(samples, Hs);
      BOOST_CHECK_EQUAL(1, Hs.size());

      // check that found matrix is equal to the GT
      EXPECT_MATRIX_PROP(H_gt[i], Hs.at(0).getMatrix(), 1e-6);
    }
  }
}

BOOST_AUTO_TEST_CASE(Homography4PKernel_Fitting)
{
  // define 3 knows homographies (Use as GT).
  std::vector<Mat3> H_gt(3);

  H_gt[0] = Mat3::Identity();
  H_gt[1] << 1,  0, -4,
             0,  1,  5,
             0,  0,  1;
  H_gt[2] << 1, -2,  3,
             4,  5, -6,
            -7,  8,  1;

  // define a set of points.
  Mat x(2, 9), xh;
  x << 0, 0, 0, 1, 1, 1, 2, 2, 2,
       0, 1, 2, 0, 1, 2, 0, 1, 2;

  euclideanToHomogeneous(x, &xh);

  for(int i = 0; i < H_gt.size(); ++i)
  {
    // transform points by the ground truth homography.
    Mat y, yh = H_gt[i] * xh;
    homogeneousToEuclidean(yh, &y);

    const relativePose::Homography4PKernel kernel(x, y);

    std::size_t samples_[5]={0,1,2,3,4};
    std::vector<std::size_t> samples(samples_,samples_+5);

    for(std::size_t j = 4; samples.size() < x.cols(); samples.push_back(j++))
    {
      std::vector<robustEstimation::Mat3Model> Hs;
      kernel.fit(samples, Hs);
      BOOST_CHECK_EQUAL(1, Hs.size());

      // check that found matrix is equal to the GT
      EXPECT_MATRIX_PROP(H_gt[i], Hs.at(0).getMatrix(), 1e-6);
    }
  }
}
