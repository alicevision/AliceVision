// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalKernel.hpp>

#define BOOST_TEST_MODULE fundamentalKernelSolver
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::multiview;

// check that sin(angle(a, b)) < tolerance.
template<typename A, typename B>
bool colinear(const A& a, const B& b, double tolerance)
{
  const bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols());

  if(!dims_match)
    return false;

  const double c = CosinusBetweenMatrices(a, b);

  if(c * c < 1)
  {
    double s = sqrt(1 - c * c);
    return fabs(s) < tolerance;
  }

  return false;
}

// check the properties of a fundamental matrix:
//   1. The determinant is 0 (rank deficient)
//   2. The condition x'T*F*x = 0 is satisfied to precision.
template<typename TMat>
bool expectFundamentalProperties(const TMat& F,
                                 const Mat& ptsA,
                                 const Mat& ptsB,
                                 double precision)
{
  bool bOk = true;
  bOk &= F.determinant() < precision;
  assert(ptsA.cols() == ptsB.cols());
  Mat hptsA, hptsB;

  euclideanToHomogeneous(ptsA, &hptsA);
  euclideanToHomogeneous(ptsB, &hptsB);

  for(int i = 0; i < ptsA.cols(); ++i)
  {
    const double residual = hptsB.col(i).dot(F * hptsA.col(i));
    bOk &= residual < precision;
  }

  return bOk;
}

// check the fundamental fitting:
//   1. Estimate the fundamental matrix
//   2. Check epipolar distance.
template<class Kernel>
bool expectKernelProperties(const Mat& x1, const Mat& x2, Mat3* F_expected = nullptr)
{
  bool bOk = true;
  const Kernel kernel(x1, x2);
  std::vector<std::size_t> samples;

  for(std::size_t i = 0; i < x1.cols(); ++i)
    samples.push_back(i);

  std::vector<robustEstimation::Mat3Model> Fs;
  kernel.fit(samples, Fs);

  bOk &= (!Fs.empty());

  for(int i = 0; i < Fs.size(); ++i)
  {
    bOk &= expectFundamentalProperties(Fs.at(i).getMatrix(), x1, x2, 1e-8);
    if(F_expected)
      bOk &= colinear(Fs.at(i).getMatrix(), *F_expected, 1e-6);
  }
  return bOk;
}

BOOST_AUTO_TEST_CASE(Fundamental7PKernel_EasyCase)
{
  Mat x1(2, 7), x2(2, 7);
  x1 << 0, 0, 0, 1, 1, 1, 2,
        0, 1, 2, 0, 1, 2, 0;
  x2 << 0, 0, 0, 1, 1, 1, 2,
        1, 2, 3, 1, 2, 3, 1;

  BOOST_CHECK(expectKernelProperties<relativePose::Fundamental7PKernel>(x1, x2));
}

BOOST_AUTO_TEST_CASE(NormalizedFundamental7PKernel_EasyCase)
{
  Mat x1(2, 7), x2(2, 7);
  x1 << 0, 0, 0, 1, 1, 1, 2,
        0, 1, 2, 0, 1, 2, 0;
  x2 << 0, 0, 0, 1, 1, 1, 2,
        1, 2, 3, 1, 2, 3, 1;

  BOOST_CHECK(expectKernelProperties<relativePose::NormalizedFundamental7PKernel>(x1, x2));
}

BOOST_AUTO_TEST_CASE(Fundamental8PKernel_EasyCase)
{
  Mat x1(2, 8), x2(2, 8);
  x1 << 0, 0, 0, 1, 1, 1, 2, 2,
        0, 1, 2, 0, 1, 2, 0, 1;
  x2 << 0, 0, 0, 1, 1, 1, 2, 2,
        1, 2, 3, 1, 2, 3, 1, 2;

  BOOST_CHECK(expectKernelProperties<relativePose::Fundamental8PKernel>(x1, x2));
}

BOOST_AUTO_TEST_CASE(NormalizedFundamental8PKernel_EasyCase)
{
  Mat x1(2, 8), x2(2, 8);
  x1 << 0, 0, 0, 1, 1, 1, 2, 2,
        0, 1, 2, 0, 1, 2, 0, 1;
  x2 << 0, 0, 0, 1, 1, 1, 2, 2,
        1, 2, 3, 1, 2, 3, 1, 2;

  BOOST_CHECK(expectKernelProperties<relativePose::NormalizedFundamental8PKernel>(x1, x2));
}
