// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"

namespace aliceVision {
namespace robustEstimation{

struct LineSolver
{
  enum { MINIMUM_SAMPLES = 2 };
  enum { MAX_MODELS = 1 };

  static void Solve(const Mat &x, std::vector<Vec2> *lines)
  {
    Mat X(x.cols(), 2);
    X.col(0).setOnes();
    X.col(1) = x.row(0).transpose();
    const Mat A(X.transpose() * X);
    const Vec b(X.transpose() * x.row(1).transpose());
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Vec2 ba = svd.solve(b);
    lines->push_back(ba);
  }
  
  static void SolveWeightedLS(const Mat &x, std::vector<Vec2> *lines, const std::vector<double> &weights)
  {
    const std::size_t numPts = x.cols();
    assert(numPts == weights.size());
    
    // create the weight matrix
    Mat W = Mat::Zero(numPts, numPts);
    for(std::size_t i = 0; i < numPts; ++i)
      W(i,i) = weights[i];

    Mat X(numPts, 2);
    X.col(0).setOnes();
    X.col(1) = x.row(0).transpose();
    const Mat A(X.transpose() * W * X);
    const Vec b(X.transpose() * W * x.row(1).transpose());
    Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Vec2 ba = svd.solve(b);
    lines->push_back(ba);
  }
};

struct pointToLineError
{
  static double Error(const Vec2 &lineEq, const Vec2 &xs)
  {
    const double b = lineEq[0];
    const double a = lineEq[1];
    const double x = xs[0];
    const double y = xs[1];
    const double e = y - (a * x + b);
    return e*e;
  }
};

// Embed the basic solver to fit from sampled point set
struct LineKernel
{
  typedef Vec2 Model;  // line parametrization: a, b;
  enum { MINIMUM_SAMPLES = 2 };

  LineKernel(const Mat2X &xs) : xs_(xs) {}

  size_t NumSamples() const { return static_cast<size_t> (xs_.cols()); }

  void Fit(const std::vector<size_t> &samples, std::vector<Vec2> *lines) const
  {
    assert(samples.size() >= (unsigned int) MINIMUM_SAMPLES);
    // Standard least squares solution.
    const Mat2X sampled_xs = ExtractColumns(xs_, samples);

    LineSolver::Solve(sampled_xs, lines);
  }

  double Error(size_t sample, const Vec2 &ba) const
  {
    return pointToLineError::Error(ba, xs_.col(sample));
  }
  
  void Unnormalize(Model * model) const
  {  }

  const Mat2X &xs_;
};

} // namespace robustEstimation
} // namespace aliceVision
