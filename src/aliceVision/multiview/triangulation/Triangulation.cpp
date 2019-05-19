// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Triangulation.hpp"
#include "NViewsTriangulationLORansac.hpp"
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>

namespace aliceVision {

void TriangulateNView(const Mat2X &x,
                      const std::vector< Mat34 > &Ps,
                      Vec4 *X, 
                      const std::vector<double> *weights)
{
  Mat2X::Index nviews = x.cols();
  assert(static_cast<std::size_t>(nviews) == Ps.size());

  Mat design = Mat::Zero(3 * nviews, 4 + nviews);
  for(Mat2X::Index i = 0; i < nviews; i++)
  {
    design.block<3, 4>(3 * i, 0) = -Ps[i];
    design(3 * i + 0, 4 + i) = x(0, i);
    design(3 * i + 1, 4 + i) = x(1, i);
    design(3 * i + 2, 4 + i) = 1.0;
  }
  Vec X_and_alphas;
  Nullspace(&design, &X_and_alphas);
  *X = X_and_alphas.head(4);
}

void TriangulateNViewAlgebraic(const Mat2X &x,
                               const std::vector< Mat34 > &Ps,
                               Vec4 *X, 
                               const std::vector<double> *weights)
{
  assert(X != nullptr);
  Mat2X::Index nviews = x.cols();
  assert(static_cast<std::size_t>(nviews) == Ps.size());

  Mat design(2 * nviews, 4);
  for(Mat2X::Index i = 0; i < nviews; ++i)
  {
    design.block<2, 4>(2 * i, 0) = SkewMatMinimal(x.col(i)) * Ps[i];
    if(weights != nullptr)
    {
      design.block<2, 4>(2 * i, 0) *= (*weights)[i];
    }
  }
  Nullspace(&design, X);
}

void TriangulateNViewLORANSAC(const Mat2X &x, 
                              const std::vector< Mat34 > &Ps,
                              Vec4 *X, 
                              std::vector<std::size_t> *inliersIndex, 
                              const double & thresholdError)
{
  using TriangulationKernel = LORansacTriangulationKernel<>;
  TriangulationKernel kernel(x, Ps);
  robustEstimation::ScoreEvaluator<TriangulationKernel> scorer(thresholdError);
  *X = robustEstimation::LO_RANSAC(kernel, scorer, inliersIndex);
}

double Triangulation::error(const Vec3 &X) const
{
  double squared_reproj_error = 0.0;
  for (const auto &view : views)
  {
    const Mat34& PMat = view.first;
    const Vec2 & xy = view.second;
    const Vec2 p = Project(PMat, X);
    squared_reproj_error += (xy - p).norm();
  }
  return squared_reproj_error;
}

Vec3 Triangulation::compute(int iter) const
{
  const auto nviews = views.size();
  assert(nviews >= 2);

  // Iterative weighted linear least squares
  Mat3 AtA;
  Vec3 Atb, X;
  std::vector<double> weights(nviews, double(1.0));
  for(int it = 0; it < iter; ++it)
  {
    AtA.fill(0.0);
    Atb.fill(0.0);
    for(std::size_t i = 0; i < nviews; ++i)
    {
      const Mat34& PMat = views[i].first;
      const Vec2 & p = views[i].second;
      const double w = weights[i];

      Vec3 v1, v2;
      for(Mat::Index j = 0; j < 3; ++j)
      {
        v1[j] = w * (PMat(0, j) - p(0) * PMat(2, j));
        v2[j] = w * (PMat(1, j) - p(1) * PMat(2, j));
        Atb[j] += w * (v1[j] * (p(0) * PMat(2, 3) - PMat(0, 3))
                + v2[j] * (p(1) * PMat(2, 3) - PMat(1, 3)));
      }

      for(Mat::Index k = 0; k < 3; ++k)
      {
        for(Mat::Index j = 0; j <= k; ++j)
        {
          const double v = v1[j] * v1[k] + v2[j] * v2[k];
          AtA(j, k) += v;
          if(j < k) AtA(k, j) += v;
        }
      }
    }

    X = AtA.inverse() * Atb;

    // Compute reprojection error, min and max depth, and update weights
    zmin = std::numeric_limits<double>::max();
    zmax = -std::numeric_limits<double>::max();
    err = 0;
    for(std::size_t i = 0; i < nviews; ++i)
    {
      const Mat34& PMat = views[i].first;
      const Vec2 & p = views[i].second;
      const Vec3 xProj = PMat * Vec4(X(0), X(1), X(2), 1.0);
      const double z = xProj(2);
      const Vec2 x = xProj.head<2>() / z;
      if(z < zmin) zmin = z;
      if(z > zmax) zmax = z;
      err += (p - x).norm();
      weights[i] = 1.0 / z;
    }
  }
  return X;
}

void TriangulateNViewsSolver::Solve(const Mat2X& x, const std::vector<Mat34>& Ps, std::vector<Vec4> &X)
{
  Vec4 pt3d;
  TriangulateNViewAlgebraic(x, Ps, &pt3d);
  X.push_back(pt3d);
  assert(X.size() == 1);
}

void TriangulateNViewsSolver::Solve(const Mat2X& x, const std::vector<Mat34>& Ps, std::vector<Vec4> &X, const std::vector<double> &weights)
{
  Vec4 pt3d;
  TriangulateNViewAlgebraic(x, Ps, &pt3d, &weights);
  X.push_back(pt3d);
  assert(X.size() == 1);
}

}  // namespace aliceCision

