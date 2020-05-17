// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Fundamental8PSolver.hpp"
#include <aliceVision/multiview/epipolarEquation.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

void solveProblem(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models, const std::vector<double>* weights = nullptr)
{
  assert(2 == x1.rows());
  assert(8 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  Vec9 f;

  if(x1.cols() == 8)
  {
    // in the minimal solution use fixed sized matrix to let Eigen and the
    // compiler doing the maximum of optimization.
    Mat9 A = Mat::Zero(9,9);
    encodeEpipolarEquation(x1, x2, &A, weights);
    Nullspace(&A, &f);
  }
  else  
  {
    MatX9 A(x1.cols(), 9);
    encodeEpipolarEquation(x1, x2, &A, weights);
    Nullspace(&A, &f);
  }

  Mat3 F = Map<RMat3>(f.data());

  // force the fundamental property if the A matrix has full rank.
  // @see HZ 11.1.1 pag.280
  if(x1.cols() > 8)
  {
    // force fundamental matrix to have rank 2
    Eigen::JacobiSVD<Mat3> USV(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3 d = USV.singularValues();
    d[2] = 0.0;
    F = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
  }
  models.emplace_back(F);
}

void Fundamental8PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
  solveProblem(x1, x2, models);
}

void Fundamental8PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models, const std::vector<double>& weights) const
{
  solveProblem(x1, x2, models, &weights);
}

}  // namespace kernel
}  // namespace fundamental
}  // namespace aliceVision
