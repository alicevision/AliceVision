// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Essential8PSolver.hpp"
#include <aliceVision/multiview/epipolarEquation.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

void Essential8PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
  assert(2 == x1.rows());
  assert(8 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  MatX9 A(x1.cols(), 9);
  encodeEpipolarEquation(x1, x2, &A);

  Vec9 e;
  Nullspace(&A, &e);
  Mat3 E = Map<RMat3>(e.data());

  // Find the closest essential matrix to E in frobenius norm
  // E = UD'VT
  if (x1.cols() > 8)
  {
    Eigen::JacobiSVD<Mat3> USV(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3 d = USV.singularValues();
    const double a = d[0];
    const double b = d[1];
    d << (a+b)/2., (a+b)/2., 0.0;
    E = USV.matrixU() * d.asDiagonal() * USV.matrixV().transpose();
  }
  models.emplace_back(E);
}

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision

