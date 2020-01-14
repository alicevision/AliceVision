// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "homographyKernelSolver.hpp"
#include <iostream>

namespace aliceVision {
namespace homography {
namespace kernel {

/// Setup the Direct Linear Transform.
///  Use template in order to support fixed or dynamic sized matrix.
/// Allow solve H as homogeneous(p2) = H homogeneous(p1)
template<typename Matrix >
void BuildActionMatrix(Matrix & L, const Mat &p1, const Mat &p2)  {

  const Mat::Index n = p1.cols();
  for (Mat::Index i = 0; i < n; ++i) {
    Mat::Index j = 2 * i;
    L(j, 0) = p1(0, i);
    L(j, 1) = p1(1, i);
    L(j, 2) = 1.0;
    L(j, 6) = -p2(0, i) * p1(0, i);
    L(j, 7) = -p2(0, i) * p1(1, i);
    L(j, 8) = -p2(0, i);

    ++j;
    L(j, 3) = p1(0, i);
    L(j, 4) = p1(1, i);
    L(j, 5) = 1.0;
    L(j, 6) = -p2(1, i) * p1(0, i);
    L(j, 7) = -p2(1, i) * p1(1, i);
    L(j, 8) = -p2(1, i);
  }
}

void FourPointSolver::Solve(const Mat &p1, const Mat &p2, vector<Mat3> *Hs) {
  assert(2 == p1.rows());
  assert(4 <= p2.cols());
  assert(p1.rows() == p2.rows());
  assert(p1.cols() == p2.cols());

  Mat::Index n = p1.cols();

  /* Normalize input */
  Mat p1_local = p1;
  Mat p2_local = p2;

  auto p1x = p1_local.block(0, 0, 1, n);
  auto p1y = p1_local.block(1, 0, 1, n);
  auto p2x = p2_local.block(0, 0, 1, n);
  auto p2y = p2_local.block(1, 0, 1, n);

  double p1x_mean = p1x.mean();
  double p1y_mean = p1y.mean();
  double p2x_mean = p2x.mean();
  double p2y_mean = p2y.mean();

  p1x.array() -= p1x_mean;
  p1y.array() -= p1y_mean;
  p2x.array() -= p2x_mean;
  p2y.array() -= p2y_mean;

  double p1_dist = sqrt((p1x * p1x.transpose() + p1y * p1y.transpose())(0,0));
  double p2_dist = sqrt((p2x * p2x.transpose() + p2y * p2y.transpose())(0,0));

  p1_local /= p1_dist;
  p2_local /= p2_dist; 

  Vec9 h;
  if (n == 4)  {
    // In the case of minimal configuration we use fixed sized matrix to let
    //  Eigen and the compiler doing the maximum of optimization.
    typedef Eigen::Matrix<double, 16, 9> Mat16_9;
    Mat16_9 L = Mat::Zero(16, 9);
    BuildActionMatrix(L, p1_local, p2_local);
    Nullspace(&L, &h);
  }
  else {
    MatX9 L = Mat::Zero(n * 2, 9);
    BuildActionMatrix(L, p1_local, p2_local);
    Nullspace(&L, &h);
  }
  
  /*Build G matrix from vector*/
  Mat3 G = Map<RMat3>(h.data()); 

  /*
  p2_local = G * p1_local
  Kp2 * p2 = G * Kp1 * p1
  p2 = Kp2^-1 * G * Kp1 * p1
  H = Kp2^-1 * G * Kp1
  */
  Eigen::Matrix3d Kp1 = Eigen::Matrix3d::Identity();
  Kp1(0, 0) = 1.0 / p1_dist;
  Kp1(0, 2) = - p1x_mean / p1_dist;
  Kp1(1, 1) = 1.0 / p1_dist;
  Kp1(1, 2) = - p1y_mean / p1_dist;

  Eigen::Matrix3d Kp2 = Eigen::Matrix3d::Identity();
  Kp2(0, 0) = 1.0 / p2_dist;
  Kp2(0, 2) = - p2x_mean / p2_dist;
  Kp2(1, 1) = 1.0 / p2_dist;
  Kp2(1, 2) = - p2y_mean / p2_dist;
  
  Eigen::Matrix3d H = Kp2.inverse() * G * Kp1;

  Hs->push_back(H);
}

}  // namespace kernel
}  // namespace homography
}  // namespace aliceVision
