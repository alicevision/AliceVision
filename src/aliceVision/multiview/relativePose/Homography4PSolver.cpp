// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Homography4PSolver.hpp"

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief Setup the Direct Linear Transform.
 *        Use template in order to support fixed or dynamic sized matrix.
 *        Allow solve H as homogeneous(x2) = H homogeneous(x1)
 */
template<typename Matrix >
void buildActionMatrix(Matrix& L, const Mat& x1, const Mat& x2)
{
  const Mat::Index n = x1.cols();
  for(Mat::Index i = 0; i < n; ++i)
  {
    Mat::Index j = 2 * i;
    L(j, 0) = x1(0, i);
    L(j, 1) = x1(1, i);
    L(j, 2) = 1.0;
    L(j, 6) = -x2(0, i) * x1(0, i);
    L(j, 7) = -x2(0, i) * x1(1, i);
    L(j, 8) = -x2(0, i);

    ++j;
    L(j, 3) = x1(0, i);
    L(j, 4) = x1(1, i);
    L(j, 5) = 1.0;
    L(j, 6) = -x2(1, i) * x1(0, i);
    L(j, 7) = -x2(1, i) * x1(1, i);
    L(j, 8) = -x2(1, i);
  }
}

void Homography4PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
  assert(2 == x1.rows());
  assert(4 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  const Mat::Index n = x1.cols();

  Vec9 h;
  if(n == 4)
  {
    // in the case of minimal configuration we use fixed sized matrix to let
    // Eigen and the compiler doing the maximum of optimization.
    typedef Eigen::Matrix<double, 16, 9> Mat16_9;
    Mat16_9 L = Mat::Zero(16, 9);
    buildActionMatrix(L, x1, x2);
    Nullspace(&L, &h);
  }
  else
  {
    MatX9 L = Mat::Zero(n * 2, 9);
    buildActionMatrix(L, x1, x2);
    Nullspace(&L, &h);
  }

  // map the linear vector as the H matrix
  Mat3 H = Map<RMat3>(h.data());
  models.emplace_back(H);
}

/// Setup the Direct Linear Transform.
///  Use template in order to support fixed or dynamic sized matrix.
/// Allow solve H as homogeneous(p2) = H homogeneous(p1)
template <typename Matrix>
void buildActionMatrixSpherical(Matrix& L, const Mat& p1, const Mat& p2)
{
    const Mat::Index n = p1.cols();

    for(Mat::Index i = 0; i < n; ++i)
    {
        Mat::Index j = 2 * i;
        L(j, 3) = -p2(2, i) * p1(0, i);
        L(j, 4) = -p2(2, i) * p1(1, i);
        L(j, 5) = -p2(2, i) * p1(2, i);
        L(j, 6) = p2(1, i) * p1(0, i);
        L(j, 7) = p2(1, i) * p1(1, i);
        L(j, 8) = p2(1, i) * p1(2, i);

        ++j;
        L(j, 0) = p2(2, i) * p1(0, i);
        L(j, 1) = p2(2, i) * p1(1, i);
        L(j, 2) = p2(2, i) * p1(2, i);
        L(j, 6) = -p2(0, i) * p1(0, i);
        L(j, 7) = -p2(0, i) * p1(1, i);
        L(j, 8) = -p2(0, i) * p1(2, i);
    }
}


void Homography4PSphericalSolver::solve(const Mat& p1, const Mat& p2, std::vector<robustEstimation::Mat3Model>& models) const
{
    assert(4 <= p2.cols());
    assert(p1.rows() == p2.rows());
    assert(p1.cols() == p2.cols());

    Mat::Index n = p1.cols();

    // No input normalization when on sphere
    Vec9 h;
    if(n == 4)
    {
        // In the case of minimal configuration we use fixed sized matrix to let
        //  Eigen and the compiler doing the maximum of optimization.
        typedef Eigen::Matrix<double, 16, 9> Mat16_9;
        Mat16_9 L = Mat::Zero(16, 9);
        buildActionMatrixSpherical(L, p1, p2);
        Nullspace(&L, &h);
    }
    else
    {
        MatX9 L = Mat::Zero(n * 2, 9);
        buildActionMatrixSpherical(L, p1, p2);
        Nullspace(&L, &h);
    }

    // Build G matrix from vector
    Mat3 H = Map<RMat3>(h.data());

    models.emplace_back(H);
}

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
