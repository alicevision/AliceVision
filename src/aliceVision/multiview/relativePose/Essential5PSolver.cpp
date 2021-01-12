// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Essential5PSolver.hpp"
#include <aliceVision/multiview/epipolarEquation.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

Vec o1(const Vec& a, const Vec& b)
{
  Vec res = Vec::Zero(20);

  res(Pc::coef_xx) = a(Pc::coef_x) * b(Pc::coef_x);
  res(Pc::coef_xy) = a(Pc::coef_x) * b(Pc::coef_y)
                   + a(Pc::coef_y) * b(Pc::coef_x);
  res(Pc::coef_xz) = a(Pc::coef_x) * b(Pc::coef_z)
                   + a(Pc::coef_z) * b(Pc::coef_x);
  res(Pc::coef_yy) = a(Pc::coef_y) * b(Pc::coef_y);
  res(Pc::coef_yz) = a(Pc::coef_y) * b(Pc::coef_z)
                   + a(Pc::coef_z) * b(Pc::coef_y);
  res(Pc::coef_zz) = a(Pc::coef_z) * b(Pc::coef_z);
  res(Pc::coef_x)  = a(Pc::coef_x) * b(Pc::coef_1)
                   + a(Pc::coef_1) * b(Pc::coef_x);
  res(Pc::coef_y)  = a(Pc::coef_y) * b(Pc::coef_1)
                   + a(Pc::coef_1) * b(Pc::coef_y);
  res(Pc::coef_z)  = a(Pc::coef_z) * b(Pc::coef_1)
                   + a(Pc::coef_1) * b(Pc::coef_z);
  res(Pc::coef_1)  = a(Pc::coef_1) * b(Pc::coef_1);

  return res;
}

Vec o2(const Vec& a, const Vec& b)
{
  Vec res(20);

  res(Pc::coef_xxx) = a(Pc::coef_xx) * b(Pc::coef_x);
  res(Pc::coef_xxy) = a(Pc::coef_xx) * b(Pc::coef_y)
                    + a(Pc::coef_xy) * b(Pc::coef_x);
  res(Pc::coef_xxz) = a(Pc::coef_xx) * b(Pc::coef_z)
                    + a(Pc::coef_xz) * b(Pc::coef_x);
  res(Pc::coef_xyy) = a(Pc::coef_xy) * b(Pc::coef_y)
                    + a(Pc::coef_yy) * b(Pc::coef_x);
  res(Pc::coef_xyz) = a(Pc::coef_xy) * b(Pc::coef_z)
                    + a(Pc::coef_yz) * b(Pc::coef_x)
                    + a(Pc::coef_xz) * b(Pc::coef_y);
  res(Pc::coef_xzz) = a(Pc::coef_xz) * b(Pc::coef_z)
                    + a(Pc::coef_zz) * b(Pc::coef_x);
  res(Pc::coef_yyy) = a(Pc::coef_yy) * b(Pc::coef_y);
  res(Pc::coef_yyz) = a(Pc::coef_yy) * b(Pc::coef_z)
                    + a(Pc::coef_yz) * b(Pc::coef_y);
  res(Pc::coef_yzz) = a(Pc::coef_yz) * b(Pc::coef_z)
                    + a(Pc::coef_zz) * b(Pc::coef_y);
  res(Pc::coef_zzz) = a(Pc::coef_zz) * b(Pc::coef_z);
  res(Pc::coef_xx)  = a(Pc::coef_xx) * b(Pc::coef_1)
                    + a(Pc::coef_x)  * b(Pc::coef_x);
  res(Pc::coef_xy)  = a(Pc::coef_xy) * b(Pc::coef_1)
                    + a(Pc::coef_x)  * b(Pc::coef_y)
                    + a(Pc::coef_y)  * b(Pc::coef_x);
  res(Pc::coef_xz)  = a(Pc::coef_xz) * b(Pc::coef_1)
                    + a(Pc::coef_x)  * b(Pc::coef_z)
                    + a(Pc::coef_z)  * b(Pc::coef_x);
  res(Pc::coef_yy)  = a(Pc::coef_yy) * b(Pc::coef_1)
                    + a(Pc::coef_y)  * b(Pc::coef_y);
  res(Pc::coef_yz)  = a(Pc::coef_yz) * b(Pc::coef_1)
                    + a(Pc::coef_y)  * b(Pc::coef_z)
                    + a(Pc::coef_z)  * b(Pc::coef_y);
  res(Pc::coef_zz)  = a(Pc::coef_zz) * b(Pc::coef_1)
                    + a(Pc::coef_z)  * b(Pc::coef_z);
  res(Pc::coef_x)   = a(Pc::coef_x)  * b(Pc::coef_1)
                    + a(Pc::coef_1)  * b(Pc::coef_x);
  res(Pc::coef_y)   = a(Pc::coef_y)  * b(Pc::coef_1)
                    + a(Pc::coef_1)  * b(Pc::coef_y);
  res(Pc::coef_z)   = a(Pc::coef_z)  * b(Pc::coef_1)
                    + a(Pc::coef_1)  * b(Pc::coef_z);
  res(Pc::coef_1)   = a(Pc::coef_1)  * b(Pc::coef_1);

  return res;
}

/**
 * @brief Compute the nullspace of the linear constraints given by the matches.
 */
Mat fivePointsNullspaceBasis(const Mat2X& x1, const Mat2X& x2)
{
  Eigen::Matrix<double,9, 9> A;
  A.setZero();  // make A square until Eigen supports rectangular SVD.
  encodeEpipolarEquation(x1, x2, &A);
  Eigen::JacobiSVD<Eigen::Matrix<double,9, 9> > svd(A,Eigen::ComputeFullV);
  return svd.matrixV().topRightCorner<9,4>();
}

/**
 * @brief Builds the polynomial constraint matrix M.
 */
Mat fivePointsPolynomialConstraints(const Mat& EBasis)
{
  // build the polynomial form of E (equation (8) in Stewenius et al. [1])
  Vec E[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      E[i][j] = Vec::Zero(20);
      E[i][j](Pc::coef_x) = EBasis(3 * i + j, 0);
      E[i][j](Pc::coef_y) = EBasis(3 * i + j, 1);
      E[i][j](Pc::coef_z) = EBasis(3 * i + j, 2);
      E[i][j](Pc::coef_1) = EBasis(3 * i + j, 3);
    }
  }

  // the constraint matrix.
  Mat M(10, 20);
  int mrow = 0;

  // determinant constraint det(E) = 0; equation (19) of Nister [2].
  M.row(mrow++) = o2(o1(E[0][1], E[1][2]) - o1(E[0][2], E[1][1]), E[2][0]) +
                  o2(o1(E[0][2], E[1][0]) - o1(E[0][0], E[1][2]), E[2][1]) +
                  o2(o1(E[0][0], E[1][1]) - o1(E[0][1], E[1][0]), E[2][2]);

  // cubic singular values constraint.
  // equation (20).
  Vec EET[3][3];
  for (int i = 0; i < 3; ++i) {    // since EET is symmetric, we only compute
    for (int j = 0; j < 3; ++j) {  // its upper triangular part.
      if (i <= j) {
        EET[i][j] = o1(E[i][0], E[j][0])
                  + o1(E[i][1], E[j][1])
                  + o1(E[i][2], E[j][2]);
      } else {
        EET[i][j] = EET[j][i];
      }
    }
  }

  // equation (21).
  Vec (&L)[3][3] = EET;
  Vec trace  = 0.5 * (EET[0][0] + EET[1][1] + EET[2][2]);
  for (int i = 0; i < 3; ++i) {
    L[i][i] -= trace;
  }

  // equation (23).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Vec LEij = o2(L[i][0], E[0][j])
               + o2(L[i][1], E[1][j])
               + o2(L[i][2], E[2][j]);
      M.row(mrow++) = LEij;
    }
  }

  return M;
}

void Essential5PSolver::solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const
{
  assert(2 == x1.rows());
  assert(5 <= x1.cols());
  assert(x1.rows() == x2.rows());
  assert(x1.cols() == x2.cols());

  // step 1: Nullspace Extraction.
  const Eigen::Matrix<double, 9, 4> EBasis = fivePointsNullspaceBasis(x1, x2);

  // step 2: Constraint Expansion.
  const Eigen::Matrix<double, 10, 20> EConstraints = fivePointsPolynomialConstraints(EBasis);

  // step 3: Gauss-Jordan Elimination (done thanks to a LU decomposition).
  typedef Eigen::Matrix<double, 10, 10> Mat10;
  Eigen::FullPivLU<Mat10> c_lu(EConstraints.block<10, 10>(0, 0));
  const Mat10 M = c_lu.solve(EConstraints.block<10, 10>(0, 10));

  // for next steps we follow the matlab code given in Stewenius et al [1].
  // build action matrix.
  const Mat10& B = M.topRightCorner<10,10>();
  Mat10 At = Mat10::Zero(10,10);
  At.block<3, 10>(0, 0) = B.block<3, 10>(0, 0);
  At.row(3) = B.row(4);
  At.row(4) = B.row(5);
  At.row(5) = B.row(7);
  At(6,0) = At(7,1) = At(8,3) = At(9,6) = -1;

  Eigen::EigenSolver<Mat10> eigensolver(At);
  const auto& eigenvectors = eigensolver.eigenvectors();
  const auto& eigenvalues = eigensolver.eigenvalues();

  // build essential matrices for the real solutions.
  models.reserve(10);
  for(int s = 0; s < 10; ++s)
  {
    // only consider real solutions.
    if(eigenvalues(s).imag() != 0)
      continue;

    Mat3 E;
    Eigen::Map<Vec9 >(E.data()) = EBasis * eigenvectors.col(s).tail<4>().real();
    models.emplace_back(E.transpose());
  }
}

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
