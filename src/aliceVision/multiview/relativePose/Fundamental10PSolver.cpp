// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (c) 2015 Jan Heller <hellej1@cmp.felk.cvut.cz>, Zuzana Kukelova <zukuke@microsoft.com>
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Fundamental10PSolver.hpp"

#include <iostream>
#include <cassert>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/**
 * @brief	Computes column echelon form of a matrix
 * @author Jan Heller, adapted to aliceVision by Michal Polic
 */
template <typename Derived>
void colEchelonForm(Eigen::MatrixBase<Derived>& M, double pivtol = 1e-12)
{
  using Scalar = typename Derived::Scalar;

  const int n = M.rows();
  const int m = M.cols();

  Eigen::Index i = 0;
  Eigen::Index j = 0;

  while((i < m) && (j < n))
  {
    Scalar p = std::numeric_limits<Scalar>::min();
    Eigen::Index col = i;

    for(Eigen::Index k = i; k < m; ++k)
    {
      Scalar tp = std::abs(M(j, k));
      if(tp > p)
      {
        p = tp;
        col = k;
      }
    }

    if(p < Scalar(pivtol))
    {
      M.block(j, i, 1, m - i).setZero();
      ++j;
    }
    else
    {
      if(col != i)
        M.block(j, i, n - j, 1).swap(M.block(j, col, n - j, 1));

      M.block(j + 1, i, n - j - 1, 1) /= M(j, i);
      M(j, i) = 1.0;

      for(Eigen::Index k = 0; k < m; ++k)
      {
        if(k == i)
          continue;

        M.block(j, k, n - j, 1) -= M(j, k) * M.block(j, i, n - j, 1);
      }

      ++i;
      ++j;
    }
  }
}

template <typename Scalar, typename Derived>
int f10e_gb(const Eigen::Matrix<Scalar, 29, 1>& params, Eigen::MatrixBase<Derived>& sols, double pivtol = 1e-16)
{
  Eigen::Matrix<Scalar, 36, 1> c;

  c(0)  = params(0)  * params(14) - params(4)  * params(10);
  c(1)  = params(0)  * params(16) + params(2)  * params(14) - params(4) * params(12) - params(6) * params(10);
  c(2)  = params(2)  * params(18) - params(8)  * params(12);
  c(3)  = params(1)  * params(15) - params(5)  * params(11);
  c(4)  = params(1)  * params(17) + params(3)  * params(15) - params(5)  * params(13) - params(7) * params(11);
  c(5)  = params(0)  * params(15) + params(1)  * params(14) - params(4)  * params(11) - params(5) * params(10);
  c(6)  = params(0)  * params(17) + params(1)  * params(16) + params(2)  * params(15) + params(3) * params(14) - params(4) * params(13) - params(5) * params(12) - params(6) * params(11) - params(7) * params(10);
  c(7)  = params(0)  * params(18) + params(2)  * params(16) - params(6)  * params(12) - params(8) * params(10);
  c(8)  = params(0)  * params(19) + params(1)  * params(18) + params(2)  * params(17) + params(3) * params(16) - params(6) * params(13) - params(7) * params(12) - params(8) * params(11) - params(9) * params(10);
  c(9)  = params(2)  * params(19) + params(3)  * params(18) - params(8)  * params(13) - params(9) * params(12);
  c(10) = params(1)  * params(19) + params(3)  * params(17) - params(7)  * params(13) - params(9) * params(11);
  c(11) = params(3)  * params(19) - params(9)  * params(13);
  c(12) = params(0)  * params(23) - params(4)  * params(20);
  c(13) = params(1)  * params(23) + params(0)  * params(25) - params(4)  * params(21) - params(5) * params(20);
  c(14) = params(2)  * params(24) - params(8)  * params(20);
  c(15) = params(3)  * params(24) + params(2)  * params(27) - params(8)  * params(21) - params(9) * params(20);
  c(16) = params(1)  * params(26) - params(5)  * params(22);
  c(17) = params(0)  * params(24) + params(2)  * params(23) - params(6)  * params(20);
  c(18) = params(0)  * params(26) + params(1)  * params(25) - params(4)  * params(22) - params(5) * params(21);
  c(19) = params(1)  * params(24) + params(3)  * params(23) + params(0)  * params(27) + params(2) * params(25) - params(6) * params(21) - params(7) * params(20);
  c(20) = params(0)  * params(28) + params(1)  * params(27) + params(2)  * params(26) + params(3) * params(25) - params(6) * params(22) - params(7) * params(21);
  c(21) = params(2)  * params(28) + params(3)  * params(27) - params(8)  * params(22) - params(9) * params(21);
  c(22) = params(1)  * params(28) + params(3)  * params(26) - params(7)  * params(22);
  c(23) = params(3)  * params(28) - params(9)  * params(22);
  c(24) = params(10) * params(23) - params(14) * params(20);
  c(25) = params(11) * params(23) + params(10) * params(25) - params(14) * params(21) - params(15) * params(20);
  c(26) = params(12) * params(24) - params(18) * params(20);
  c(27) = params(13) * params(24) + params(12) * params(27) - params(18) * params(21) - params(19) * params(20);
  c(28) = params(11) * params(26) - params(15) * params(22);
  c(29) = params(10) * params(24) + params(12) * params(23) - params(16) * params(20);
  c(30) = params(10) * params(26) + params(11) * params(25) - params(14) * params(22) - params(15) * params(21);
  c(31) = params(11) * params(24) + params(13) * params(23) + params(10) * params(27) + params(12) * params(25) - params(16) * params(21) - params(17) * params(20);
  c(32) = params(10) * params(28) + params(11) * params(27) + params(12) * params(26) + params(13) * params(25) - params(16) * params(22) - params(17) * params(21);
  c(33) = params(12) * params(28) + params(13) * params(27) - params(18) * params(22) - params(19) * params(21);
  c(34) = params(11) * params(28) + params(13) * params(26) - params(17) * params(22);
  c(35) = params(13) * params(28) - params(19) * params(22);

  Eigen::Matrix<Scalar, 20, 10> M;
  M.setZero();

  M(0)   =  c(0);  M(61)  =  c(0);  M(82)  =  c(0);
  M(144) =  c(0);  M(2)   =  c(1);  M(64)  =  c(1);
  M(85)  =  c(1);  M(148) =  c(1);  M(9)   =  c(2);
  M(72)  =  c(2);  M(93)  =  c(2);  M(156) =  c(2);
  M(3)   =  c(3);  M(66)  =  c(3);  M(87)  =  c(3);
  M(150) =  c(3);  M(7)   =  c(4);  M(70)  =  c(4);
  M(91)  =  c(4);  M(154) =  c(4);  M(1)   =  c(5);
  M(63)  =  c(5);  M(84)  =  c(5);  M(147) =  c(5);
  M(4)   =  c(6);  M(67)  =  c(6);  M(88)  =  c(6);
  M(151) =  c(6);  M(5)   =  c(7);  M(68)  =  c(7);
  M(89)  =  c(7);  M(152) =  c(7);  M(8)   =  c(8);
  M(71)  =  c(8);  M(92)  =  c(8);  M(155) =  c(8);
  M(12)  =  c(9);  M(75)  =  c(9);  M(96)  =  c(9);
  M(158) =  c(9);  M(11)  = c(10);  M(74)  = c(10);
  M(95)  = c(10);  M(157) = c(10);  M(15)  = c(11);
  M(77)  = c(11);  M(98)  = c(11);  M(159) = c(11);
  M(20)  = c(12);  M(102) = c(12);  M(165) = c(12);
  M(21)  = c(13);  M(104) = c(13);  M(168) = c(13);
  M(25)  = c(14);  M(109) = c(14);  M(173) = c(14);
  M(28)  = c(15);  M(112) = c(15);  M(176) = c(15);
  M(26)  = c(16);  M(110) = c(16);  M(174) = c(16);
  M(22)  = c(17);  M(105) = c(17);  M(169) = c(17);
  M(23)  = c(18);  M(107) = c(18);  M(171) = c(18);
  M(24)  = c(19);  M(108) = c(19);  M(172) = c(19);
  M(27)  = c(20);  M(111) = c(20);  M(175) = c(20);
  M(31)  = c(21);  M(115) = c(21);  M(178) = c(21);
  M(30)  = c(22);  M(114) = c(22);  M(177) = c(22);
  M(34)  = c(23);  M(117) = c(23);  M(179) = c(23);
  M(40)  = c(24);  M(122) = c(24);  M(185) = c(24);
  M(41)  = c(25);  M(124) = c(25);  M(188) = c(25);
  M(45)  = c(26);  M(129) = c(26);  M(193) = c(26);
  M(48)  = c(27);  M(132) = c(27);  M(196) = c(27);
  M(46)  = c(28);  M(130) = c(28);  M(194) = c(28);
  M(42)  = c(29);  M(125) = c(29);  M(189) = c(29);
  M(43)  = c(30);  M(127) = c(30);  M(191) = c(30);
  M(44)  = c(31);  M(128) = c(31);  M(192) = c(31);
  M(47)  = c(32);  M(131) = c(32);  M(195) = c(32);
  M(51)  = c(33);  M(135) = c(33);  M(198) = c(33);
  M(50)  = c(34);  M(134) = c(34);  M(197) = c(34);
  M(54)  = c(35);  M(137) = c(35);  M(199) = c(35);

  colEchelonForm(M, pivtol);

  Eigen::Matrix<Scalar, 10, 10> A;
  A.setZero();

  A(0, 2) =  1.000000;  A(1, 4) =  1.000000;  A(2, 5) =  1.000000;
  A(3, 7) =  1.000000;  A(4, 8) =  1.000000;  A(5, 9) =  1.000000;
  A(6, 0) = -M(19, 9);  A(6, 1) = -M(18, 9);  A(6, 2) = -M(17, 9);
  A(6, 3) = -M(16, 9);  A(6, 4) = -M(15, 9);  A(6, 5) = -M(14, 9);
  A(6, 6) = -M(13, 9);  A(6, 7) = -M(12, 9);  A(6, 8) = -M(11, 9);
  A(6, 9) = -M(10, 9);  A(7, 0) = -M(19, 8);  A(7, 1) = -M(18, 8);
  A(7, 2) = -M(17, 8);  A(7, 3) = -M(16, 8);  A(7, 4) = -M(15, 8);
  A(7, 5) = -M(14, 8);  A(7, 6) = -M(13, 8);  A(7, 7) = -M(12, 8);
  A(7, 8) = -M(11, 8);  A(7, 9) = -M(10, 8);  A(8, 0) = -M(19, 7);
  A(8, 1) = -M(18, 7);  A(8, 2) = -M(17, 7);  A(8, 3) = -M(16, 7);
  A(8, 4) = -M(15, 7);  A(8, 5) = -M(14, 7);  A(8, 6) = -M(13, 7);
  A(8, 7) = -M(12, 7);  A(8, 8) = -M(11, 7);  A(8, 9) = -M(10, 7);
  A(9, 0) = -M(19, 6);  A(9, 1) = -M(18, 6);  A(9, 2) = -M(17, 6);
  A(9, 3) = -M(16, 6);  A(9, 4) = -M(15, 6);  A(9, 5) = -M(14, 6);
  A(9, 6) = -M(13, 6);  A(9, 7) = -M(12, 6);  A(9, 8) = -M(11, 6);
  A(9, 9) = -M(10, 6);

  const Eigen::EigenSolver<Eigen::Matrix<Scalar, 10, 10> > eig(A);
  Eigen::Matrix<std::complex<Scalar>, 10, 2> esols;
  esols.col(0).array() = eig.eigenvectors().row(2).array() / eig.eigenvectors().row(0).array();
  esols.col(1).array() = eig.eigenvectors().row(1).array() / eig.eigenvectors().row(0).array();

  int nsols = 0;
  for(Eigen::Index i = 0; i < 10; ++i)
  {
    if(esols.row(i).imag().isZero(100.0 * std::numeric_limits<Scalar>::epsilon()))
      sols.col(nsols++) = esols.row(i).real();
  }

  return nsols;
}

/**
 * @brief Computes the relative pose and two radial disortion coefficients for two cameras from 10 correspondences
 *        Epipolar geometry F + l1 + l2 solver F10e:
 * @code{.unparsed}
 *                          [F11 F12 F13] [u]
 * [x, y, 1+l1*(x^2+y^2)] * [F21 F22 F23] [v]               = 0
 *                          [F31 F32 F33] [1+l2*(u^2+v^2)]
 * @endcode
 * @param[in] X 10x2 matrix of 2D distorted measurements (X == [x y]')
 * @param[in] U 10x2 matrix of 2D distorted measurements (U == [u v]')
 * @param[out] F list of candidate fundamental matrices solutions
 * @param[out] L list of candidate radial disortion solutions (L[i] = [l1 l2]').
 * @return nsols
 *
 * @author	Zuzana Kukelova, Jan Heller, Martin Bujnak, Andrew Fitzgibbon, Tomas Pajdla, adapted to aliceVision by Michal Polic
 * @ref [1]	Zuzana Kukelova, Jan Heller, Martin Bujnak, Andrew Fitzgibbon, Tomas Pajdla:
 *			Efficient Solution to the Epipolar Geometry for Radially Distorted Cameras,
 *			The IEEE International Conference on Computer Vision (ICCV),
 *			December, 2015, Santiago, Chile.
 */
int F10RelativePose(const Mat& X, const Mat& U, std::vector<Fundamental10PModel>& models)
{
  using Mat21 = Eigen::Matrix<double, 2, 1>;

  assert((X.rows() == 10 && X.cols() == 2) &&  "The first parameter (x) must be a 10x2 matrix");
  assert((U.rows() == 10 && U.cols() == 2) && "The second parameter (u) must be a 10x2 matrix");

  Eigen::Matrix<double, 10, 1> Z1;
  Eigen::Matrix<double, 10, 1> Z2;
  Eigen::Matrix<double, 10, 16> A;

  Z1.array() = X.col(0).array() * X.col(0).array() + X.col(1).array() * X.col(1).array();
  Z2.array() = U.col(0).array() * U.col(0).array() + U.col(1).array() * U.col(1).array();

  A.col(0).array()  = X.col(0).array() * U.col(0).array();
  A.col(1).array()  = X.col(0).array() * U.col(1).array();
  A.col(2).array()  = X.col(1).array() * U.col(0).array();
  A.col(3).array()  = X.col(1).array() * U.col(1).array();
  A.col(4).array()  = U.col(0).array() * Z1.array();
  A.col(5).array()  = U.col(0).array();
  A.col(6).array()  = U.col(1).array() * Z1.array();
  A.col(7).array()  = U.col(1).array();
  A.col(8).array()  = X.col(0).array() * Z2.array();
  A.col(9).array()  = X.col(0).array();
  A.col(10).array() = X.col(1).array() * Z2.array();
  A.col(11).array() = X.col(1).array();
  A.col(12).array() = Z1.array() * Z2.array();
  A.col(13).array() = Z1.array();
  A.col(14).array() = Z2.array();
  A.col(15).fill(1.0);

  const Eigen::Matrix<double, 10, 6> mr = A.block<10, 10>(0, 0).lu().solve(A.block<10, 6>(0, 10));

  Eigen::Matrix<double, 29, 1> params;

  params << mr(5, 0), mr(5, 1), -mr(4, 0), -mr(4, 1), mr(5, 2), mr(5, 3), mr(5, 4) - mr(4, 2),
            mr(5, 5) - mr(4, 3), -mr(4, 4), -mr(4, 5),
            mr(7, 0), mr(7, 1), -mr(6, 0), -mr(6, 1), mr(7, 2), mr(7, 3), mr(7, 4) - mr(6, 2),
            mr(7, 5) - mr(6, 3), -mr(6, 4), -mr(6, 5),
            mr(9, 0), mr(9, 1) - mr(8, 0), -mr(8, 1), mr(9, 2), mr(9, 4), mr(9, 3) - mr(8, 2),
            -mr(8, 3), mr(9, 5) - mr(8, 4), -mr(8, 5);

  Mat Ls(2, 10);
  const int nsols = f10e_gb(params, Ls);

  if(nsols == 0)
    return 0;

  Eigen::Matrix<double, 10, 1> b;

  b << mr(5, 0), mr(5, 1), -mr(4, 0), -mr(4, 1), mr(5, 2), mr(5, 3),
       mr(5, 4) - mr(4, 2), mr(5, 5) - mr(4, 3), -mr(4, 4), -mr(4, 5);

  models.reserve(nsols);

  for(Eigen::Index i = 0; i < nsols; ++i)
  {
    const double l1 = Ls(0, i);
    const double l2 = Ls(1, i);
    const double l1l1 = l1 * l1;
    const double l1l2 = l1 * l2;

    const Eigen::Matrix<double, 4, 1> m1 = (Eigen::Matrix<double, 4, 1>() << l1l2, l1, l2, 1).finished();
    const Eigen::Matrix<double, 6, 1> m2 = (Eigen::Matrix<double, 6, 1>() << l1l2 * l1, l1l1, l1l2, l1, l2, 1).finished();

    const double f23 = -b.block<6, 1>(4, 0).dot(m2) / b.block<4, 1>(0, 0).dot(m1);

    const Eigen::Matrix<double, 6, 1> m3 = (Eigen::Matrix<double, 6, 1>() << l2 * f23, f23, l1l2, l1, l2, 1).finished();

    Mat3 F;
    Mat21 L;

    L <<	l1, l2;
    F <<	m3.dot(-mr.row(0)), m3.dot(-mr.row(1)), m3.dot(-mr.row(9)),
          m3.dot(-mr.row(2)), m3.dot(-mr.row(3)), f23,
          m3.dot(-mr.row(5)), m3.dot(-mr.row(7)), 1;

    models.emplace_back(F, L);
  }

  return nsols;
}

void Fundamental10PSolver::solve(const Mat& x1, const Mat& x2, std::vector<Fundamental10PModel>& models) const
{
  Mat x1_ = x1;
  x1_.transposeInPlace();
  Mat x2_ = x2;
  x2_.transposeInPlace();
  F10RelativePose(x1_, x2_, models);
}

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
