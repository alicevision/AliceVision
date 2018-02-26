// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/multiview/twoViewKernel.hpp>

#include <vector>


namespace aliceVision {
namespace fundamental {
namespace kernel {

using namespace std;

/**
 * Seven-point algorithm for solving for the fundamental matrix from point
 * correspondences. See page 281 in HZ, though oddly they use a different
 * equation: \f$det(\alpha F_1 + (1-\alpha)F_2) = 0\f$. Since \f$F_1\f$ and
 * \f$F2\f$ are projective, there's no need to balance the relative scale.
 * Instead, here, the simpler equation is solved: \f$det(F_1 + \alpha F_2) =
 * 0\f$.
 *
 * \see http://www.cs.unc.edu/~marc/tutorial/node55.html
 */
struct SevenPointSolver {
  enum { MINIMUM_SAMPLES = 7 };
  enum { MAX_MODELS = 3 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *F);
};

struct EightPointSolver {
  enum { MINIMUM_SAMPLES = 8 };
  enum { MAX_MODELS = 1 };
  static void Solve(const Mat &x1, const Mat &x2, vector<Mat3> *Fs, const vector<double> *weights = nullptr);
};

/**
 * Build a 9 x n matrix from point matches, where each row is equivalent to the
 * equation x'T*F*x = 0 for a single correspondence pair (x', x). The domain of
 * the matrix is a 9 element vector corresponding to F. In other words, set up
 * the linear system
 *
 *   Af = 0,
 *
 * where f is the F matrix as a 9-vector rather than a 3x3 matrix (row
 * major). If the points are well conditioned and there are 8 or more, then
 * the nullspace should be rank one. If the nullspace is two dimensional,
 * then the rank 2 constraint must be enforced to identify the appropriate F
 * matrix.
 *
 * Note that this does not resize the matrix A; it is expected to have the
 * appropriate size already.
 */
template<typename TMatX, typename TMatA>
inline void EncodeEpipolarEquation(const TMatX &x1, const TMatX &x2, TMatA *A, const vector<double> *weights = nullptr) 
{
  assert(x1.cols()==x2.cols());
  if(weights)
  {
    assert(x1.cols()==weights->size());
  }
  for (typename TMatX::Index i = 0; i < x1.cols(); ++i) 
  {
    const Vec2 xx1 = x1.col(i);
    const Vec2 xx2 = x2.col(i);
    A->row(i) <<
      xx2(0) * xx1(0),  // 0 represents x coords,
      xx2(0) * xx1(1),  // 1 represents y coords.
      xx2(0),
      xx2(1) * xx1(0),
      xx2(1) * xx1(1),
      xx2(1),
      xx1(0),
      xx1(1),
      1.0;
    if(weights)
      A->row(i) *= (*weights)[i];
  }
}

/// Compute SampsonError related to the Fundamental matrix and 2 correspondences
struct SampsonError {
  static double Error(const Mat3 &F, const Vec2 &x1, const Vec2 &x2) {
    Vec3 x(x1(0), x1(1), 1.0);
    Vec3 y(x2(0), x2(1), 1.0);
    // See page 287 equation (11.9) of HZ.
    Vec3 F_x = F * x;
    Vec3 Ft_y = F.transpose() * y;
    return Square(y.dot(F_x)) / (  F_x.head<2>().squaredNorm()
                                + Ft_y.head<2>().squaredNorm());
  }
};

struct SymmetricEpipolarDistanceError {
  static double Error(const Mat3 &F, const Vec2 &x1, const Vec2 &x2) {
    Vec3 x(x1(0), x1(1), 1.0);
    Vec3 y(x2(0), x2(1), 1.0);
    // See page 288 equation (11.10) of HZ.
    Vec3 F_x = F * x;
    Vec3 Ft_y = F.transpose() * y;
    return Square(y.dot(F_x)) * ( 1.0 / F_x.head<2>().squaredNorm()
                                + 1.0 / Ft_y.head<2>().squaredNorm())
      / 4.0;  // The divide by 4 is to make this match the Sampson distance.
  }
};

struct EpipolarDistanceError {
  static double Error(const Mat3 &F, const Vec2 &x1, const Vec2 &x2) {
    // Transfer error in image 2
    // See page 287 equation (11.9) of HZ.
    Vec3 x(x1(0), x1(1), 1.0);
    Vec3 y(x2(0), x2(1), 1.0);
    Vec3 F_x = F * x;
    return Square(F_x.dot(y)) /  F_x.head<2>().squaredNorm();
  }
};
typedef EpipolarDistanceError SimpleError;

//-- Kernel solver for the 8pt Fundamental Matrix Estimation
typedef twoView::kernel::Kernel<SevenPointSolver, SampsonError, Mat3>
  SevenPointKernel;

//-- Kernel solver for the 8pt Fundamental Matrix Estimation
typedef twoView::kernel::Kernel<EightPointSolver, SampsonError, Mat3>
  EightPointKernel;

//-- Normalized 7pt kernel -> conditioning from HZ (Algo 11.1) pag 282
typedef twoView::kernel::Kernel<
  twoView::kernel::NormalizedSolver<SevenPointSolver, UnnormalizerT>,
  SampsonError,
  Mat3>
  NormalizedSevenPointKernel;

//-- Normalized 8pt kernel -> conditioning from HZ (Algo 11.1) pag 282
typedef twoView::kernel::Kernel<
  twoView::kernel::NormalizedSolver<EightPointSolver, UnnormalizerT>,
  SampsonError,
  Mat3>
  NormalizedEightPointKernel;

}  // namespace kernel
}  // namespace fundamental
}  // namespace aliceVision
