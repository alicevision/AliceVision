// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/multiview/projection.hpp>
#include <aliceVision/multiview/twoViewKernel.hpp>

#include <vector>

namespace aliceVision {
namespace homography {
namespace kernel {

using namespace std;

struct FourPointSolver {
  enum { MINIMUM_SAMPLES = 4 };
  enum { MAX_MODELS = 1 };
  /**
   * Computes the homography that transforms x to y with the Direct Linear
   * Transform (DLT).
   *
   * \param x  A 2xN matrix of column vectors.
   * \param y  A 2xN matrix of column vectors.
   * \param Hs A vector into which the computed homography is stored.
   *
   * The estimated homography should approximately hold the condition y = H x.
   */
  static void Solve(const Mat &x, const Mat &y, vector<Mat3> *Hs);
};

// Should be distributed as Chi-squared with k = 2.
struct AsymmetricError {
  static double Error(const Mat &H, const Vec2 &x1, const Vec2 &x2) {
    Vec3 x2h_est = H * EuclideanToHomogeneous(x1);
    Vec2 x2_est = x2h_est.head<2>() / x2h_est[2];
    return (x2 - x2_est).squaredNorm();
  }
};

// Kernel that works on original data point
typedef twoView::kernel::Kernel<FourPointSolver, AsymmetricError, Mat3>
  UnnormalizedKernel;

// By default use the normalized version for increased robustness.
typedef twoView::kernel::Kernel<
    twoView::kernel::NormalizedSolver<FourPointSolver, UnnormalizerI>,
    AsymmetricError,
    Mat3>
  Kernel;

}  // namespace kernel
}  // namespace homography
}  // namespace aliceVision
