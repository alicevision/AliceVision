// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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
   * Computes the homography that transforms p1 to p2 with the Direct Linear
   * Transform (DLT).
   *
   * \param p1  A 2xN matrix of column vectors.
   * \param p2  A 2xN matrix of column vectors.
   * \param Hs A vector into which the computed homography is stored.
   *
   * The estimated homography should approximately hold the condition p2 = H p1.
   */
  static void Solve(const Mat &p1, const Mat &p2, vector<Mat3> *Hs);
};

// Should be distributed as Chi-squared with k = 2.
struct AsymmetricError {
  static double Error(const Mat &H, const Vec2 &p1, const Vec2 &p2) {
    Vec3 p2h_est = H * EuclideanToHomogeneous(p1);
    Vec2 p2_est = p2h_est.head<2>() / p2h_est[2];
    return (p2 - p2_est).squaredNorm();
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
