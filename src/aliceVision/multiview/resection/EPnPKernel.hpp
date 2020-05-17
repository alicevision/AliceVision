// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/resection/EPnPSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>
#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/numeric/projection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief Euclidean (EPnP) resection kernel
 * @note Have K intrinsic helps
 */
class EPnPKernel : public robustEstimation::PointFittingKernel<EPnPSolver, ProjectionDistanceError, robustEstimation::Mat34Model>
{
 public:

  using KernelBase = robustEstimation::PointFittingKernel<EPnPSolver, ProjectionDistanceError, robustEstimation::Mat34Model>;

  EPnPKernel(const Mat2X& x2d, const Mat3X& x3d)
    : KernelBase(x2d, x3d)
    , x_camera_(x2d)
    , X_(x3d)
  {
        assert(x2d.cols() == x3d.cols());
        K_ = Mat3::Identity();
  }

  EPnPKernel(const Mat2X& x2d, const Mat3X& x3d, const Mat3& K)
    : KernelBase(x2d, x3d)
    , X_(x3d), K_(K)
    {
      assert(x2d.cols() == x3d.cols());
      // Conversion from image coordinates to normalized camera coordinates
      euclideanToNormalizedCamera(x2d, K, &x_camera_);
    }

  void fit(const std::vector<std::size_t>& samples, std::vector<robustEstimation::Mat34Model>& models) const override
  {
    Mat2X x = ExtractColumns(x_camera_, samples);
    Mat3X X = ExtractColumns(X_, samples);
    Mat34 P;
    Mat3 R;
    Vec3 t;
    if (_kernelSolver.resection(x, X, &R, &t))
    {
      P_from_KRt(K_, R, t, &P);
      models.emplace_back(P);
    }
  }

 private:
  // x_camera_ contains the normalized camera coordinates
  Mat2X  x_camera_;
  const Mat3X &X_;
  Mat3 K_;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision

