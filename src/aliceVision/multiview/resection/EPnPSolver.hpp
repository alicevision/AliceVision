// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

class EPnPSolver : public robustEstimation::ISolver<robustEstimation::Mat34Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return /*5*/ 6;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 1;
  }

  /**
   * @brief Solve the problem of camera pose.
   *
   * @param[in] x2d 2d points in the first image.  One per column.
   * @param[in] x3d Corresponding 3d points in the second image. One per column.
   * @param[out] models A list of solutions.
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models) const override;

  /**
   * @brief Solve the problem.
   *
   * @param[in]  x2d 2d points in the first image. One per column.
   * @param[in]  x3d Corresponding 3d points in the second image. One per column.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models, const std::vector<double>& weights) const override
  {
     solve(x2d, x3d, models);
  }

  /**
   * @brief Computes the extrinsic parameters, R and t for a calibrated camera from 4 or
   *        more 3D points and their images.
   *
   * @param[in] x2d Image points in normalized camera coordinates, e.g. x2d = inv(K) * x_image
   * @param[in] x3d 3D points in the world coordinate system
   * @param[out] R Solution for the camera rotation matrix
   * @param[out] t Solution for the camera translation vector

   * @see "{EPnP: An Accurate $O(n)$ Solution to the PnP Problem", by V. Lepetit
   *       and F. Moreno-Noguer and P. Fua, IJCV 2009. vol. 81, no. 2
   *
   * @note: the non-linear optimization is not implemented here.
   */
  bool resection(const Mat2X& x2d, const Mat3X& x3d, Mat3* R, Vec3* t) const;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
