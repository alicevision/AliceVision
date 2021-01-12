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

class Resection6PSolver : public robustEstimation::ISolver<robustEstimation::Mat34Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 6;
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
   * @brief Six-point resection
   *        P Matrix estimation (Pose estimation)
   *        Compute a projection matrix using linear least squares.
   *        Rely on Linear Resection algorithm.
   *        Work from 6 to N points.
   *
   * @note First 3d point will be translated in order to have X0 = (0,0,0,1)
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
   void solve(const Mat& x2d, const Mat& x3d, std::vector<robustEstimation::Mat34Model>& models, const std::vector<double>& weights) const override;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
