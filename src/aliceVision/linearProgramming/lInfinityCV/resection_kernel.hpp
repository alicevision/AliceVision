// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <vector>

namespace aliceVision {
namespace lInfinityCV {
namespace kernel {

/**
 * Six-point resection
 * P Matrix estimation (Pose estimation)
 * Rely on L1 Resection algorithm.
 * Work from 6 to N points.
 */
struct l1SixPointResectionSolver : public robustEstimation::ISolver<robustEstimation::Mat34Model>
{
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
   * @brief Solve the problem of camera pose.
   * @note First 3d point will be translated in order to have X0 = (0,0,0,1)
   */
  void solve(const Mat& pt2D, const Mat& pt3D, std::vector<robustEstimation::Mat34Model>& P) const;

  /**
   * @brief Solve the problem.
   * @param[in]  x1  A 2xN matrix of column vectors.
   * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat34Model>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("l1SixPointResectionSolver does not support problem solving with weights.");
   }

};

/**
 * @brief Usable solver for the l1 6pt Resection Estimation
 */
using l1PoseResectionKernel = robustEstimation::PointFittingKernel<l1SixPointResectionSolver, multiview::resection::ProjectionDistanceError, robustEstimation::Mat34Model>;

}  // namespace kernel
}  // namespace lInfinityCV
}  // namespace aliceVision
