// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ISolverErrorRelativePose.hpp"

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>

#include <vector>


namespace aliceVision {
namespace multiview {
namespace relativePose {


struct Rotation3PSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override { return 3; }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override { return 1; }

  /**
   * @brief Computes the homography that transforms x to y with the Direct Linear
   * Transform (DLT).
   *
   * @param[in] x  A 2xN matrix of column vectors.
   * @param[in] y  A 2xN matrix of column vectors.
   * @param[out] Hs A vector into which the computed homography is stored.
   *
   * The estimated homography should approximately hold the condition y = H x.
   */
  void solve(const Mat& x, const Mat& y, std::vector<robustEstimation::Mat3Model>& models) const override;

  /**
   * @brief Solve the problem.
   * @param[in]  x1  A 2xN matrix of column vectors.
   * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
  void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models,
             const std::vector<double>& weights) const override
  {
      throw std::logic_error("Rotation3PSolver does not support problem solving with weights.");
  }
};

struct RotationError
{
    double error(const robustEstimation::Mat3Model& R, const Vec3& p1, const Vec3& p2) const
    {
        Eigen::Vector3d p1_est = R.getMatrix() * p1;

        double sina = (p1_est.cross(p2)).norm();
        double cosa = p1_est.dot(p2);

        double angle = std::atan2(sina, cosa);

        return angle * angle;
    }
};


}  // namespace kernel
}  // namespace rotation
}  // namespace aliceVision
