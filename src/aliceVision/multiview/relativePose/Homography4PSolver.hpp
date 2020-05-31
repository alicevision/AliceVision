// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

class Homography4PSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 4;
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
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("Homography4PSolver does not support problem solving with weights.");
   }
};


class Homography4PSphericalSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
public:
    /**
     * @brief Return the minimum number of required samples
     * @return minimum number of required samples
     */
    inline std::size_t getMinimumNbRequiredSamples() const override { return 4; }

    /**
     * @brief Return the maximum number of models
     * @return maximum number of models
     */
    inline std::size_t getMaximumNbModels() const override { return 1; }

    /**
     * @brief Computes the homography that transforms p1 to p2 with the Direct Linear
     * Transform (DLT).
     *
     * @param[in] p1  A 2xN matrix of column vectors.
     * @param[in] p2  A 2xN matrix of column vectors.
     * @param[out] models A vector into which the computed homography is stored.
     *
     * The estimated homography should approximately hold the condition y = H x.
     */
    void solve(const Mat& p1, const Mat& p2, std::vector<robustEstimation::Mat3Model>& models) const override;

    /**
     * @brief Solve the problem.
     * @param[in]  p1  A 2xN matrix of column vectors.
     * @param[in]  p2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
     * @param[out] models A vector into which the computed models are stored.
     * @param[in]  weights.
     */
    void solve(const Mat& p1, const Mat& p2, std::vector<robustEstimation::Mat3Model>& models,
               const std::vector<double>& weights) const override
    {
        throw std::logic_error("Homography4PSphericalSolver does not support problem solving with weights.");
    }
};

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
