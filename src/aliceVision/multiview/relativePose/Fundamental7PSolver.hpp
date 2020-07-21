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

class Fundamental7PSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 7;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 3;
  }

  /**
   * @brief Seven-point algorithm for solving for the fundamental matrix from point
   *        correspondences. @see page 281 in HZ
   *
   * Though oddly they use a different equation: \f$det(\alpha F_1 + (1-\alpha)F_2) = 0\f$.
   * Since \f$F_1\f$ and \f$F2\f$ are projective, there's no need to balance the relative scale.
   * Instead, here, the simpler equation is solved: \f$det(F_1 + \alpha F_2) = 0\f$.
   *
   * @see http://www.cs.unc.edu/~marc/tutorial/node55.html
   *
   * @param[in] x1 Points in the first image.  One per column.
   * @param[in] x2 Corresponding points in the second image. One per column.
   * @param[out] models  A list of at most 10 candidate essential matrix solutions.
   */
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const override;

   /**
    * @brief Solve the problem.
    * @param[in]  x1  A 2xN matrix of column vectors.
    * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
    * @param[out] models A vector into which the computed models are stored.
    * @param[in]  weights.
    */
   void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("Fundamental7PSolver does not support problem solving with weights.");
   }
};


class Fundamental7PSphericalSolver : public robustEstimation::ISolver<robustEstimation::Mat3Model>
{
public:
    /**
     * @brief Return the minimum number of required samples
     * @return minimum number of required samples
     */
    inline std::size_t getMinimumNbRequiredSamples() const override { return 7; }

    /**
     * @brief Return the maximum number of models
     * @return maximum number of models
     */
    inline std::size_t getMaximumNbModels() const override { return 3; }

    /**
     * @brief Seven-point algorithm for solving for the fundamental matrix from point
     *        correspondences. @see page 281 in HZ
     *
     * Though oddly they use a different equation: \f$det(\alpha F_1 + (1-\alpha)F_2) = 0\f$.
     * Since \f$F_1\f$ and \f$F2\f$ are projective, there's no need to balance the relative scale.
     * Instead, here, the simpler equation is solved: \f$det(F_1 + \alpha F_2) = 0\f$.
     *
     * @see http://www.cs.unc.edu/~marc/tutorial/node55.html
     *
     * @param[in] x1 Points in the first image.  One per column.
     * @param[in] x2 Corresponding points in the second image. One per column.
     * @param[out] models  A list of at most 10 candidate essential matrix solutions.
     */
    void solve(const Mat& x1, const Mat& x2, std::vector<robustEstimation::Mat3Model>& models) const override;

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
        throw std::logic_error("Fundamental7PSphericalSolver does not support problem solving with weights.");
    }
};

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
