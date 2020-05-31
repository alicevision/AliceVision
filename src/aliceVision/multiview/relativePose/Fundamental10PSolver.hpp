// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (c) 2015 Jan Heller <hellej1@cmp.felk.cvut.cz>, Zuzana Kukelova <zukuke@microsoft.com>
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace relativePose {

/***
 * @brief Model for relative pose and two radial distortion coefficients.
 */
struct Fundamental10PModel : public robustEstimation::Mat3Model
{
  using Mat21 = Eigen::Matrix<double, 2, 1>;

  Fundamental10PModel() = default;

  /**
   * @brief Constructor
   * @param[in] F the 3x3 fundamental matrix.
   * @param[in] L a 2x1 matrix containing the two radial distortion parameters.
   */
  Fundamental10PModel(const Mat3& F, const Mat21& L)
    : robustEstimation::Mat3Model(F) , _L(L)
  {}

  /**
   * @brief Get the estimated radial distortion parameters.
   * @return the radial distortion parameters as a 2x1 matrix.
   */
  inline const Mat21& getRadialDistortion() const
  {
    return _L;
  }

  /**
   * @brief Get the estimated fundamental matrix.
   * @return the fundamental matrix.
   */
  inline const Mat3& getFundamentalMatrix() const
  {
    return getMatrix();
  }

private:
  /// the two radial distortion parameters
  Mat21 _L;
};

/***
 * @brief Solver for the relative pose and two radial distortion coefficients for two cameras from 10 correspondences.
 */
class Fundamental10PSolver : public robustEstimation::ISolver<Fundamental10PModel>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 10;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 10;
  }

  /**
   * @brief Computes the relative pose and two radial distortion coefficients for two cameras from 10 correspondences.
   * @see Efficient Solution to the Epipolar Geometry for Radially Distorted Cameras,
   *			The IEEE International Conference on Computer Vision (ICCV),
   *      Zuzana Kukelova, Jan Heller, Martin Bujnak, Andrew Fitzgibbon, Tomas Pajdla
   *			December, 2015, Santiago, Chile.
   *
   * @param[in] x1 Points in the first image.  One per column.
   * @param[in] x2 Corresponding points in the second image. One per column.
   * @param[out] models  A list of at most 10 candidate essential matrix solutions.
   */
   void solve(const Mat& x1, const Mat& x2, std::vector<Fundamental10PModel>& models) const override;

   /**
    * @brief Solve the problem.
    * @param[in]  x1  A 2xN matrix of column vectors.
    * @param[in]  x2  A 2xN (relative pose) or 3xN (resection) matrix of column vectors.
    * @param[out] models A vector into which the computed models are stored.
    * @param[in]  weights.
    */
   void solve(const Mat& x1, const Mat& x2, std::vector<Fundamental10PModel>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("Fundamental10PModel does not support problem solving with weights.");
   }
};

}  // namespace relativePose
}  // namespace multiview
}  // namespace aliceVision
