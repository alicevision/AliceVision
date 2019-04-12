// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2011 Laurent Kneip, ETH Zurich.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/ISolver.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

typedef Eigen::Matrix<double, 5, 1> Vec5;

class P3PSolver : public ISolver<Mat34Model>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 3;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 4;
  }

  /**
   * @brief Solve the problem of camera pose.
   *
   * @param[in] x2d 2d points in the first image. One per column.
   * @param[in] x3d Corresponding 3d points in the second image. One per column.
   * @param[out] models A list of at most 4 candidate solutions.
   */
   void solve(const Mat& x2d, const Mat& x3d, std::vector<Mat34Model>& models) const override;

   /**
    * @brief Solve the problem.
    *
    * @param[in]  x2d 2d points in the first image. One per column.
    * @param[in]  x3d Corresponding 3d points in the second image. One per column.
    * @param[out] models A vector into which the computed models are stored.
    * @param[in]  weights.
    */
   void solve(const Mat& x2d, const Mat& x3d, std::vector<Mat34Model>& models, const std::vector<double>& weights) const override
   {
      throw std::logic_error("P3PSolver does not support problem solving with weights.");
   }
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision








/*

class P3P_ResectionKernel_K
{
public:
  typedef Mat34 Model;

  enum
  {
    MINIMUM_SAMPLES = 3
  };

  P3P_ResectionKernel_K(const Mat2X &x_camera, const Mat3X &X, const Mat3 &K = Mat3::Identity());

  void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const;

  double Error(size_t sample, const Model &model) const;

  size_t NumSamples() const;

private:
  Mat2X x_image_; // camera coordinates
  Mat3X x_camera_; // camera coordinates (normalized)
  Mat3X X_; // 3D points
  Mat3 K_;
};
*/
