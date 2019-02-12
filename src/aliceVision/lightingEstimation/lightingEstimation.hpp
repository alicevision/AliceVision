// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <aliceVision/lightingEstimation/augmentedNormals.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

namespace aliceVision {
namespace lightingEstimation {

using Eigen::MatrixXf;

/**
 * @brief Augmented lighting vetor for augmented Lambert's law (using Spherical Harmonics model)
 * Composed of 9 coefficients
 */ 
using LightingVector = Eigen::Matrix<float, 9, 3>;

/**
 * @brief The LighthingEstimator class
 * Allows to estimate LightingVector from a single image or multiple images
 * @warning Image pixel type can be:
 * - RGB (float) for light and color estimation
 * - Greyscale (float) for luminance estimation
 */
class LighthingEstimator
{
public:

  /**
   * @brief Aggregate image data
   * @param albedo[in] the corresponding albedo image
   * @param picture[in] the corresponding picture
   * @param normals[in] the corresponding normals image
   */
  void addImage(const image::Image<float>& albedo, const image::Image<float>& picture, const image::Image<image::RGBfColor>& normals);
  void addImage(const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals);

  /**
   * @brief Estimate ligthing from the aggregate image(s) data
   * @param[out] lighting Estimate ligthing @see LightingVector
   */
  void estimateLigthing(LightingVector& lighting) const;

  /**
   * @brief Clear all the aggregate image(s) data
   */
  void clear();

private:
  std::array<std::vector<MatrixXf>, 3> _all_rhoTimesN;
  std::array<std::vector<MatrixXf>, 3> _all_pictureChannel;
};

} // namespace lightingEstimation
} // namespace aliceVision
