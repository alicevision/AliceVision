// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "augmentedNormals.hpp"

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

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
 * @brief Lighting estimation from picture, albedo and geometry
 */ 
void estimateLigthing(LightingVector& lighting, const image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, const image::Image<image::RGBfColor>& normals);


}
}
