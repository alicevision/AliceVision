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

using LightingVector = Eigen::Matrix<float, 9, 3>;


void estimateLigthing(LightingVector& lighting, const Image<RGBfColor>& albedo, const Image<RGBfColor>& picture, const Image<RGBfColor>& normals);


}
}
