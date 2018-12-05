#pragma once

#include "augmentedNormals.hpp"

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

namespace aliceVision {
namespace inverseRendering {

using Eigen::MatrixXf;

using LightingVector = Eigen::Matrix<float, 9, 3>;


void estimateLigthing(LightingVector& lighting, const Image<RGBfColor>& albedo, const Image<RGBfColor>& picture, const Image<RGBfColor>& normals);


}
}
