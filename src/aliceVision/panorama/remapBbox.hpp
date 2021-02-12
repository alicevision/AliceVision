#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/camera.hpp>

#include "boundingBox.hpp"

namespace aliceVision
{

bool computeCoarseBB(BoundingBox& coarse_bbox, const std::pair<int, int>& panoramaSize, const geometry::Pose3& pose,
                     const aliceVision::camera::IntrinsicBase& intrinsics);

}