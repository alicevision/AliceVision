// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <Eigen/Dense>
#include <aliceVision/geometry/Pose3.hpp>

namespace aliceVision {
namespace geometry {

Eigen::Vector4d getParallaxRepresentationFromPoint(const geometry::Pose3 & posePrimary, const geometry::Pose3 & poseSecondary, const Eigen::Vector3d & point);

Eigen::Vector3d getPointFromParallaxRepresentation(const geometry::Pose3 & posePrimary, const geometry::Pose3 & poseSecondary, const Eigen::Vector4d & parallax);

}
}