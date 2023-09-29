// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/geometry/Parallax.hpp>

namespace aliceVision {
namespace geometry {

Eigen::Vector4d getParallaxRepresentationFromPoint(const geometry::Pose3 & posePrimary, const geometry::Pose3 & poseSecondary, const Eigen::Vector3d & point)
{
    Eigen::Vector4d ret;

    Vec3 pmj = posePrimary.center();
    Vec3 paj = poseSecondary.center();

    Vec3 posePrimary_n = posePrimary(point).normalized();

    Vec3 v1 = (pmj - point).normalized();
    Vec3 v2 = (paj - point).normalized();
    double theta = std::acos(v1.dot(v2));

    ret.block<3, 1>(0, 0) = posePrimary_n;
    ret(3) = theta;

    return ret;
}

Eigen::Vector3d getPointFromParallaxRepresentation(const geometry::Pose3 & posePrimary, const geometry::Pose3 & poseSecondary, const Eigen::Vector4d & parallax)
{
    Eigen::Vector3d ret;

    Vec3 pmj = posePrimary.center();
    Vec3 paj = poseSecondary.center();

    double theta = parallax(3);
    Vec3 posePrimary_n = parallax.block<3, 1>(0, 0);
    Vec3 world_n = posePrimary.rotation().transpose() * posePrimary_n;

    double alpha = std::acos((pmj - paj).normalized().dot(world_n));

    double coeff = sin(alpha - theta) / sin(theta);

    ret = pmj + coeff * (pmj - paj).norm() * world_n;

    return ret;
}

}
}