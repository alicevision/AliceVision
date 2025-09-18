// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "poseNoise.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/geometry/Pose3.hpp>

namespace aliceVision {
namespace sfm {

bool addPoseNoise(sfmData::SfMData& sfmData, const float positionNoise, const float rotationNoise)
{
    if (positionNoise == 0. && rotationNoise == 0.)
    {
        ALICEVISION_LOG_INFO("poseNoise::nothing to do");
        return true;
    }

    ALICEVISION_LOG_INFO("poseNoise::process start");

    using namespace Eigen;

    std::mt19937 gen;
    std::uniform_real_distribution<double> dist(-1., 1.);

    for (const auto& [viewID, viewPtr] : sfmData.getViews())
    {
        const sfmData::CameraPose viewPose = sfmData.getPose(*viewPtr);
        Vector3d viewCenter = viewPose.getTransform().center();
        Matrix3d viewRotation = viewPose.getTransform().rotation();

        // Add noise to view position
        if (positionNoise != 0.)
        {
            viewCenter = viewCenter + positionNoise * Vector3d::Random();
        }

        // Add noise to view orientation
        if (rotationNoise != 0.)
        {
            AngleAxisd aa(viewRotation);
            double viewRotAngle = aa.angle();
            Vector3d viewRotAxis = aa.axis();

            // Add noise to the rotation angle
            viewRotAngle = viewRotAngle + rotationNoise * M_PI * dist(gen);

            // Apply a random rotation to the rotation axis
            // (the random rotation axis coordinates do not need to be weighted by rotationNoise)
            AngleAxisd randomRot(rotationNoise * M_PI * dist(gen), Vector3d(dist(gen), dist(gen), dist(gen)).normalized());
            viewRotAxis = randomRot.toRotationMatrix() * viewRotAxis;

            aa = AngleAxisd(viewRotAngle, viewRotAxis);
            viewRotation = aa.toRotationMatrix();
        }

        geometry::Pose3 newPose(viewRotation, viewCenter);
        sfmData.setPose(*viewPtr, sfmData::CameraPose(newPose));
    }

    ALICEVISION_LOG_INFO("poseNoise::process end");
    return true;
}
} // namespace sfm
} // namespace aliceVision
