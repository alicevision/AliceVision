// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PosesContainer.hpp"

#include <aliceVision/sfmData/CameraPose.hpp>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

PosesContainer::PosesContainer()
    : _manifoldCRDC(6, std::vector<int>{3, 4, 5}),
      _manifoldDRCC(6, std::vector<int>{0, 1, 2})
{
}

std::vector<double*> PosesContainer::getAllParams()
{
    std::vector<double*> allParams;
    for (auto& [_, info] : _poses)
    {
        double* ptr = info.params.data();
        allParams.push_back(ptr);
    }

    return allParams;
}

void PosesContainer::fill(const sfmData::SfMData & sfmData, bool forceRotationConstant, bool forceCenterConstant)
{
    _poses.clear();

    for (const auto& [id, pose] : sfmData.getPoses().valueRange())
    {
        // skip camera pose set as Ignored in the Local strategy
        if (pose.getState() == EEstimatorParameterState::IGNORED)
        {
            continue;
        }

        PoseInfo info;

        const Mat3 & R = pose.getTransform().rotation();
        const Vec3 & c = pose.getTransform().center();

        double angleAxis[3];
        ceres::RotationMatrixToAngleAxis(static_cast<const double*>(R.data()), angleAxis);

        info.params[0] = angleAxis[0];
        info.params[1] = angleAxis[1];
        info.params[2] = angleAxis[2];
        info.params[3] = c(0);
        info.params[4] = c(1);
        info.params[5] = c(2);
        info.isRotationConstant = (pose.getState() == EEstimatorParameterState::CONSTANT) || forceRotationConstant;
        info.isCenterConstant = (pose.getState() == EEstimatorParameterState::CONSTANT) || forceCenterConstant;

        _poses[id] = info;
    }
}

void PosesContainer::apply(ceres::Problem & problem)
{
    for (auto& [id, info] : _poses)
    {
        double* ptr = info.params.data();
        
        problem.AddParameterBlock(ptr, 6);

        if (info.isRotationConstant && !info.isCenterConstant)
        {
            problem.SetManifold(ptr, &_manifoldCRDC);
        }
        else if (!info.isRotationConstant && info.isCenterConstant)
        {
            problem.SetManifold(ptr, &_manifoldDRCC);
        }
        else
        {
            problem.SetParameterBlockConstant(ptr);
        }
    }
}

void PosesContainer::update(sfmData::SfMData & sfmData)
{
    for (const auto& [id, pose] : sfmData.getPoses().valueRange())
    {
        auto it = _poses.find(id);
        if (it == _poses.end())
        {
            continue;
        }

        PoseInfo & info = it->second;
        if (info.isCenterConstant && info.isRotationConstant)
        {
            continue;
        }

        const std::array<double, 6> & poseBlock = info.params;
        pose.updateFromEstimator(poseBlock);
    }
}

}
}