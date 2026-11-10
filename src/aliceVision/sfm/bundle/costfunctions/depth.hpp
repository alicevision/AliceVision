// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfm/bundle/costfunctions/utils.hpp>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

/**
 * Compare the "measured" depth with the estimated point Z position in camera's space
*/
struct DepthErrorFunctor
{
    explicit DepthErrorFunctor(double depth, double depthStdDev) 
    : _depth(depth), _depthStdDev(depthStdDev)      
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_pose = parameters[0];
        const T* parameter_point = parameters[1];

        // Apply external parameters (Pose)
        T transformedPoint[3];
        applyPose(parameter_pose, parameter_point, transformedPoint);

        residuals[0] = T(1.0 / _depthStdDev) * (transformedPoint[2] - T(_depth));

        return true;
    }

    /**
     * @brief Create the appropriate cost function
     * @param[in] observation The corresponding observation
     * @param[in] depthStdDev Standard deviation of the measured depth (the residual is divided by it)
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
        const sfmData::Observation& observation, double depthStdDev)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<DepthErrorFunctor>(new DepthErrorFunctor(observation.getDepth(), depthStdDev));

        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(1);

        return costFunction;
    }
    
    const double _depth;
    const double _depthStdDev;
};

}  // namespace sfm
}  // namespace aliceVision
