// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

/**
 * Compare the "measured" depth with the estimated point Z position in camera's space
*/
struct DepthErrorFunctor
{
    explicit DepthErrorFunctor(double depth, double depthVariance) 
    : _depth(depth), _depthVariance(depthVariance)      
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_pose = parameters[0];
        const T* parameter_point = parameters[1];

        //--
        // Apply external parameters (Pose)
        //--
        const T* cam_R = parameter_pose;
        const T* cam_c = &parameter_pose[3];
        
        T centeredPoint[3];
        centeredPoint[0] = parameter_point[0] - cam_c[0];
        centeredPoint[1] = parameter_point[1] - cam_c[1];
        centeredPoint[2] = parameter_point[2] - cam_c[2];

        T transformedPoint[3];
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(cam_R, centeredPoint, transformedPoint);

        residuals[0] = T(1.0 / _depthVariance) * (transformedPoint[2] - T(_depth));

        return true;
    }

    /**
     * @brief Create the appropriate cost function
     * @param[in] observation The corresponding observation
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
        const sfmData::Observation& observation, double depthVariance)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<DepthErrorFunctor>(new DepthErrorFunctor(observation.getDepth(), depthVariance));

        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(1);

        return costFunction;
    }
    
    const double _depth;
    const double _depthVariance;
};

}  // namespace sfm
}  // namespace aliceVision
