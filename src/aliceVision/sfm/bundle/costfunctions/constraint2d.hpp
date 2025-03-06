// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/Observation.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsLift.hpp>
#include <ceres/rotation.h>
#include "dynamic_cost_function_to_functor.h"

namespace aliceVision {
namespace sfm {

struct Constraint2dErrorFunctor
{
    explicit Constraint2dErrorFunctor(const sfmData::Observation& obs1, 
                                            const sfmData::Observation& obs2, 
                                            const std::shared_ptr<camera::IntrinsicBase>& intrinsics)        
    : _intrinsicLiftFunctor(new CostIntrinsicsLift(obs1.getCoordinates(), intrinsics)),
    _intrinsicProjectFunctor(new CostIntrinsicsProject(obs2, intrinsics))
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_undistortion = parameters[2];
        const T* parameter_pose1 = parameters[3];
        const T* parameter_pose2 = parameters[4];

        
        Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone;
        ceres::AngleAxisToRotationMatrix(parameter_pose1, oneRo.data());
        ceres::AngleAxisToRotationMatrix(parameter_pose2, twoRo.data());
        twoRone = twoRo * oneRo.transpose();

        Eigen::Vector<T, 3> lifted;
        const T * liftParameters[3];
        liftParameters[0] = parameter_intrinsics;
        liftParameters[1] = parameter_distortion;
        liftParameters[2] = parameter_undistortion;
        _intrinsicLiftFunctor(liftParameters, lifted.data());
        Eigen::Vector<T, 3> transformed = twoRone * lifted;

        const T * projectParameters[4];
        projectParameters[0] = parameter_intrinsics;
        projectParameters[1] = parameter_distortion;
        projectParameters[2] = parameter_undistortion;
        projectParameters[3] = transformed.data();
        return _intrinsicProjectFunctor(projectParameters, residuals);
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicLiftFunctor;
    ceres::DynamicCostFunctionToFunctorTmp _intrinsicProjectFunctor;
};

}  // namespace sfm
}  // namespace aliceVision
