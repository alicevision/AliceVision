// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <ceres/rotation.h>
#include "dynamic_cost_function_to_functor.h"

#include <memory>

// Define ceres Cost_functor for each AliceVision camera model

namespace aliceVision {
namespace sfm {

struct ProjectionSimpleErrorFunctor
{
    explicit ProjectionSimpleErrorFunctor(const sfmData::Observation& obs, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)        
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics))
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_pose = parameters[2];
        const T* parameter_point = parameters[3];

        //--
        // Apply external parameters (Pose)
        //--
        const T* cam_R = parameter_pose;
        const T* cam_t = &parameter_pose[3];
        
        T transformedPoint[3];
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(cam_R, parameter_point, transformedPoint);

        // Apply the camera translation
        transformedPoint[0] += cam_t[0];
        transformedPoint[1] += cam_t[1];
        transformedPoint[2] += cam_t[2];

        const T * innerParameters[3];
        innerParameters[0] = parameter_intrinsics;
        innerParameters[1] = parameter_distortion;
        innerParameters[2] = transformedPoint;

        return _intrinsicFunctor(innerParameters, residuals);
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
};


struct ProjectionSurveyErrorFunctor
{
    explicit ProjectionSurveyErrorFunctor(const Vec3 & point, 
                                            const sfmData::Observation& obs, 
                                            const std::shared_ptr<camera::IntrinsicBase>& intrinsics)        
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics)), _point(point)
    {        
    }

    template<typename T>
    T func(const T & input) const 
    {
        const T alpha = T(5.0);
        const T coeff = T(0.05);

        const T p1 = abs(alpha - T(2));
        const T p2 = coeff * input * input;
        
        return (p1/alpha) * (pow(T(1) + p2/p1, alpha / T(2)) - T(1));
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_pose = parameters[2];
        T parameter_point[3];
        
        parameter_point[0] = T(_point.x());
        parameter_point[1] = T(_point.y());
        parameter_point[2] = T(_point.z());

        //--
        // Apply external parameters (Pose)
        //--
        const T* cam_R = parameter_pose;
        const T* cam_t = &parameter_pose[3];
        
        T transformedPoint[3];
        // Rotate the point according the camera rotation
        ceres::AngleAxisRotatePoint(cam_R, parameter_point, transformedPoint);

        // Apply the camera translation
        transformedPoint[0] += cam_t[0];
        transformedPoint[1] += cam_t[1];
        transformedPoint[2] += cam_t[2];

        const T * innerParameters[3];
        innerParameters[0] = parameter_intrinsics;
        innerParameters[1] = parameter_distortion;
        innerParameters[2] = transformedPoint;

        if (!_intrinsicFunctor(innerParameters, residuals))
        {
            return false;
        }

        residuals[0] = func(residuals[0]);
        residuals[1] = func(residuals[1]);

        return true;
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
    Vec3 _point;
};

struct ProjectionErrorFunctor
{
    explicit ProjectionErrorFunctor(const sfmData::Observation& obs, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)        
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics))
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_pose = parameters[2];
        const T* parameter_subpose = parameters[3];
        const T* parameter_point = parameters[4];

        T transformedPoint[3];
        {
            const T* cam_R = parameter_pose;
            const T* cam_t = &parameter_pose[3];

            // Rotate the point according the camera rotation
            ceres::AngleAxisRotatePoint(cam_R, parameter_point, transformedPoint);

            // Apply the camera translation
            transformedPoint[0] += cam_t[0];
            transformedPoint[1] += cam_t[1];
            transformedPoint[2] += cam_t[2];
        }

        {
            const T* cam_R = parameter_subpose;
            const T* cam_t = &parameter_subpose[3];

            // Rotate the point according to the camera rotation
            T transformedPointBuf[3] = {transformedPoint[0], transformedPoint[1], transformedPoint[2]};
            ceres::AngleAxisRotatePoint(cam_R, transformedPointBuf, transformedPoint);

            // Apply the camera translation
            transformedPoint[0] += cam_t[0];
            transformedPoint[1] += cam_t[1];
            transformedPoint[2] += cam_t[2];
        }

        const T * innerParameters[3];
        innerParameters[0] = parameter_intrinsics;
        innerParameters[1] = parameter_distortion;
        innerParameters[2] = transformedPoint;

        return _intrinsicFunctor(innerParameters, residuals);
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
};


}  // namespace sfm
}  // namespace aliceVision
