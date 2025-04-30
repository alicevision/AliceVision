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

    /**
     * @brief Create the appropriate cost functor according the provided input camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] observation The corresponding observation
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
        const std::shared_ptr<camera::IntrinsicBase> intrinsic, const sfmData::Observation& observation)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionSimpleErrorFunctor>(new ProjectionSimpleErrorFunctor(observation, intrinsic));

        int distortionSize = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                distortionSize = distortion->getParameters().size();
            }
        }

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(distortionSize);
        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(2);

        return costFunction;
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

    /**
     * @brief Create the appropriate cost functor according the provided input camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] point the  reference point to compare to
     * @param[in] observation The corresponding observation
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
                                            const std::shared_ptr<camera::IntrinsicBase> intrinsic, 
                                            const Vec3 & point,
                                            const sfmData::Observation& observation)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionSurveyErrorFunctor>(new ProjectionSurveyErrorFunctor(point, observation, intrinsic));

        int distortionSize = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                distortionSize = distortion->getParameters().size();
            }
        }

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(distortionSize);
        costFunction->AddParameterBlock(6);
        costFunction->SetNumResiduals(2);

        return costFunction;
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

    /**
     * @brief Create the appropriate cost functor according the provided input rig camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] observation The corresponding observation
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(std::shared_ptr<camera::IntrinsicBase> intrinsic, 
                                                        const sfmData::Observation& observation)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionErrorFunctor>(new ProjectionErrorFunctor(observation, intrinsic));

        int distortionSize = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                distortionSize = distortion->getParameters().size();
            }
        }

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(distortionSize);
        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(2);

        return costFunction;
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
};

struct ProjectionRelativeErrorFunctor
{
    explicit ProjectionRelativeErrorFunctor(const sfmData::Observation& obs
    , const std::shared_ptr<camera::IntrinsicBase>& intrinsics
    , bool noPose)        
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics))
    , withoutPose(noPose)
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        
        
        T transformedPoint[3];
        if (withoutPose)
        {
            const T* parameter_relativepoint = parameters[2];
            transformedPoint[0] = parameter_relativepoint[0] / parameter_relativepoint[2];
            transformedPoint[1] = parameter_relativepoint[1] / parameter_relativepoint[2];
            transformedPoint[2] = 1.0 / parameter_relativepoint[2];
        }
        else
        {
            const T* parameter_pose = parameters[2];
            const T* parameter_refpose = parameters[3];
            const T* parameter_relativepoint = parameters[4];

            //From inverse depth to cartesian
            T relpoint[3];
            relpoint[0] = parameter_relativepoint[0] / parameter_relativepoint[2];
            relpoint[1] = parameter_relativepoint[1] / parameter_relativepoint[2];
            relpoint[2] = 1.0 / parameter_relativepoint[2];

            const T* refcam_R = parameter_refpose;
            const T* refcam_t = &parameter_refpose[3];
            
            // Apply inverse reference transformation to cartesian landmark
            relpoint[0] = relpoint[0] - refcam_t[0];
            relpoint[1] = relpoint[1] - refcam_t[1];
            relpoint[2] = relpoint[2] - refcam_t[2];

            T invrefcam_R[3];
            invrefcam_R[0] = -refcam_R[0];
            invrefcam_R[1] = -refcam_R[1];
            invrefcam_R[2] = -refcam_R[2];

            T absolutePoint[3];  
            ceres::AngleAxisRotatePoint(invrefcam_R, relpoint, absolutePoint);

            //--
            // Apply external parameters (Pose)
            //--
            const T* cam_R = parameter_pose;
            const T* cam_t = &parameter_pose[3];
            
            
            // Rotate the point according the camera rotation
            ceres::AngleAxisRotatePoint(cam_R, absolutePoint, transformedPoint);

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

    /**
     * @brief Create the appropriate cost functor according the provided input camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] observation The corresponding observation
     * @param[in] withoutPose When reference view and current view are the same, poses must be ignored
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
        const std::shared_ptr<camera::IntrinsicBase> intrinsic, 
        const sfmData::Observation& observation,
        bool withoutPose)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionRelativeErrorFunctor>(new ProjectionRelativeErrorFunctor(observation, intrinsic, withoutPose));

        int distortionSize = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                distortionSize = distortion->getParameters().size();
            }
        }

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(distortionSize);

        if (!withoutPose)
        {
            costFunction->AddParameterBlock(6);
            costFunction->AddParameterBlock(6);
        }

        costFunction->AddParameterBlock(3);
        costFunction->SetNumResiduals(2);

        return costFunction;
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
    bool withoutPose;
};


}  // namespace sfm
}  // namespace aliceVision
