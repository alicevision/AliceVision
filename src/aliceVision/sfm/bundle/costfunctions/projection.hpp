// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <aliceVision/sfm/bundle/costfunctions/utils.hpp>
#include <ceres/rotation.h>
#include "dynamic_cost_function_to_functor.h"

#include <memory>

// Define ceres Cost_functor for each AliceVision camera model

namespace aliceVision {
namespace sfm {

/**
 * @brief Reprojection error of a world point
 * Parameter blocks: intrinsics, distortion, pose, [rig sub-pose,] point
 * @tparam withSubPose true when the view is part of a rig and the rig sub-pose is applied after the pose
 */
template<bool withSubPose>
struct ProjectionErrorFunctor
{
    static constexpr int pointIndex = withSubPose ? 4 : 3;

    explicit ProjectionErrorFunctor(const sfmData::Observation& obs, const std::shared_ptr<camera::IntrinsicBase>& intrinsics)
      : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics))
    {}

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_pose = parameters[2];
        const T* parameter_point = parameters[pointIndex];

        T transformedPoint[3];
        applyPose(parameter_pose, parameter_point, transformedPoint);

        if constexpr (withSubPose)
        {
            const T* parameter_subpose = parameters[3];
            applyPose(parameter_subpose, transformedPoint, transformedPoint);
        }

        const T* innerParameters[3];
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
    inline static ceres::CostFunction* createCostFunction(const std::shared_ptr<camera::IntrinsicBase>& intrinsic,
                                                          const sfmData::Observation& observation)
    {
        auto costFunction =
          new ceres::DynamicAutoDiffCostFunction<ProjectionErrorFunctor>(new ProjectionErrorFunctor(observation, intrinsic));

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(getDistortionSize(intrinsic));
        costFunction->AddParameterBlock(6);
        if constexpr (withSubPose)
        {
            costFunction->AddParameterBlock(6);
        }
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
            //withoutPose is used if reference view and current view are the same views
            const T* parameter_relativepoint = parameters[2];
            transformedPoint[0] = parameter_relativepoint[0];
            transformedPoint[1] = parameter_relativepoint[1];
            transformedPoint[2] = parameter_relativepoint[2];
        }
        else
        {
            const T* parameter_pose = parameters[2];
            const T* parameter_refpose = parameters[3];
            const T* parameter_relativepoint = parameters[4];

            /// (cam_T_world) * (ref_T_world)^-1 * ref_p
            /// cam_R_world * (ref_R_world^T * ref_p - ref_R_world^T * ref_t_world) + cam_t_world
            /// cam_R_world * (ref_R_world^T * ref_p + ref_c_world) + cam_t_world
            /// cam_R_world * (ref_R_world^T * ref_p + ref_c_world) - cam_R_world * cam_c_world
            /// cam_R_world * (ref_R_world^T * ref_p + ref_c_world - cam_c_world)

            //Retrieve point
            T relpoint[3];
            relpoint[0] = parameter_relativepoint[0];
            relpoint[1] = parameter_relativepoint[1];
            relpoint[2] = parameter_relativepoint[2];

            const T* refcam_R = parameter_refpose;
            const T* refcam_c = &parameter_refpose[3];
            
            // Apply transformation such that the point
            // Which was defined in the camera geometric frame
            // is now defined in the world frame
            T invrefcam_R[3];
            invrefcam_R[0] = -refcam_R[0];
            invrefcam_R[1] = -refcam_R[1];
            invrefcam_R[2] = -refcam_R[2];

            T absolutePoint[3];  
            ceres::AngleAxisRotatePoint(invrefcam_R, relpoint, absolutePoint);

            absolutePoint[0] += refcam_c[0];
            absolutePoint[1] += refcam_c[1];
            absolutePoint[2] += refcam_c[2];

            // Apply external parameters (Pose)
            applyPose(parameter_pose, absolutePoint, transformedPoint);
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
        const std::shared_ptr<camera::IntrinsicBase>& intrinsic,
        const sfmData::Observation& observation,
        bool withoutPose)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionRelativeErrorFunctor>(new ProjectionRelativeErrorFunctor(observation, intrinsic, withoutPose));

        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(getDistortionSize(intrinsic));

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
