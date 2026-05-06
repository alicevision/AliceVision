// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <aliceVision/sfm/bundle/costfunctions/meshIntersect.hpp>
#include <aliceVision/sfmData/PointFetcher.hpp>

#include <ceres/rotation.h>


#include "dynamic_cost_function_to_functor.h"

#include <memory>


namespace aliceVision {
namespace sfm {

struct ProjectionMeshErrorFunctor
{
    explicit ProjectionMeshErrorFunctor(const sfmData::Observation& obs
    , const std::shared_ptr<camera::IntrinsicBase>& intrinsics
    , sfmData::PointFetcher::sptr fetcher
    , bool samePose)
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics))
    , _meshIntersectFunctor(new CostMeshIntersector(fetcher))
    , _samePose(samePose)
    {
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_currentPose = parameters[2];
        const T* parameter_referencePose = _samePose ? parameter_currentPose : parameters[3];
        const T* parameter_point = _samePose ? parameters[3] : parameters[4];


        const T* refcam_r_world = parameter_referencePose;
        const T* world_t_refcam = &parameter_referencePose[3];
        const T* curcam_r_world = parameter_currentPose;
        const T* world_t_curcam = &parameter_currentPose[3];

        T world_r_refcam[3];
        world_r_refcam[0] = -refcam_r_world[0];
        world_r_refcam[1] = -refcam_r_world[1];
        world_r_refcam[2] = -refcam_r_world[2];

        T norm = sqrt(parameter_point[0] * parameter_point[0] + parameter_point[1] * parameter_point[1] + 1.0);

        // Get normalized direction
        T refcam_point[3];
        refcam_point[0] = parameter_point[0] / norm;
        refcam_point[1] = parameter_point[1] / norm;
        refcam_point[2] = 1.0 / norm;

        // Compute direction = R^t reference_point
        T direction[3];
        ceres::AngleAxisRotatePoint(world_r_refcam, refcam_point, direction);


        T worldPoint[3];
        const T * intersectParameters[2];
        intersectParameters[0] = world_t_refcam;
        intersectParameters[1] = direction;
        if (!_meshIntersectFunctor(intersectParameters, worldPoint))
        {
            return false;
        }

        // Compute point in camera coordinates
        T transformedPoint[3];
        worldPoint[0] -= world_t_curcam[0];
        worldPoint[1] -= world_t_curcam[1];
        worldPoint[2] -= world_t_curcam[2];
        ceres::AngleAxisRotatePoint(curcam_r_world, worldPoint, transformedPoint);

        // Project
        const T * innerParameters[3];
        innerParameters[0] = parameter_intrinsics;
        innerParameters[1] = parameter_distortion;
        innerParameters[2] = transformedPoint;
        return _intrinsicFunctor(innerParameters, residuals);
    }

    /**
     * @brief Create the appropriate cost functor according to the provided input camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] observation The corresponding observation
     * @param[in] fetcher the mesh point fetcher used for 3D point retrieval
     * @param[in] samePose When reference view and current view are the same, one pose is added
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
        const std::shared_ptr<camera::IntrinsicBase> intrinsic,
        const sfmData::Observation& observation,
        sfmData::PointFetcher::sptr fetcher,
        bool samePose)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<ProjectionMeshErrorFunctor>(new ProjectionMeshErrorFunctor(observation, intrinsic, fetcher, samePose));

        // Estimate distortion size from intrinsics
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

        // Intrinsic paramters
        costFunction->AddParameterBlock(intrinsic->getParameters().size());
        costFunction->AddParameterBlock(distortionSize);

        // Add parameters for poses
        if (!samePose)
        {
            costFunction->AddParameterBlock(6);
        }
        costFunction->AddParameterBlock(6);

        // Only consider the first two coordinates
        costFunction->AddParameterBlock(2);

        // Residual is pixel error
        costFunction->SetNumResiduals(2);

        return costFunction;
    }

    ceres::DynamicCostFunctionToFunctorTmp _intrinsicFunctor;
    ceres::DynamicCostFunctionToFunctorTmp _meshIntersectFunctor;
    bool _samePose;
};


}  // namespace sfm
}  // namespace aliceVision
