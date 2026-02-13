// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
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
                                    const std::shared_ptr<camera::IntrinsicBase>& intrinsic_1,
                                    const std::shared_ptr<camera::IntrinsicBase>& intrinsic_2,
                                    bool hasRig1,
                                    bool hasRig2,
                                    bool sharedPose,
                                    bool sharedSubPose)        
    : _sharedIntrinsic(intrinsic_1==intrinsic_2),
    _sharedSubPose(sharedSubPose),
    _sharedPose(sharedPose),
    _hasRig1(hasRig1),
    _hasRig2(hasRig2),
    _intrinsicLiftFunctor(new CostIntrinsicsLift(obs1.getCoordinates(), intrinsic_1)),
    _intrinsicProjectFunctor(new CostIntrinsicsProject(obs2, intrinsic_2))
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        int lastParam = 0;

        const T * parameter_intrinsics_lift = parameters[lastParam++];
        const T * parameter_distortion_lift = parameters[lastParam++];
        const T * parameter_intrinsics_project = _sharedIntrinsic ? parameter_intrinsics_lift : parameters[lastParam++];
        const T * parameter_distortion_project = _sharedIntrinsic ? parameter_distortion_lift : parameters[lastParam++];
        const T * parameter_pose1 = parameters[lastParam++];
        const T * parameter_pose2 = (_sharedPose) ? parameter_pose1 : parameters[lastParam++];
        const T * parameter_rig1 = (_hasRig1) ? parameters[lastParam++] : nullptr;
        const T * parameter_rig2 = (_hasRig2) ? (_sharedSubPose ? parameter_rig1 : parameters[lastParam++]) : nullptr;
        

        Eigen::Matrix<T, 3, 3> oneRo;
        if (parameter_rig1)
        {
            Eigen::Matrix<T, 3, 3> oneRrig;
            Eigen::Matrix<T, 3, 3> rigRo;
            ceres::AngleAxisToRotationMatrix(parameter_pose1, rigRo.data());
            ceres::AngleAxisToRotationMatrix(parameter_rig1, oneRrig.data());
            oneRo = oneRrig * rigRo;
        }   
        else
        {
            ceres::AngleAxisToRotationMatrix(parameter_pose1, oneRo.data());
        }

        Eigen::Matrix<T, 3, 3> twoRo;
        if (parameter_rig2)
        {
            Eigen::Matrix<T, 3, 3> twoRrig;
            Eigen::Matrix<T, 3, 3> rigRo;
            ceres::AngleAxisToRotationMatrix(parameter_pose2, rigRo.data());
            ceres::AngleAxisToRotationMatrix(parameter_rig2, twoRrig.data());
            twoRo = twoRrig * rigRo;
        }   
        else
        {
            ceres::AngleAxisToRotationMatrix(parameter_pose2, twoRo.data());
        }

        Eigen::Matrix<T, 3, 3> twoRone = twoRo * oneRo.transpose();

        Eigen::Vector<T, 3> lifted;
        const T * liftParameters[2];
        liftParameters[0] = parameter_intrinsics_lift;
        liftParameters[1] = parameter_distortion_lift;
        _intrinsicLiftFunctor(liftParameters, lifted.data());
        Eigen::Vector<T, 3> transformed = twoRone * lifted;

        const T * projectParameters[3];
        projectParameters[0] = parameter_intrinsics_project;
        projectParameters[1] = parameter_distortion_project;
        projectParameters[2] = transformed.data();

        return _intrinsicProjectFunctor(projectParameters, residuals);
    }

    /**
     * @brief Create the appropriate cost functor according to the provided input camera intrinsic model
     * @param[in] intrinsicLift The intrinsic pointer of the origin camera (lifted)
     * @param[in] intrinsicProject The intrinsic pointer of the destination camera (projected)
     * @param[in] observation_first The corresponding observation in the first view
     * @param[in] observation_second The corresponding observation in the second view
     * @param[in] rigLift Is the first view part of a rig ?
     * @param[in] rigProject Is the second view part of a rig ?
     * @param[in] sharedPose both views are using the same pose
     * @param[in] sharedSubPose both views are using the same subpose of the rig
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(std::shared_ptr<camera::IntrinsicBase> intrinsicLift,
                                            std::shared_ptr<camera::IntrinsicBase> intrinsicProject,
                                            const sfmData::Observation& observation_first,
                                            const sfmData::Observation& observation_second,
                                            bool rigLift,
                                            bool rigProject,
                                            bool sharedPose,
                                            bool sharedSubPose
                                        )
    {       
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<Constraint2dErrorFunctor>(new Constraint2dErrorFunctor(observation_first, observation_second, intrinsicLift, intrinsicProject, rigLift, rigProject, sharedPose, sharedSubPose));

        int distortionLiftSize = 1;
        auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsicLift);
        if (isod)
        {
            auto distortion = isod->getDistortion();
            if (distortion)
            {
                distortionLiftSize = distortion->getParameters().size();
            }
        }

        costFunction->AddParameterBlock(intrinsicLift->getParameters().size()); 
        costFunction->AddParameterBlock(distortionLiftSize);

        // Add intrinsic parameters for the second camera if this
        // camera is different from the first
        if (intrinsicLift != intrinsicProject)
        {
            int distortionProjectSize = 1;
            auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsicProject);
            if (isod)
            {
                auto distortion = isod->getDistortion();
                if (distortion)
                {
                    distortionProjectSize = distortion->getParameters().size();
                }
            }

            costFunction->AddParameterBlock(intrinsicProject->getParameters().size()); 
            costFunction->AddParameterBlock(distortionProjectSize);
        }

        costFunction->AddParameterBlock(6);
        if (!sharedPose)
        {
            costFunction->AddParameterBlock(6);
        }

        if (rigLift)
        {
            costFunction->AddParameterBlock(6);
        }

        if (rigProject && !sharedSubPose)
        {
            costFunction->AddParameterBlock(6);
        }

        costFunction->SetNumResiduals(2);

        return costFunction;
    }

    bool _sharedIntrinsic;
    bool _sharedSubPose;
    bool _sharedPose;
    bool _hasRig1;
    bool _hasRig2;
    ceres::DynamicCostFunctionToFunctorTmp _intrinsicLiftFunctor;
    ceres::DynamicCostFunctionToFunctorTmp _intrinsicProjectFunctor;
};

}  // namespace sfm
}  // namespace aliceVision
