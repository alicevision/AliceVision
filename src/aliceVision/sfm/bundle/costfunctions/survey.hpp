// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


namespace aliceVision {
namespace sfm {


struct SurveyErrorFunctor
{
    explicit SurveyErrorFunctor(const Vec3 & point, 
                                const sfmData::Observation& obs, 
                                const Vec2 & residual,
                                const std::shared_ptr<camera::IntrinsicBase>& intrinsics,
                                double weight)        
    : _intrinsicFunctor(new CostIntrinsicsProject(obs, intrinsics)), _point(point), _originalResidual(residual), _weight(weight)
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_intrinsics = parameters[0];
        const T* parameter_distortion = parameters[1];
        const T* parameter_pose = parameters[2];
        const T* parameter_center = parameters[3];
        
        //Create input point
        T parameter_point[3];
        parameter_point[0] = T(_point.x()) - parameter_center[0];
        parameter_point[1] = T(_point.y()) - parameter_center[1];
        parameter_point[2] = T(_point.z()) - parameter_center[2];

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

        const T * innerParameters[3];
        innerParameters[0] = parameter_intrinsics;
        innerParameters[1] = parameter_distortion;
        innerParameters[2] = transformedPoint;

        if (!_intrinsicFunctor(innerParameters, residuals))
        {
            return false;
        }

        T epsilon(1e-10);
        T rx(std::abs(_originalResidual[0]) * 1.1);
        T ry(std::abs(_originalResidual[1]) * 1.1);

        residuals[0] = ceres::sqrt(_weight) * ceres::fmax(T(0), ceres::sqrt(residuals[0]*residuals[0] + epsilon) - rx);
        residuals[1] = ceres::sqrt(_weight) * ceres::fmax(T(0), ceres::sqrt(residuals[1]*residuals[1] + epsilon) - ry);

        return true;
    }

    /**
     * @brief Create the appropriate cost functor according to the provided input camera intrinsic model
     * @param[in] intrinsicPtr The intrinsic pointer
     * @param[in] point the reference point to compare to
     * @param[in] residual 2d vector with the original residual for the observation
     * @param[in] observation The corresponding observation
     * @param[in] weight the scale for the loss
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(
                                            const std::shared_ptr<camera::IntrinsicBase> intrinsic, 
                                            const Vec3 & point,
                                            const Vec2 & residual,
                                            const sfmData::Observation& observation,
                                            double weight)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<SurveyErrorFunctor>(new SurveyErrorFunctor(point, observation, residual, intrinsic, weight));

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
    Vec3 _point;
    Vec2 _originalResidual;
    double _weight;
};

}  // namespace sfm
}  // namespace aliceVision
