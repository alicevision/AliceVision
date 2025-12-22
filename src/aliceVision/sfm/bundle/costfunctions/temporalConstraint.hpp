// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsProject.hpp>
#include <aliceVision/sfm/bundle/costfunctions/intrinsicsLift.hpp>
#include <ceres/rotation.h>
#include "dynamic_cost_function_to_functor.h"

namespace aliceVision {
namespace sfm {

/**
 * @brief TemporalConstraintFunctor implements a temporal constraint for camera poses in bundle adjustment.
 *
 * This functor is designed to enforce smoothness between consecutive camera poses (positions and/or orientations)
 * of a video sequence.
 * It computes residuals based on the differences in position and orientation between consecutive views, weighted
 * by the provided position and orientation weights.
 *
 * The 'boundaryCase' parameter is used to handle boundary conditions. (0 is used for the general case)
 */
struct TemporalConstraintFunctor
{
    explicit TemporalConstraintFunctor(const double positionWeight, const double orientationWeight,
                const double c0pWeight, const double c1pWeight, const double c2pWeight,
                const double c0oWeight, const double c1oWeight, const double c2oWeight, const int boundaryCase)
    : _boundaryCase(boundaryCase)
    {
        _noPositionConstraint = positionWeight == 0.;

        _c0pWeight = c0pWeight * positionWeight;
        _c1pWeight = c1pWeight * positionWeight;
        _c2pWeight = c2pWeight * positionWeight;

        _c0oWeight = c0oWeight * orientationWeight;
        _c1oWeight = c1oWeight * orientationWeight;
        _c2oWeight = c2oWeight * orientationWeight;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {
        const T* para0 = parameters[0];
        const T* para1 = parameters[1];
        const T* para2 = parameters[2];
        const T* para3 = parameters[3];

        const auto viewPreProcess = [&](const T* angleAxis, const T* translation, T* viewCenter, T* quaternion, T* invQuaternion)
        {
            T invAngleAxis[3];
            // Inverse the rotation to compute the camera position
            invAngleAxis[0] = -angleAxis[0];
            invAngleAxis[1] = -angleAxis[1];
            invAngleAxis[2] = -angleAxis[2];
            ceres::AngleAxisRotatePoint(invAngleAxis, translation, viewCenter);
            ceres::AngleAxisToQuaternion(angleAxis, quaternion);
            ceres::AngleAxisToQuaternion(invAngleAxis, invQuaternion);
        };

        T viewCenter0[3], viewCenter1[3], viewCenter2[3], viewCenter3[3];
        T quaternion0[4], quaternion1[4], quaternion2[4], quaternion3[4];
        T invQuaternion0[4], invQuaternion1[4], invQuaternion2[4], invQuaternion3[4];

        viewPreProcess(&para0[0], &para0[3], viewCenter0, quaternion0, invQuaternion0);
        viewPreProcess(&para1[0], &para1[3], viewCenter1, quaternion1, invQuaternion1);
        viewPreProcess(&para2[0], &para2[3], viewCenter2, quaternion2, invQuaternion2);
        viewPreProcess(&para3[0], &para3[3], viewCenter3, quaternion3, invQuaternion3);

        const auto computeAngleDiff = [&](const T* quaternionA, const T* invQuaternionB, T* angleAxisDiff)
        {
            T quaternionDiff[4];
            ceres::QuaternionProduct(quaternionA, invQuaternionB, quaternionDiff);
            ceres::QuaternionToAngleAxis(quaternionDiff, angleAxisDiff);
        };

        T angleAxisDiff0[3], angleAxisDiff1[3], angleAxisDiff2[3];

        computeAngleDiff(quaternion1, invQuaternion0, angleAxisDiff0);
        computeAngleDiff(quaternion2, invQuaternion1, angleAxisDiff1);
        computeAngleDiff(quaternion3, invQuaternion2, angleAxisDiff2);

        if (_boundaryCase == 2)
        {
            residuals[0] = _c0oWeight * angleAxisDiff0[0];
            residuals[1] = _c0oWeight * angleAxisDiff0[1];
            residuals[2] = _c0oWeight * angleAxisDiff0[2];
        }
        else if (_boundaryCase == 1)
        {
            residuals[0] = _c0oWeight * angleAxisDiff1[0];
            residuals[1] = _c0oWeight * angleAxisDiff1[1];
            residuals[2] = _c0oWeight * angleAxisDiff1[2];

            residuals[3] = _c1oWeight * (angleAxisDiff1[0] - angleAxisDiff0[0]);
            residuals[4] = _c1oWeight * (angleAxisDiff1[1] - angleAxisDiff0[1]);
            residuals[5] = _c1oWeight * (angleAxisDiff1[2] - angleAxisDiff0[2]);
        }
        else
        {
            residuals[0] = _c0oWeight * angleAxisDiff2[0];
            residuals[1] = _c0oWeight * angleAxisDiff2[1];
            residuals[2] = _c0oWeight * angleAxisDiff2[2];

            residuals[3] = _c1oWeight * (angleAxisDiff2[0] - angleAxisDiff1[0]);
            residuals[4] = _c1oWeight * (angleAxisDiff2[1] - angleAxisDiff1[1]);
            residuals[5] = _c1oWeight * (angleAxisDiff2[2] - angleAxisDiff1[2]);

            residuals[6] = _c2oWeight * (2. * angleAxisDiff1[0] - angleAxisDiff0[0] - angleAxisDiff2[0]);
            residuals[7] = _c2oWeight * (2. * angleAxisDiff1[1] - angleAxisDiff0[1] - angleAxisDiff2[1]);
            residuals[8] = _c2oWeight * (2. * angleAxisDiff1[2] - angleAxisDiff0[2] - angleAxisDiff2[2]);
        }

        if (_noPositionConstraint)
            return true;

        if (_boundaryCase == 2)
        {
            residuals[3] = _c0pWeight * (viewCenter1[0] - viewCenter0[0]);
            residuals[4] = _c0pWeight * (viewCenter1[1] - viewCenter0[1]);
            residuals[5] = _c0pWeight * (viewCenter1[2] - viewCenter0[2]);
        }
        else if (_boundaryCase == 1)
        {
            residuals[6] = _c0pWeight * (viewCenter2[0] - viewCenter1[0]);
            residuals[7] = _c0pWeight * (viewCenter2[1] - viewCenter1[1]);
            residuals[8] = _c0pWeight * (viewCenter2[2] - viewCenter1[2]);

            T c1normalization = sqrt( (viewCenter2[0] - viewCenter1[0]) * (viewCenter2[0] - viewCenter1[0])
                                    + (viewCenter2[1] - viewCenter1[1]) * (viewCenter2[1] - viewCenter1[1])
                                    + (viewCenter2[2] - viewCenter1[2]) * (viewCenter2[2] - viewCenter1[2])
                                    + (viewCenter1[0] - viewCenter0[0]) * (viewCenter1[0] - viewCenter0[0])
                                    + (viewCenter1[1] - viewCenter0[1]) * (viewCenter1[1] - viewCenter0[1])
                                    + (viewCenter1[2] - viewCenter0[2]) * (viewCenter1[2] - viewCenter0[2]) );

            residuals[9]  = _c1pWeight * (2. * viewCenter1[0] - viewCenter0[0] - viewCenter2[0]) / c1normalization;
            residuals[10] = _c1pWeight * (2. * viewCenter1[1] - viewCenter0[1] - viewCenter2[1]) / c1normalization;
            residuals[11] = _c1pWeight * (2. * viewCenter1[2] - viewCenter0[2] - viewCenter2[2]) / c1normalization;
        }
        else
        {
            residuals[9]  = _c0pWeight * (viewCenter3[0] - viewCenter2[0]);
            residuals[10] = _c0pWeight * (viewCenter3[1] - viewCenter2[1]);
            residuals[11] = _c0pWeight * (viewCenter3[2] - viewCenter2[2]);

            T c1normalization = sqrt( (viewCenter3[0] - viewCenter2[0]) * (viewCenter3[0] - viewCenter2[0])
                                    + (viewCenter3[1] - viewCenter2[1]) * (viewCenter3[1] - viewCenter2[1])
                                    + (viewCenter3[2] - viewCenter2[2]) * (viewCenter3[2] - viewCenter2[2])
                                    + (viewCenter2[0] - viewCenter1[0]) * (viewCenter2[0] - viewCenter1[0])
                                    + (viewCenter2[1] - viewCenter1[1]) * (viewCenter2[1] - viewCenter1[1])
                                    + (viewCenter2[2] - viewCenter1[2]) * (viewCenter2[2] - viewCenter1[2]) );

            residuals[12] = _c1pWeight * (2. * viewCenter2[0] - viewCenter1[0] - viewCenter3[0]) / c1normalization;
            residuals[13] = _c1pWeight * (2. * viewCenter2[1] - viewCenter1[1] - viewCenter3[1]) / c1normalization;
            residuals[14] = _c1pWeight * (2. * viewCenter2[2] - viewCenter1[2] - viewCenter3[2]) / c1normalization;

            T c2normalization = sqrt( (viewCenter3[0] - viewCenter2[0]) * (viewCenter3[0] - viewCenter2[0])
                                    + (viewCenter3[1] - viewCenter2[1]) * (viewCenter3[1] - viewCenter2[1])
                                    + (viewCenter3[2] - viewCenter2[2]) * (viewCenter3[2] - viewCenter2[2])
                                    + (viewCenter2[0] - viewCenter1[0]) * (viewCenter2[0] - viewCenter1[0])
                                    + (viewCenter2[1] - viewCenter1[1]) * (viewCenter2[1] - viewCenter1[1])
                                    + (viewCenter2[2] - viewCenter1[2]) * (viewCenter2[2] - viewCenter1[2])
                                    + (viewCenter1[0] - viewCenter0[0]) * (viewCenter1[0] - viewCenter0[0])
                                    + (viewCenter1[1] - viewCenter0[1]) * (viewCenter1[1] - viewCenter0[1])
                                    + (viewCenter1[2] - viewCenter0[2]) * (viewCenter1[2] - viewCenter0[2]) );

            residuals[15] = _c2pWeight * (3. * viewCenter1[0] - 3. * viewCenter2[0] + viewCenter3[0] - viewCenter0[0]) / c2normalization;
            residuals[16] = _c2pWeight * (3. * viewCenter1[1] - 3. * viewCenter2[1] + viewCenter3[1] - viewCenter0[1]) / c2normalization;
            residuals[17] = _c2pWeight * (3. * viewCenter1[2] - 3. * viewCenter2[2] + viewCenter3[2] - viewCenter0[2]) / c2normalization;
        }

        return true;
    }

    /**
     * @brief Create the appropriate cost functor
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(const double positionWeight, const double orientationWeight,
                                        const double c0pWeight, const double c1pWeight, const double c2pWeight,
                                        const double c0oWeight, const double c1oWeight, const double c2oWeight, const int boundaryCase = 0)
    {
        auto costFunction = new ceres::DynamicAutoDiffCostFunction<TemporalConstraintFunctor>
                            (
                                new TemporalConstraintFunctor(
                                    positionWeight, orientationWeight, 
                                    c0pWeight, c1pWeight, c2pWeight, c0oWeight, c1oWeight, c2oWeight, 
                                    boundaryCase
                                )
                            );  

        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(6);
        costFunction->AddParameterBlock(6); 
        
        if (boundaryCase == 2)
        {
            if (positionWeight != 0.)
            {
                costFunction->SetNumResiduals(6);
            }
            else
            {
                costFunction->SetNumResiduals(3);
            }
        }
        else if (boundaryCase == 1)
        {
            if (positionWeight != 0.)
            {
                costFunction->SetNumResiduals(12);
            }
            else
            {
                costFunction->SetNumResiduals(6);
            }
        }
        else
        {
            if (positionWeight != 0.)
            {
                costFunction->SetNumResiduals(18);
            }
            else
            {
                costFunction->SetNumResiduals(9);
            }
        }

        return costFunction;
    }

    double _c0oWeight;
    double _c1oWeight;
    double _c2oWeight;
    double _c0pWeight;
    double _c1pWeight;
    double _c2pWeight;

    bool _noPositionConstraint;

    int _boundaryCase;
};

/**
 * @brief Trajectory length regularization cost functor for bundle adjustment.
 *
 * This functor imposes a regularization of the temporal constraint on a sequence of camera poses
 * by penalizing deviations of the total distance between the consecutive camera positions from
 * a target value (the specified trajectory length).
 *
 * @param regularizationWeight  Weight applied to the regularization term.
 * @param trajectoryLength     The expected (target) total distance between consecutive camera positions.
 * @param paraSize        The number of camera parameter blocks (i.e., number of consecutive poses).
 *
 * The cost is computed as difference between the sum of the distance between all consecutive pairs
 * of views and the target distance, scaled by the regularization weight.
 */
struct TrajectoryLengthRegularizationFunctor
{
    explicit TrajectoryLengthRegularizationFunctor(const double regularizationWeight, const double trajectoryLength, const int paraSize)
    : _regWeight(regularizationWeight),
    _trajectoryLength(trajectoryLength),
    _paraSize(paraSize)
    {
    }

    template<typename T>
    bool operator()(const T* const * para, T* residuals) const
    {
        residuals[0] = _regWeight * (sqrt( (para[1][3] - para[0][3]) * (para[1][3] - para[0][3])
                                         + (para[1][4] - para[0][4]) * (para[1][4] - para[0][4])
                                         + (para[1][5] - para[0][5]) * (para[1][5] - para[0][5]) ) - _trajectoryLength);

        for (int paraIdx = 2; paraIdx < _paraSize; paraIdx++)
        {
            residuals[0] += _regWeight * sqrt( (para[paraIdx][3] - para[paraIdx-1][3]) * (para[paraIdx][3] - para[paraIdx-1][3])
                                             + (para[paraIdx][4] - para[paraIdx-1][4]) * (para[paraIdx][4] - para[paraIdx-1][4])
                                             + (para[paraIdx][5] - para[paraIdx-1][5]) * (para[paraIdx][5] - para[paraIdx-1][5]) );
        }

        return true;
    }

    /**
     * @brief Create the appropriate cost functor
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(const double regularizationWeight,
                                                          const double trajectoryLength, const int paraSize)
    {
        auto costFunction =  new ceres::DynamicAutoDiffCostFunction<TrajectoryLengthRegularizationFunctor>
                    (new TrajectoryLengthRegularizationFunctor(regularizationWeight, trajectoryLength, paraSize));

        for (int paraIdx = 0; paraIdx < paraSize; paraIdx++)
        {
            costFunction->AddParameterBlock(6);
        }
        costFunction->SetNumResiduals(1);

        return costFunction;
    }

    int _paraSize;
    double _regWeight;
    double _trajectoryLength;
};

/**
 * @brief Functor for regularizing the relationship distance between landmarks and views.
 *
 * This functor is used to impose a regularization constraint that encourages
 * the mean position of a set of landmarks to remain close to a reference mean
 * position ('meanL2V') relative to the camera poses (views). This is useful in
 * bundle adjustment to prevent drift (potential scale down).
 *
 * The regularization is controlled by the following parameters:
 * - @param regularizationWeight: The weight of the regularization term.
 * - @param landmarks: A vector of pointers to the 3D coordinates of the landmarks.
 * - @param meanL2V: The reference mean distance (as a 3D vector) representing the expected
 *   mean relative position of the landmarks to the views.
 * - @param paraSize: The number of parameter blocks (typically the number of views).
 *
 * The cost is computed as the difference between the current mean position of the landmarks
 * relative to the camera poses and the reference mean ('meanL2V'), scaled by the regularization
 * weight.
 */
struct Landmarks2viewsRegularizationFunctor
{
    explicit Landmarks2viewsRegularizationFunctor(const double regularizationWeight, const std::vector<double*> landmarks, const std::vector<double> meanL2V, const int paraSize)
    : _regWeight(regularizationWeight),
    _landmarks(landmarks),
    _meanL2V(meanL2V),
    _paraSize(paraSize)
    {
    }

    template<typename T>
    bool operator()(const T* const * para, T* residuals) const
    {
        double landmarksMean0 = 0.;
        double landmarksMean1 = 0.;
        double landmarksMean2 = 0.;

        double landmarksCount = double(_landmarks.size());

        for (double* landmark : _landmarks)
        {
            landmarksMean0 += landmark[0];
            landmarksMean1 += landmark[1];
            landmarksMean2 += landmark[2];
        }

        T landmarksMean0T = static_cast<T> ( landmarksMean0 / landmarksCount );
        T landmarksMean1T = static_cast<T> ( landmarksMean1 / landmarksCount );
        T landmarksMean2T = static_cast<T> ( landmarksMean2 / landmarksCount );

        T posesMean0 = para[0][3];
        T posesMean1 = para[0][4];
        T posesMean2 = para[0][5];

        for (int paraIdx = 1; paraIdx < _paraSize; paraIdx++)
        {
            posesMean0 += para[paraIdx][3];
            posesMean1 += para[paraIdx][4];
            posesMean2 += para[paraIdx][5];
        }

        T posesCount = static_cast<T>(double(_paraSize));

        posesMean0 /= posesCount;
        posesMean1 /= posesCount;
        posesMean2 /= posesCount;

        residuals[0] = _regWeight * (posesMean0 - landmarksMean0T - static_cast<T>(_meanL2V[0]));
        residuals[1] = _regWeight * (posesMean1 - landmarksMean1T - static_cast<T>(_meanL2V[1]));
        residuals[2] = _regWeight * (posesMean2 - landmarksMean2T - static_cast<T>(_meanL2V[2]));

        return true;
    }

    /**
     * @brief Create the appropriate cost functor
     * @return cost functor
     */
    inline static ceres::CostFunction* createCostFunction(const double regularizationWeight,
                                        const std::vector<double*> landmarks, const std::vector<double> meanL2V, const int paraSize)
    {
        auto costFunction =  new ceres::DynamicAutoDiffCostFunction<Landmarks2viewsRegularizationFunctor>
                    (new Landmarks2viewsRegularizationFunctor(regularizationWeight, landmarks, meanL2V, paraSize));

        for (int paraIdx = 0; paraIdx < paraSize; paraIdx++)
        {
            costFunction->AddParameterBlock(6);
        }
        costFunction->SetNumResiduals(3);

        return costFunction;
    }

    int _paraSize;
    double _regWeight;
    std::vector<double*> _landmarks;
    std::vector<double> _meanL2V;
};


}  // namespace sfm
}  // namespace aliceVision
