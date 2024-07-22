// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "distortionEstimationPoint.hpp"

#include <aliceVision/system/Logger.hpp>
#include <ceres/ceres.h>
#include <cmath>

namespace aliceVision {
namespace calibration {

class CostPoint : public ceres::CostFunction
{
public:
    CostPoint(std::shared_ptr<camera::Undistortion> & undistortion, const Vec2& ptUndistorted, const Vec2 &ptDistorted, double sigma)
        : _ptUndistorted(ptUndistorted)
        , _ptDistorted(ptDistorted)
        , _undistortion(undistortion)
        , _sigma(sigma)
    {
        set_num_residuals(2);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getUndistortionParametersCount());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_center = parameters[0];
        const double* parameter_disto = parameters[1];

        const int undistortionSize = _undistortion->getUndistortionParametersCount();

        //Read parameters and update camera
        std::vector<double> undistortionParams(undistortionSize);

        for (int idParam = 0; idParam < undistortionSize; idParam++)
        {
            undistortionParams[idParam] = parameter_disto[idParam];
        }
        _undistortion->setParameters(undistortionParams);

        Vec2 offset;
        offset.x() = parameter_center[0];
        offset.y() = parameter_center[1];
        _undistortion->setOffset(offset);

        //Estimate measure
        const Vec2 ipt = _undistortion->undistort(_ptDistorted);

        const Vec2 hsize = (_undistortion->getSize() * 0.5);
        Vec2 npt = (_ptDistorted - hsize);
        npt.x() = npt.x() / hsize.x();
        npt.y() = npt.y() / hsize.y();

        const double w1 = std::max(std::abs(npt.x()), std::abs(npt.y()));
        const double w = w1 * w1 / _sigma;

        residuals[0] = w * (ipt.x() - _ptUndistorted.x());
        residuals[1] = w * (ipt.y() - _ptUndistorted.y());

        if(jacobians == nullptr)
        {
            return true;
        }


        if(jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[0]);

            J = w * _undistortion->getDerivativeUndistortWrtOffset(_ptDistorted);
        }

        if(jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[1], 2, undistortionSize);

            J = w * _undistortion->getDerivativeUndistortWrtParameters(_ptDistorted);
        }

        return true;
    }

private:
    std::shared_ptr<camera::Undistortion> _undistortion;
    Vec2 _ptUndistorted;
    Vec2 _ptDistorted;
    double _sigma;
};

bool estimate(std::shared_ptr<camera::Undistortion> undistortionToEstimate,
              Statistics& statistics,
              const std::vector<PointPair>& pointpairs,
              const bool lockCenter,
              const std::vector<bool>& lockDistortions)
{
    if (!undistortionToEstimate)
    {
        return false;
    }

    if (pointpairs.empty())
    {
        return false;
    }

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(2.0);

    std::vector<double> undistortionParameters = undistortionToEstimate->getParameters();
    Vec2 undistortionOffset = undistortionToEstimate->getOffset();

    const std::size_t countUndistortionParams = undistortionParameters.size();

    if (lockDistortions.size() != countUndistortionParams)
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion parameters (lockDistortions=" << lockDistortions.size() << ", countDistortionParams="
                                                                                          << countUndistortionParams << ").");
        return false;
    }

    double* ptrUndistortionParameters = &undistortionParameters[0];
    double* center = &undistortionOffset.x();

    // Add off center parameter
    problem.AddParameterBlock(center, 2);
    if (lockCenter)
    {
        problem.SetParameterBlockConstant(center);
    }

    // Add distortion parameter
    problem.AddParameterBlock(ptrUndistortionParameters, countUndistortionParams);

    // Check if all distortions are locked
    bool allLocked = true;
    for (bool lock : lockDistortions)
    {
        if (!lock)
        {
            allLocked = false;
        }
    }

    if (allLocked)
    {
        problem.SetParameterBlockConstant(ptrUndistortionParameters);
    }
    else
    {
        // At least one parameter is not locked

        std::vector<int> constantDistortions;
        for (int idParamDistortion = 0; idParamDistortion < lockDistortions.size(); ++idParamDistortion)
        {
            if (lockDistortions[idParamDistortion])
            {
                constantDistortions.push_back(idParamDistortion);
            }
        }

        if (!constantDistortions.empty())
        {
            ceres::SubsetManifold* subsetManifold = new ceres::SubsetManifold(countUndistortionParams, constantDistortions);
            problem.SetManifold(ptrUndistortionParameters, subsetManifold);
        }
    }

    for (auto& ppt : pointpairs)
    {
        ceres::CostFunction* costFunction = new CostPoint(undistortionToEstimate, ppt.undistortedPoint, ppt.distortedPoint, pow(2.0, ppt.scale));
        problem.AddResidualBlock(costFunction, lossFunction, center, ptrUndistortionParameters);
    }

    ceres::Solver::Options options;
    options.use_inner_iterations = true;
    options.max_num_iterations = 1000;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ALICEVISION_LOG_TRACE(summary.FullReport());

    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_ERROR("Lens calibration estimation failed.");
        return false;
    }

    undistortionToEstimate->setOffset(undistortionOffset);
    undistortionToEstimate->setParameters(undistortionParameters);
    std::vector<double> errors;
    
    for (auto& ppt : pointpairs)
    {
        const Vec2 ipt = undistortionToEstimate->undistort(ppt.distortedPoint);
        double divider = pow(2.0, ppt.scale);
        const double res = (ipt - ppt.undistortedPoint).norm() / divider;
        errors.push_back(res);
    }

    const double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
    const double sqSum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    const double stddev = std::sqrt(sqSum / errors.size() - mean * mean);
    std::sort(errors.begin(), errors.end());
    const double median = errors[errors.size() / 2];
    const double max = errors[errors.size() - 1];
    const double lastDecile = errors[errors.size() * 0.9];

    statistics.mean = mean;
    statistics.stddev = stddev;
    statistics.median = median;
    statistics.max = max;
    statistics.lastDecile = lastDecile;
    
    return true;
}

}  // namespace calibration
}  // namespace aliceVision
