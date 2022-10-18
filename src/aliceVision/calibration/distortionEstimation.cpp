// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "distortionEstimation.hpp"

#include <ceres/ceres.h>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/utils/CeresUtils.hpp>


namespace aliceVision {
namespace calibration {

class CostLine : public ceres::CostFunction
{
public:
    CostLine(std::shared_ptr<camera::Undistortion> & undistortion, const Vec2& pt)
        : _pt(pt)
        , _undistortion(undistortion)
    {
        set_num_residuals(1);

        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getParameters().size());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_angle_line = parameters[0];
        const double* parameter_dist_line = parameters[1];
        const double* parameter_offset = parameters[2];
        const double* parameter_disto = parameters[3];

        const double angle = parameter_angle_line[0];
        const double distanceToLine = parameter_dist_line[0];

        const double cangle = cos(angle);
        const double sangle = sin(angle);
    
        std::vector<double> cameraDistortionParams = _undistortion->getParameters();
        const int distortionSize = cameraDistortionParams.size();
        for (int idParam = 0; idParam < distortionSize; idParam++)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _undistortion->setParameters(cameraDistortionParams);

        Vec2 undistortionOffset;
        undistortionOffset.x() = parameter_offset[0];
        undistortionOffset.y() = parameter_offset[1];
        _undistortion->setOffset(undistortionOffset);

        const Vec2 ipt = _undistortion->undistort(_pt);
        const double w = 1.0; // w1 * w1;

        residuals[0] = w * (cangle * ipt.x() + sangle * ipt.y() - distanceToLine);

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[0]);

            J(0, 0) = w * (ipt.x() * -sangle + ipt.y() * cangle);
        }

        if(jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[1]);
            J(0, 0) = -w;
        }

        if(jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> J(jacobians[2]);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtOffset(_pt);
        }

        if(jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 1, distortionSize);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtParameters(_pt);
        }

        return true;
    }

private:
    std::shared_ptr<camera::Undistortion> _undistortion;
    Vec2 _pt;
};

bool estimate(std::shared_ptr<camera::Undistortion> & undistortionToEstimate, Statistics & statistics, std::vector<LineWithPoints> & lines, bool lockCenter, const std::vector<bool> & lockDistortions)
{
    if (!undistortionToEstimate)
    {
        return false; 
    }

    if (lines.empty())
    {
        return false;
    }

    
    

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;
    

    std::vector<double> undistortionParameters = undistortionToEstimate->getParameters();
    Vec2 undistortionOffset = undistortionToEstimate->getOffset();
    const size_t countUndistortionParams = undistortionParameters.size();

    if (lockDistortions.size() != countUndistortionParams)
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion parameters (lockDistortions=" << lockDistortions.size() << ", countDistortionParams=" << countUndistortionParams << ").");
        return false;
    }

    double* ptrUndistortionParameters = &undistortionParameters[0];
    double* center = &undistortionOffset.x();

    //Add off center parameter
    problem.AddParameterBlock(center, 2);
    if (lockCenter)
    {
        problem.SetParameterBlockConstant(center);
    }

    //Add distortion parameter
    problem.AddParameterBlock(ptrUndistortionParameters, countUndistortionParams);

    //Check if all distortions are locked 
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
        //At least one parameter is not locked

        std::vector<int> constantDistortions;
        for (int idParamDistortion = 0; idParamDistortion < lockDistortions.size(); idParamDistortion++)
        {
            if (lockDistortions[idParamDistortion])
            {
                constantDistortions.push_back(idParamDistortion);
            }
        }

        if (!constantDistortions.empty())
        {
            ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(countUndistortionParams, constantDistortions);
            problem.SetParameterization(ptrUndistortionParameters, subsetParameterization);
        }
    }
    
    
    
    for (auto & l : lines)
    {
        problem.AddParameterBlock(&l.angle, 1);
        problem.AddParameterBlock(&l.dist, 1);

        for (Vec2 pt : l.points)
        {
            ceres::CostFunction * costFunction = new CostLine(undistortionToEstimate, pt);
            problem.AddResidualBlock(costFunction, lossFunction, &l.angle, &l.dist, center, ptrUndistortionParameters);
        }
    }

  
    ceres::Solver::Options options;
    options.use_inner_iterations = true;
    options.max_num_iterations = 100; 
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

    for (auto & l : lines)
    {
        const double sangle = sin(l.angle);
        const double cangle = cos(l.angle);

        for(const Vec2& pt : l.points)
        {
            const Vec2 ipt = undistortionToEstimate->undistort(pt);
            const double res = (cangle * ipt.x() + sangle * ipt.y() - l.dist);
            errors.push_back(std::abs(res));
        }
    }

    const double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / double(errors.size());
    const double sqSum = std::inner_product(errors.begin(), errors.end(), errors.begin(), 0.0);
    const double stddev = std::sqrt(sqSum / errors.size() - mean * mean);
    std::nth_element(errors.begin(), errors.begin() + errors.size()/2, errors.end());
    const double median = errors[errors.size() / 2];

    statistics.mean = mean;
    statistics.stddev = stddev;
    statistics.median = median;

    return true;
}

}//namespace calibration
}//namespace aliceVision
