// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "distortionEstimation.hpp"

#include <ceres/ceres.h>
#include <aliceVision/system/Logger.hpp>


namespace aliceVision {
namespace calibration {

class CostLine : public ceres::CostFunction
{
public:
    CostLine(std::shared_ptr<camera::Pinhole> & camera, const Vec2& pt)
        : _pt(pt)
        , _camera(camera)
    {
        set_num_residuals(1);

        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(camera->getDistortionParams().size());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_angle_line = parameters[0];
        const double* parameter_dist_line = parameters[1];
        const double* parameter_scale = parameters[2];
        const double* parameter_center = parameters[3];
        const double* parameter_disto = parameters[4];

        const double angle = parameter_angle_line[0];
        const double distanceToLine = parameter_dist_line[0];

        const double cangle = cos(angle);
        const double sangle = sin(angle);

        const int distortionSize = _camera->getDistortionParams().size();

        //Read parameters and update camera
        _camera->setScale(parameter_scale[0], parameter_scale[1]);
        _camera->setOffset(parameter_center[0], parameter_center[1]);
        std::vector<double> cameraDistortionParams = _camera->getDistortionParams();

        for (int idParam = 0; idParam < distortionSize; idParam++)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _camera->setDistortionParams(cameraDistortionParams);


        //Estimate measure
        const Vec2 cpt = _camera->ima2cam(_pt);
        const Vec2 distorted = _camera->addDistortion(cpt);
        const Vec2 ipt = _camera->cam2ima(distorted);

        const double w1 = std::max(0.4, std::max(std::abs(distorted.x()), std::abs(distorted.y())));
        const double w = w1 * w1;

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

            J = w * Jline * (_camera->getDerivativeIma2CamWrtScale(distorted) + _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtPt(cpt) * _camera->getDerivativeIma2CamWrtScale(_pt));
        }

        if(jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> J(jacobians[3]);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = w * Jline * (_camera->getDerivativeCam2ImaWrtPrincipalPoint() + _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtPt(cpt) * _camera->getDerivativeIma2CamWrtPrincipalPoint());
        }

        if(jacobians[4] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[4], 1, distortionSize);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle;

            J = w * Jline * _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtDisto(cpt);
        }

        return true;
    }

private:
    std::shared_ptr<camera::Pinhole> _camera;
    Vec2 _pt;
};


class CostPoint : public ceres::CostFunction
{
public:
    CostPoint(std::shared_ptr<camera::Pinhole> & camera, const Vec2& ptUndistorted, const Vec2 &ptDistorted)
        : _ptUndistorted(ptUndistorted)
        , _ptDistorted(ptDistorted)
        , _camera(camera)
    {
        set_num_residuals(2);

        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(camera->getDistortionParams().size());
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_scale = parameters[0];
        const double* parameter_center = parameters[1];
        const double* parameter_disto = parameters[2];

        const int distortionSize = _camera->getDistortionParams().size();

        //Read parameters and update camera
        _camera->setScale(parameter_scale[0], parameter_scale[1]);
        _camera->setOffset(parameter_center[0], parameter_center[1]);
        std::vector<double> cameraDistortionParams = _camera->getDistortionParams();

        for (int idParam = 0; idParam < distortionSize; idParam++)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _camera->setDistortionParams(cameraDistortionParams);

        //Estimate measure
        const Vec2 cpt = _camera->ima2cam(_ptUndistorted);
        const Vec2 distorted = _camera->addDistortion(cpt);
        const Vec2 ipt = _camera->cam2ima(distorted);

        const double w1 = std::max(std::abs(distorted.x()), std::abs(distorted.y()));
        const double w = w1 * w1;

        residuals[0] = w * (ipt.x() - _ptDistorted.x());
        residuals[1] = w * (ipt.y() - _ptDistorted.y());

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[0]);
            
            J = w * (_camera->getDerivativeIma2CamWrtScale(distorted) + _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtPt(cpt) * _camera->getDerivativeIma2CamWrtScale(_ptUndistorted));
        }

        if(jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[1]);


            J = w * (_camera->getDerivativeCam2ImaWrtPrincipalPoint() + _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtPt(cpt) * _camera->getDerivativeIma2CamWrtPrincipalPoint());
        }

        if(jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, distortionSize);

            J = w * _camera->getDerivativeCam2ImaWrtPoint() * _camera->getDerivativeAddDistoWrtDisto(cpt);
        }

        return true;
    }

private:
    std::shared_ptr<camera::Pinhole> _camera;
    Vec2 _ptUndistorted;
    Vec2 _ptDistorted;
};


bool estimate(std::shared_ptr<camera::Pinhole> & cameraToEstimate, Statistics & statistics, std::vector<LineWithPoints> & lines, bool lockScale, bool lockCenter, const std::vector<bool> & lockDistortions)
{
    if (!cameraToEstimate)
    {
        return false; 
    }

    if (lines.empty())
    {
        return false;
    }

    const size_t countDistortionParams = cameraToEstimate->getDistortionParams().size();
    if (lockDistortions.size() != countDistortionParams) 
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion parameters (lockDistortions=" << lockDistortions.size() << ", countDistortionParams=" << countDistortionParams << ").");
        return false;
    }

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    std::vector<double> params = cameraToEstimate->getParams();
    double * scale = &params[0];
    double * center = &params[2];
    double * distortionParameters = &params[4];

    problem.AddParameterBlock(scale, 2);
    if (lockScale)
    {
        problem.SetParameterBlockConstant(scale);
    }
    else
    {
        ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(2, {1});   
        problem.SetParameterization(scale, subsetParameterization);
    }

    //Add off center parameter
    problem.AddParameterBlock(center, 2);
    if (lockCenter)
    {
        problem.SetParameterBlockConstant(center);
    }

    //Add distortion parameter
    problem.AddParameterBlock(distortionParameters, countDistortionParams);

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
        problem.SetParameterBlockConstant(distortionParameters);
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
            ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(countDistortionParams, constantDistortions);   
            problem.SetParameterization(distortionParameters, subsetParameterization);
        }
    }
    
    
    for (auto & l : lines)
    {
        problem.AddParameterBlock(&l.angle, 1);
        problem.AddParameterBlock(&l.dist, 1);

        for (Vec2 pt : l.points)
        {
            ceres::CostFunction * costFunction = new CostLine(cameraToEstimate, pt);   
            problem.AddResidualBlock(costFunction, lossFunction, &l.angle, &l.dist, scale, center, distortionParameters);
        }
    }

    ceres::Solver::Options options;
    options.use_inner_iterations = true;
    options.max_num_iterations = 10000; 
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;  
    ceres::Solve(options, &problem, &summary);

    ALICEVISION_LOG_TRACE(summary.FullReport());

    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_ERROR("Lens calibration estimation failed.");
        return false;
    }

    cameraToEstimate->updateFromParams(params);

    std::vector<double> errors;

    for (auto & l : lines)
    {
        const double sangle = sin(l.angle);
        const double cangle = cos(l.angle);

        for(const Vec2& pt : l.points)
        {
            const Vec2 cpt = cameraToEstimate->ima2cam(pt);
            const Vec2 distorted = cameraToEstimate->addDistortion(cpt);
            const Vec2 ipt = cameraToEstimate->cam2ima(distorted);

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

bool estimate(std::shared_ptr<camera::Pinhole> & cameraToEstimate, Statistics & statistics, std::vector<PointPair> & points, bool lockScale, bool lockCenter, const std::vector<bool> & lockDistortions)
{
    if (!cameraToEstimate)
    {
        return false; 
    }

    if (points.empty())
    {
        return false;
    }

    size_t countDistortionParams = cameraToEstimate->getDistortionParams().size();
    if (lockDistortions.size() != countDistortionParams) 
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion parameters (lockDistortions=" << lockDistortions.size() << ", countDistortionParams=" << countDistortionParams << ").");
        return false;
    }

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    std::vector<double> params = cameraToEstimate->getParams();
    double * scale = &params[0];
    double * center = &params[2];
    double * distortionParameters = &params[4];

    problem.AddParameterBlock(scale, 2);
    if (lockScale)
    {
        problem.SetParameterBlockConstant(scale);
    }
    else
    {
        ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(2, {1});   
        problem.SetParameterization(scale, subsetParameterization);
    }
    

    //Add off center parameter
    problem.AddParameterBlock(center, 2);
    if (lockCenter)
    {
        problem.SetParameterBlockConstant(center);
    }

    //Add distortion parameter
    problem.AddParameterBlock(distortionParameters, countDistortionParams);

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
        problem.SetParameterBlockConstant(distortionParameters);
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
            ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(countDistortionParams, constantDistortions);   
            problem.SetParameterization(distortionParameters, subsetParameterization);
        }
    }
    
    for (const PointPair & pt : points)
    {
        ceres::CostFunction * costFunction = new CostPoint(cameraToEstimate, pt.undistortedPoint, pt.distortedPoint);   
        problem.AddResidualBlock(costFunction, lossFunction, scale, center, distortionParameters);
    }

    // google::SetCommandLineOption("GLOG_minloglevel", "3");
    ceres::Solver::Options options;
    options.use_inner_iterations = true;
    options.max_num_iterations = 10000; 
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;  
    ceres::Solve(options, &problem, &summary);

    ALICEVISION_LOG_TRACE(summary.FullReport());

    if (!summary.IsSolutionUsable())
    {
        ALICEVISION_LOG_ERROR("Lens calibration estimation failed.");
        return false;
    }

    cameraToEstimate->updateFromParams(params);

    std::vector<double> errors;

    for (PointPair pp : points)
    {
        const Vec2 cpt = cameraToEstimate->ima2cam(pp.undistortedPoint);
        const Vec2 distorted = cameraToEstimate->addDistortion(cpt);
        const Vec2 ipt = cameraToEstimate->cam2ima(distorted);

        const double res = (ipt - pp.distortedPoint).norm();

        errors.push_back(res);
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
