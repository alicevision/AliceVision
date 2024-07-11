// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "distortionEstimation.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/bundle/manifolds/so3.hpp>

#include <ceres/ceres.h>

#include <cmath>

namespace aliceVision {
namespace calibration {

class CostLine : public ceres::CostFunction
{
  public:
    CostLine(const std::shared_ptr<camera::Undistortion>& undistortion, const Vec2& pt, double sigma)
      : _pt(pt),
        _undistortion(undistortion),
        _sigma(sigma)
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

        const double cangle = std::cos(angle);
        const double sangle = std::sin(angle);

        std::vector<double> cameraDistortionParams = _undistortion->getParameters();
        const std::size_t distortionSize = cameraDistortionParams.size();
        for (std::size_t idParam = 0; idParam < distortionSize; ++idParam)
        {
            cameraDistortionParams[idParam] = parameter_disto[idParam];
        }
        _undistortion->setParameters(cameraDistortionParams);

        Vec2 undistortionOffset;
        undistortionOffset.x() = parameter_offset[0];
        undistortionOffset.y() = parameter_offset[1];
        _undistortion->setOffset(undistortionOffset);

        const Vec2 ipt = _undistortion->undistort(_pt);
        const double pa = _undistortion->getPixelAspectRatio();
        const double ny = ipt.y() / pa;
        const double w = 1.0 / _sigma;
        
        residuals[0] = w * (cangle * ipt.x() + sangle * ny - distanceToLine);

        if (jacobians == nullptr)
        {
            return true;
        }

        if (jacobians[0] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[0]);
            J(0, 0) = w * (ipt.x() * -sangle + ny * cangle);
        }

        if (jacobians[1] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1, Eigen::RowMajor>> J(jacobians[1]);
            J(0, 0) = -w;
        }

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2, Eigen::RowMajor>> J(jacobians[2]);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle / pa;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtOffset(_pt);
        }

        if (jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 1, distortionSize);

            Eigen::Matrix<double, 1, 2> Jline;
            Jline(0, 0) = cangle;
            Jline(0, 1) = sangle / pa;

            J = w * Jline * _undistortion->getDerivativeUndistortWrtParameters(_pt);
        }

        return true;
    }

  private:
    std::shared_ptr<camera::Undistortion> _undistortion;
    Vec2 _pt;
    double _sigma;
};

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

class CostPointGeometry : public ceres::CostFunction
{
public:
    CostPointGeometry(std::shared_ptr<camera::Undistortion> & undistortion, const Vec2& ptUndistorted, const Vec2 &ptDistorted, double sigma)
        : _ptUndistorted(ptUndistorted)
        , _ptDistorted(ptDistorted)
        , _undistortion(undistortion)
        , _sigma(sigma)
    {
        set_num_residuals(2);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getUndistortionParametersCount());
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(1);
        mutable_parameter_block_sizes()->push_back(9);
        mutable_parameter_block_sizes()->push_back(3);
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        const double* parameter_center = parameters[0];
        const double* parameter_disto = parameters[1];
        const double* parameter_scale = parameters[2];
        const double* parameter_offsetx = parameters[3];
        const double* parameter_offsety = parameters[4];
        const double* parameter_rotation = parameters[5];
        const double* parameter_translation = parameters[6];

        const double scale = *parameter_scale;
        const double offsetx = *parameter_offsetx;
        const double offsety = *parameter_offsety;

        const Eigen::Map<const SO3::Matrix> R(parameter_rotation);
        const Eigen::Map<const Eigen::Vector3d> t(parameter_translation);

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

        const double pa = _undistortion->getPixelAspectRatio();

        const Vec2 upt = _undistortion->undistort(_ptDistorted);

        //Estimate measure
        const Vec2 ipt = _ptUndistorted;
                
        Vec3 ipt3;
        ipt3.x() = ipt.x();
        ipt3.y() = ipt.y();
        ipt3.z() = 0.0;

        const Vec3 tpt = R * ipt3 + t;
        
        Vec2 projected = tpt.head(2) / tpt(2);
        Vec2 scaled;

        scaled.x() = scale * projected.x() + offsetx;
        scaled.y() = pa * scale * projected.y() + offsety;
                
        const double w = (1 + ipt.norm()) / _sigma;

        residuals[0] = w * (upt.x() - scaled.x());
        residuals[1] = w * (upt.y() - scaled.y());

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

        if (jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, 1);
            
            Vec2 d_scaled_d_scale;
            d_scaled_d_scale(0) = projected.x();
            d_scaled_d_scale(1) = pa * projected.y();
            
            J = -w * d_scaled_d_scale;
        }

        if (jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 2, 1);
            
            Vec2 d_scaled_d_offsetx;
            d_scaled_d_offsetx(0) = 1.0;
            d_scaled_d_offsetx(1) = 0.0;
            
            J = -w * d_scaled_d_offsetx;
        }

        if (jacobians[4] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[4], 2, 1);
            
            Vec2 d_scaled_d_offsety;
            d_scaled_d_offsety(0) = 0.0;
            d_scaled_d_offsety(1) = 1.0;
            
            J = -w * d_scaled_d_offsety;
        }

        if (jacobians[5] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[5]);
            
            Eigen::Matrix<double, 2, 3> d_projected_d_tpt;
            d_projected_d_tpt(0, 0) = 1.0 / tpt.z();
            d_projected_d_tpt(0, 1) = 0.0;
            d_projected_d_tpt(0, 2) = - tpt.x() / (tpt.z() * tpt.z());
            d_projected_d_tpt(1, 0) = 0.0;
            d_projected_d_tpt(1, 1) = 1.0 / tpt.z();
            d_projected_d_tpt(1, 2) = - tpt.y() / (tpt.z() * tpt.z());

            Eigen::Matrix<double, 3, 2> d_ipt3_d_ipt;
            d_ipt3_d_ipt(0, 0) = 1;
            d_ipt3_d_ipt(0, 1) = 0;
            d_ipt3_d_ipt(1, 0) = 0;
            d_ipt3_d_ipt(1, 1) = 1;
            d_ipt3_d_ipt(2, 0) = 0;
            d_ipt3_d_ipt(2, 1) = 0;

            Eigen::Matrix<double, 2, 2> d_scaled_d_projected;
            d_scaled_d_projected(0, 0) = scale;
            d_scaled_d_projected(0, 1) = 0;
            d_scaled_d_projected(1, 0) = 0;
            d_scaled_d_projected(1, 1) = pa * scale;

            J = - w * d_scaled_d_projected * d_projected_d_tpt * getJacobian_AB_wrt_A<3, 3, 1>(R, ipt3) * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), R);
        }

        if (jacobians[6] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[6]);
            
            Eigen::Matrix<double, 2, 3> d_projected_d_tpt;
            d_projected_d_tpt(0, 0) = 1.0 / tpt.z();
            d_projected_d_tpt(0, 1) = 0.0;
            d_projected_d_tpt(0, 2) = - tpt.x() / (tpt.z() * tpt.z());
            d_projected_d_tpt(1, 0) = 0.0;
            d_projected_d_tpt(1, 1) = 1.0 / tpt.z();
            d_projected_d_tpt(1, 2) = - tpt.y() / (tpt.z() * tpt.z());

            Eigen::Matrix<double, 3, 2> d_ipt3_d_ipt;
            d_ipt3_d_ipt(0, 0) = 1;
            d_ipt3_d_ipt(0, 1) = 0;
            d_ipt3_d_ipt(1, 0) = 0;
            d_ipt3_d_ipt(1, 1) = 1;
            d_ipt3_d_ipt(2, 0) = 0;
            d_ipt3_d_ipt(2, 1) = 0;

            Eigen::Matrix<double, 2, 2> d_scaled_d_projected;
            d_scaled_d_projected(0, 0) = scale;
            d_scaled_d_projected(0, 1) = 0;
            d_scaled_d_projected(1, 0) = 0;
            d_scaled_d_projected(1, 1) = pa * scale;

            J = - w * d_scaled_d_projected * d_projected_d_tpt;
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
              std::vector<LineWithPoints>& lines,
              const bool lockCenter,
              const bool lockAngles,
              const std::vector<bool>& lockDistortions)
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


    for (auto& l : lines)
    {
        problem.AddParameterBlock(&l.angle, 1);
        problem.AddParameterBlock(&l.dist, 1);

        if (lockAngles)
        {
            problem.SetParameterBlockConstant(&l.angle);
        }

        for (const auto & pt : l.points)
        {
            ceres::CostFunction* costFunction = new CostLine(undistortionToEstimate, pt.center, pow(2.0, pt.scale));
            problem.AddResidualBlock(costFunction, lossFunction, &l.angle, &l.dist, center, ptrUndistortionParameters);
        }
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
    for (auto& l : lines)
    {
        const double sangle = std::sin(l.angle);
        const double cangle = std::cos(l.angle);

        for (const auto & pt : l.points)
        {
            const Vec2 ipt = undistortionToEstimate->undistort(pt.center);
            const double pa = undistortionToEstimate->getPixelAspectRatio();
            const double ny = ipt.y() / pa;
            double divider = pow(2.0, pt.scale);
            const double res = (cangle * ipt.x() + sangle * ny - l.dist) / divider;
            errors.push_back(std::abs(res));
        }
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

bool estimate(std::shared_ptr<camera::Undistortion> undistortionToEstimate,
              Statistics& statistics,
              const std::vector<PointPair>& pointpairs,
              const bool lockCenter,
              const std::vector<bool>& lockDistortions,
              Eigen::Matrix3d & R, 
              Eigen::Vector3d & t,
              double & scale)
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
    ceres::LossFunction* lossFunction = nullptr;

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

    double offsetx = undistortionToEstimate->getCenter().x();
    double offsety = undistortionToEstimate->getCenter().y();
    
    problem.AddParameterBlock(&scale, 1);
    problem.AddParameterBlock(&offsetx, 1);
    problem.AddParameterBlock(&offsety, 1);
    
    problem.SetParameterLowerBound(&offsetx, 0, offsetx - 100.0);
    problem.SetParameterUpperBound(&offsetx, 0, offsetx + 100.0);
    problem.SetParameterLowerBound(&offsety, 0, offsety - 100.0);
    problem.SetParameterUpperBound(&offsety, 0, offsety + 100.0);

    SO3::Matrix Rest = R;
    Eigen::Vector3d Test = t;

    problem.AddParameterBlock(Rest.data(), 9);
    problem.AddParameterBlock(Test.data(), 3);

    problem.SetManifold(Rest.data(), new sfm::SO3Manifold);

    for (auto& ppt : pointpairs)
    {
        ceres::CostFunction* costFunction = new CostPointGeometry(undistortionToEstimate, ppt.undistortedPoint, ppt.distortedPoint, pow(2.0, ppt.scale));
        problem.AddResidualBlock(costFunction, lossFunction, center, ptrUndistortionParameters, &scale, &offsetx, &offsety, Rest.data(), Test.data());
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
    const double pa = undistortionToEstimate->getPixelAspectRatio();

    for (auto& ppt : pointpairs)
    {
        const Vec2 ipt = ppt.undistortedPoint;
        
        Vec3 ipt3;
        ipt3.x() = ipt.x();
        ipt3.y() = ipt.y();
        ipt3.z() = 0.0;

        const Vec3 tpt = Rest * ipt3 + Test;
        
        Vec2 projected = tpt.head(2) / tpt(2);
        Vec2 scaled;
        scaled.x() = scale * projected.x() + offsetx;
        scaled.y() = pa * scale * projected.y() + offsety;

        double divider = pow(2.0, ppt.scale);

        const double res = (undistortionToEstimate->undistort(ppt.distortedPoint) - scaled).norm() / divider;
        

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

    R = Rest;
    t = Test;
    
    return true;
}


}  // namespace calibration
}  // namespace aliceVision
