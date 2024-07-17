// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "distortionEstimationGeometry.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/bundle/manifolds/so3.hpp>
#include <ceres/ceres.h>
#include <cmath>

namespace aliceVision {
namespace calibration {

class CostPointGeometry : public ceres::CostFunction
{
public:
    CostPointGeometry(std::shared_ptr<camera::Undistortion> & undistortion, const std::vector<bool> & sharedParams, const Vec2& ptUndistorted, const Vec2 &ptDistorted, double sigma)
        : _ptUndistorted(ptUndistorted)
        , _ptDistorted(ptDistorted)
        , _undistortion(undistortion)
        , _sharedParams(sharedParams)
        , _sigma(sigma)
    {
        set_num_residuals(2);
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getUndistortionParametersCount());
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
        const double* parameter_distoShared = parameters[1];
        const double* parameter_disto = parameters[2];
        const double* parameter_scale = parameters[3];
        const double* parameter_offsetx = parameters[4];
        const double* parameter_offsety = parameters[5];
        const double* parameter_rotation = parameters[6];
        const double* parameter_translation = parameters[7];

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
            if (_sharedParams[idParam])
            {
                undistortionParams[idParam] = parameter_distoShared[idParam];
            }
            else 
            {
                undistortionParams[idParam] = parameter_disto[idParam];
            }
        }
        _undistortion->setParameters(undistortionParams);

        Vec2 offset;
        offset.x() = parameter_center[0];
        offset.y() = parameter_center[1];
        _undistortion->setOffset(offset);

        const double pa = (_undistortion->isDesqueezed())?1.0:_undistortion->getPixelAspectRatio();

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

        if(jacobians[2] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, undistortionSize);

            J = w * _undistortion->getDerivativeUndistortWrtParameters(_ptDistorted);
        }

        if (jacobians[3] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[3], 2, 1);
            
            Vec2 d_scaled_d_scale;
            d_scaled_d_scale(0) = projected.x();
            d_scaled_d_scale(1) = pa * projected.y();
            
            J = -w * d_scaled_d_scale;
        }

        if (jacobians[4] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[4], 2, 1);
            
            Vec2 d_scaled_d_offsetx;
            d_scaled_d_offsetx(0) = 1.0;
            d_scaled_d_offsetx(1) = 0.0;
            
            J = -w * d_scaled_d_offsetx;
        }

        if (jacobians[5] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[5], 2, 1);
            
            Vec2 d_scaled_d_offsety;
            d_scaled_d_offsety(0) = 0.0;
            d_scaled_d_offsety(1) = 1.0;
            
            J = -w * d_scaled_d_offsety;
        }

        if (jacobians[6] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[6]);
            
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

        if (jacobians[7] != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[7]);
            
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
    std::vector<bool> _sharedParams;
    double _sigma;
};

void DistortionEstimationGeometry::addView(std::shared_ptr<camera::Undistortion> undistortion, const std::vector<PointPair>& pointpairs)
{
    // Compute statistics on distorted points
    double minx = std::numeric_limits<double>::max();
    double maxx = 0.0;
    for (const auto & ppt : pointpairs)
    {
        const auto & pt = ppt.distortedPoint;

        minx = std::min(pt.x(), minx);
        maxx = std::max(pt.x(), maxx);
    }

    DistortionEstimationView dev;
    dev.pointpairs = pointpairs;

    //Board has 1m size so the scale is the "size" of the 
    dev.scale = maxx - minx;
    dev.offsetx = undistortion->getCenter().x();
    dev.offsety = undistortion->getCenter().y();

    //Keep a pointer to the undistortion object
    dev.undistortion = undistortion;
    dev.distortionOffset = undistortion->getOffset();
    dev.parameters = undistortion->getParameters();

    // Initialize pose
    dev.R.setIdentity();
    dev.t(0) = 0;
    dev.t(1) = 0;
    dev.t(2) = 1;


    _views.push_back(dev);
}

bool DistortionEstimationGeometry::compute(Statistics & statistics, const bool lockCenter, const std::vector<bool>& lockDistortions)
{
    //Create ceres objects
    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    if (_views.size() == 0)
    {
        return false;
    }

    if (lockDistortions.size() != _views[0].parameters.size())
    {
        ALICEVISION_LOG_ERROR("Invalid number of distortion locks");
        return false;
    }

    std::vector<bool> lockDistortionsNonShared = lockDistortions;
    

    std::vector<double> parametersShared = _views[0].parameters;
    double* ptrUndistortionSharedParameters = &parametersShared[0];
    const std::size_t countUndistortionParams = parametersShared.size();
    
    problem.AddParameterBlock(ptrUndistortionSharedParameters, countUndistortionParams);    

    std::vector<bool> lockShared(countUndistortionParams);
    for (int id = 0; id < countUndistortionParams; id++)
    {
        if (!_sharedParams[id])
        {
            lockShared[id] = true;
            continue;
        }

        lockShared[id] = lockDistortions[id];

        //If this value is shared, lock it for non shared vector
        lockDistortionsNonShared[id] = false;
    }

    // Check if all distortions are locked
    bool allSharedLocked = true;
    for (bool lock : lockShared)
    {
        if (!lock)
        {
            allSharedLocked = false;
        }
    }

    if (allSharedLocked)
    {
        problem.SetParameterBlockConstant(ptrUndistortionSharedParameters);
    }
    else
    {
        // At least one parameter is not locked

        std::vector<int> constantDistortions;
        for (int idParamDistortion = 0; idParamDistortion < lockShared.size(); ++idParamDistortion)
        {
            if (lockShared[idParamDistortion])
            {
                constantDistortions.push_back(idParamDistortion);
            }
        }

        if (!constantDistortions.empty())
        {
            ceres::SubsetManifold* subsetManifold = new ceres::SubsetManifold(countUndistortionParams, constantDistortions);
            problem.SetManifold(ptrUndistortionSharedParameters, subsetManifold);
        }
    }
    
    
    //Loop through views to create parameters
    for (auto & view : _views)
    {
        std::vector<double> & undistortionParameters = view.parameters;
        
        // Add distortion parameter
        double* ptrUndistortionParameters = &undistortionParameters[0];
        problem.AddParameterBlock(ptrUndistortionParameters, countUndistortionParams);

        // Check if all distortions are locked
        bool allLocked = true;
        for (bool lock : lockDistortionsNonShared)
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
            for (int idParamDistortion = 0; idParamDistortion < lockDistortionsNonShared.size(); ++idParamDistortion)
            {
                if (lockDistortionsNonShared[idParamDistortion])
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


        // Add off center parameter
        double* center = &view.distortionOffset.x();
        problem.AddParameterBlock(center, 2);
        if (lockCenter)
        {
            problem.SetParameterBlockConstant(center);
        }

        problem.AddParameterBlock(&view.scale, 1);

        // Optical center parameter
        problem.AddParameterBlock(&view.offsetx, 1);
        problem.AddParameterBlock(&view.offsety, 1);
        problem.SetParameterLowerBound(&view.offsetx, 0, view.offsetx - 100.0);
        problem.SetParameterUpperBound(&view.offsetx, 0, view.offsetx + 100.0);
        problem.SetParameterLowerBound(&view.offsety, 0, view.offsety - 100.0);
        problem.SetParameterUpperBound(&view.offsety, 0, view.offsety + 100.0);


        problem.AddParameterBlock(view.R.data(), 9);
        problem.SetManifold(view.R.data(), new sfm::SO3Manifold);
        problem.AddParameterBlock(view.t.data(), 3);

        for (auto& ppt : view.pointpairs)
        {
            ceres::CostFunction* costFunction = new CostPointGeometry(view.undistortion, 
                                                                    _sharedParams,
                                                                    ppt.undistortedPoint, ppt.distortedPoint, 
                                                                    pow(2.0, ppt.scale));
            problem.AddResidualBlock(costFunction, lossFunction, 
                                    center, 
                                    ptrUndistortionSharedParameters,
                                    ptrUndistortionParameters, 
                                    &view.scale, &view.offsetx, &view.offsety, 
                                    view.R.data(), view.t.data());
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

    
    std::vector<double> errors;    

    //Analyse result by computing statistics for all views/points
    for (auto & view : _views)
    {
        view.undistortion->setOffset(view.distortionOffset);

        for (int id = 0; id < _sharedParams.size(); id++)
        {
            if (_sharedParams[id])
            {
                view.parameters[id] = parametersShared[id];
            }
        }


        view.undistortion->setParameters(view.parameters);

        const double pa = (view.undistortion->isDesqueezed())?1.0:view.undistortion->getPixelAspectRatio();

        for (auto& ppt : view.pointpairs)
        {
            const Vec2 ipt = ppt.undistortedPoint;
            
            Vec3 ipt3;
            ipt3.x() = ipt.x();
            ipt3.y() = ipt.y();
            ipt3.z() = 0.0;

            const Vec3 tpt = view.R * ipt3 + view.t;
            
            Vec2 projected = tpt.head(2) / tpt(2);
            Vec2 scaled;
            scaled.x() = view.scale * projected.x() + view.offsetx;
            scaled.y() = pa * view.scale * projected.y() + view.offsety;

            double divider = pow(2.0, ppt.scale);

            const double res = (view.undistortion->undistort(ppt.distortedPoint) - scaled).norm() / divider;
            
            errors.push_back(res);
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

}  // namespace calibration
}  // namespace aliceVision
