// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LaguerreBACalibration.hpp"
#include "sampling.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>

#include <Eigen/Dense>

#include <ceres/ceres.h>

#include <utility>
#include <iostream>
#include <cassert>
#include <numeric>

namespace aliceVision {
namespace hdr {

using namespace aliceVision::image;

double d_laguerreFunction_d_param(double a, double x)
{
    double c = 2.0 / M_PI;

    double m_pi_x = M_PI * x;
    double sin_m_pi_x = sin(m_pi_x);
    double cos_m_pi_x = cos(m_pi_x);
    double nom = a * sin_m_pi_x;
    double denom = 1.0 - a * cos_m_pi_x;
    /*double atanx = atan(nom / denom);
    double res = x + c * atanx;*/

    double d_res_d_atanx = c;
    double d_atanx_d_nom = denom / (nom * nom + denom * denom);
    double d_atanx_d_denom = - nom / (nom * nom + denom * denom);

    double d_nom_d_a = sin_m_pi_x;
    double d_denom_d_a = -cos_m_pi_x;

    return d_res_d_atanx * (d_atanx_d_nom * d_nom_d_a + d_atanx_d_denom * d_denom_d_a);
}

double d_laguerreFunction_d_x(double a, double x)
{
    double c = 2.0 / M_PI;

    double m_pi_x = M_PI * x;
    double sin_m_pi_x = sin(m_pi_x);
    double cos_m_pi_x = cos(m_pi_x);
    double nom = a * sin_m_pi_x;
    double denom = 1.0 - a * cos_m_pi_x;

    double d_res_d_atanx = c;
    double d_atanx_d_nom = denom / (nom * nom + denom * denom);
    double d_atanx_d_denom = -nom / (nom * nom + denom * denom);

    double d_nom_d_sin_m_pi_x = a;
    double d_denom_d_cos_m_pi_x = -a;
    double d_sin_m_pi_x_d_m_pi_x = cos(m_pi_x);
    double d_cos_m_pi_x_d_m_pi_x = -sin(m_pi_x);
    double d_m_pi_x_d_x = M_PI;

    return 1.0 + d_res_d_atanx * (d_atanx_d_nom * d_nom_d_sin_m_pi_x * d_sin_m_pi_x_d_m_pi_x * d_m_pi_x_d_x + d_atanx_d_denom * d_denom_d_cos_m_pi_x * d_cos_m_pi_x_d_m_pi_x * d_m_pi_x_d_x);
}


class HdrResidualAnalytic : public ceres::SizedCostFunction<2, 1, 1>
{
public:
    HdrResidualAnalytic(double a, double b)
        : _colorA(a)
        , _colorB(b)
    {
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        double laguerre_param = parameters[0][0];
        double ratio_expB_over_expA = parameters[1][0];

        double a = laguerreFunctionInv(laguerre_param, _colorA) * ratio_expB_over_expA;
        double b = laguerreFunctionInv(laguerre_param, _colorB) / ratio_expB_over_expA;

        double errorCost_1 = laguerreFunction(laguerre_param, a) - _colorB;
        double errorCost_2 = laguerreFunction(laguerre_param, b) - _colorA;
        
        residuals[0] = errorCost_1;
        residuals[1] = errorCost_2;

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            double d_errorCost_1_d_laguerre_param = d_laguerreFunction_d_param(laguerre_param, a) + d_laguerreFunction_d_x(laguerre_param, a) * ratio_expB_over_expA * -d_laguerreFunction_d_param(-laguerre_param, _colorA);
            double d_errorCost_2_d_laguerre_param = d_laguerreFunction_d_param(laguerre_param, b) + d_laguerreFunction_d_x(laguerre_param, b) / ratio_expB_over_expA * -d_laguerreFunction_d_param(-laguerre_param, _colorB);

            jacobians[0][0] = d_errorCost_1_d_laguerre_param;
            jacobians[0][1] = d_errorCost_2_d_laguerre_param;
        }

        if(jacobians[1] != nullptr)
        {
            jacobians[1][0] = d_laguerreFunction_d_x(laguerre_param, a) * laguerreFunctionInv(laguerre_param, _colorA);
            jacobians[1][1] = d_laguerreFunction_d_x(laguerre_param, b) * laguerreFunctionInv(laguerre_param, _colorB) * (-1.0 / (ratio_expB_over_expA * ratio_expB_over_expA));
        }

        return true;
    }

private:
    double _colorA;
    double _colorB;
};

class ExposureConstraint : public ceres::SizedCostFunction<1, 1>
{
public:
    explicit ExposureConstraint(double ratio) : _ratio(ratio)
    {
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        double ratio_cur = parameters[0][0];

        const double w = 1000.0;
        residuals[0] = w * (ratio_cur - _ratio);

        if(jacobians == nullptr)
        {
            return true;
        }

        if (jacobians[0] != nullptr) {
            jacobians[0][0] = w;
        }

        return true;
    }
private:
    double _ratio;
};

void LaguerreBACalibration::process(const std::vector<std::vector<std::string>>& imagePathsGroups,
                                    const std::size_t channelQuantization,
                                    std::vector<std::vector<float>>& cameraExposures, int nbPoints, int imageDownscale,
                                    bool fisheye, bool refineExposures, rgbCurve& response)
{
    /*Extract samples from images*/
    ALICEVISION_LOG_DEBUG("Extract color samples");
    std::vector<std::vector<ImageSamples>> samples;
    extractSamples(samples, imagePathsGroups, cameraExposures, nbPoints, imageDownscale, fisheye);
    
    std::vector<std::vector<double>> exposuresRatios;

    for (std::vector<float> & group : cameraExposures) {
        std::vector<double> dest;

        for (int index = 0; index < group.size() - 1; index++) {
            
            double exposure_first = group[index];
            double exposure_second = group[index + 1];
            dest.push_back(exposure_second / exposure_first);
        }
        
        exposuresRatios.push_back(dest);
    }

    std::array<double, 3> laguerreParam = {0, 0, 0};

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    
    // Convert selected samples into residual blocks
    for (int groupId = 0; groupId < samples.size(); ++groupId)
    {
        std::vector<ImageSamples> & group = samples[groupId];

        for (int imageId = 0; imageId < group.size() - 1; imageId++) {

            const ImageSamples & samples = group[imageId];
            const ImageSamples & samplesOther = group[imageId + 1];

            double * ratioExp = &(exposuresRatios[groupId][imageId]);

            for(int sampleId = 0; sampleId < samples.colors.size(); sampleId++)
            {
                problem.AddResidualBlock(new HdrResidualAnalytic(samples.colors[sampleId].r(), samplesOther.colors[sampleId].r()), lossFunction, &(laguerreParam.data()[0]), ratioExp);
                problem.AddResidualBlock(new HdrResidualAnalytic(samples.colors[sampleId].g(), samplesOther.colors[sampleId].g()), lossFunction, &(laguerreParam.data()[1]), ratioExp);
                problem.AddResidualBlock(new HdrResidualAnalytic(samples.colors[sampleId].b(), samplesOther.colors[sampleId].b()), lossFunction, &(laguerreParam.data()[2]), ratioExp);
            }
        }
    }

    

    if (!refineExposures)
    {
        /*Fix exposures*/
        for (auto & group : exposuresRatios)
        {
            for(auto & expratio: group)
            {
                problem.SetParameterBlockConstant(&expratio);
            }
        }
    }
    else {
        for (auto & group : exposuresRatios)
        {
            for (double & ratioId : group)
            {
                problem.AddResidualBlock(new ExposureConstraint(ratioId), nullptr, &ratioId);
            }
        }
    }
    

    ALICEVISION_LOG_INFO("BA Solve");

    ceres::Solver::Options solverOptions;
    //solverOptions.minimizer_type = ceres::LINE_SEARCH;
    solverOptions.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    solverOptions.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = true;
    solverOptions.use_inner_iterations = true;
    solverOptions.use_nonmonotonic_steps = false;
    solverOptions.max_num_iterations = 100;
    solverOptions.function_tolerance = 1e-16;
    solverOptions.parameter_tolerance = 1e-16;
    
    ceres::Solver::Summary summary;
    ceres::Solve(solverOptions, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    for(unsigned int channel = 0; channel < 3; ++channel)
    {
        std::vector<float>& curve = response.getCurve(channel);
        const double step = 1.0 / double(curve.size());

        std::cout << laguerreParam[channel] << std::endl;
        for(unsigned int i = 0; i < curve.size(); ++i)
        {
            curve[i] = laguerreFunctionInv(laguerreParam[channel], i * step);
        }
    }

    if(refineExposures)
    {
        for(int groupId = 0; groupId < exposuresRatios.size(); groupId++)
        {   
            std::vector<double> & groupRatios = exposuresRatios[groupId];
            std::vector<float> & groupDestination = cameraExposures[groupId];

            for (int j = 0; j < groupRatios.size(); ++j)
            {
                groupDestination[j + 1] = groupDestination[j] * groupRatios[j];
            }
        }
    }
}

} // namespace hdr
} // namespace aliceVision
