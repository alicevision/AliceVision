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
    double d_atanx_d_denom = -nom / (nom * nom + denom * denom);

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

    return 1.0 + d_res_d_atanx * (d_atanx_d_nom * d_nom_d_sin_m_pi_x * d_sin_m_pi_x_d_m_pi_x * d_m_pi_x_d_x +
                                  d_atanx_d_denom * d_denom_d_cos_m_pi_x * d_cos_m_pi_x_d_m_pi_x * d_m_pi_x_d_x);
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
            double d_errorCost_1_d_laguerre_param = d_laguerreFunction_d_param(laguerre_param, a) +
                                                    d_laguerreFunction_d_x(laguerre_param, a) * ratio_expB_over_expA *
                                                        -d_laguerreFunction_d_param(-laguerre_param, _colorA);
            double d_errorCost_2_d_laguerre_param = d_laguerreFunction_d_param(laguerre_param, b) +
                                                    d_laguerreFunction_d_x(laguerre_param, b) / ratio_expB_over_expA *
                                                        -d_laguerreFunction_d_param(-laguerre_param, _colorB);

            jacobians[0][0] = d_errorCost_1_d_laguerre_param;
            jacobians[0][1] = d_errorCost_2_d_laguerre_param;
        }

        if(jacobians[1] != nullptr)
        {
            jacobians[1][0] = d_laguerreFunction_d_x(laguerre_param, a) * laguerreFunctionInv(laguerre_param, _colorA);
            jacobians[1][1] = d_laguerreFunction_d_x(laguerre_param, b) * laguerreFunctionInv(laguerre_param, _colorB) *
                              (-1.0 / (ratio_expB_over_expA * ratio_expB_over_expA));
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
    explicit ExposureConstraint(double ratio)
        : _ratio(ratio)
    {
    }

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
    {
        double ratio_cur = parameters[0][0];

        const double w = 1.0;
        residuals[0] = w * (ratio_cur - _ratio);

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            jacobians[0][0] = w;
        }

        return true;
    }

private:
    double _ratio;
};

void LaguerreBACalibration::process(const std::vector<std::vector<ImageSample>>& ldrSamples,
                                    std::vector<std::vector<float>>& cameraExposures,
                                    const std::size_t channelQuantization,
                                    bool refineExposures, rgbCurve& response)
{
    std::map<std::pair<float, float>, double> exposureParameters;
    for(std::vector<float>& group : cameraExposures)
    {
        for(int index = 0; index < group.size() - 1; index++)
        {
            std::pair<float, float> exposurePair;
            exposurePair.first = group[index];
            exposurePair.second = group[index + 1];
            exposureParameters[exposurePair] = double(exposurePair.second) / double(exposurePair.first);
        } 
    }

    std::array<double, 3> laguerreParam = {0.0, 0.0, 0.0};

    ceres::Problem problem;
    ceres::LossFunction* lossFunction = nullptr;

    for(auto& param : exposureParameters) {
        problem.AddParameterBlock(&param.second, 1);
    }

    // Convert selected samples into residual blocks
    for(int groupId = 0; groupId < ldrSamples.size(); ++groupId)
    {
        const std::vector<ImageSample> & group = ldrSamples[groupId];

        for (int sampleId = 0; sampleId < group.size(); sampleId++) {

            const ImageSample & sample = group[sampleId];

            for (int bracketPos = 0; bracketPos < sample.descriptions.size() - 1; bracketPos++) {
                
                std::pair<float, float> exposurePair;
                exposurePair.first = sample.descriptions[bracketPos].exposure;
                exposurePair.second = sample.descriptions[bracketPos + 1].exposure;

                double * expParam = &exposureParameters[exposurePair];

                for (int channel = 0; channel < 3; channel++) {
                    ceres::CostFunction * cost = new HdrResidualAnalytic(sample.descriptions[bracketPos].mean(channel), sample.descriptions[bracketPos + 1].mean(channel));
                    problem.AddResidualBlock(cost, lossFunction, &(laguerreParam.data()[channel]), expParam);
                }
            }
        }
    }

    if(!refineExposures)
    {
        for(auto& param : exposureParameters)
        {
            problem.AddParameterBlock(&param.second, 1);
            problem.SetParameterBlockConstant(&param.second);
        }
    }
    else
    {
        for(auto& param : exposureParameters)
        {
            problem.AddResidualBlock(new ExposureConstraint(param.second), nullptr, &param.second);
        }
    }

    ALICEVISION_LOG_INFO("BA Solve");

    ceres::Solver::Options solverOptions;
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

    for (unsigned int channel = 0; channel < 3; ++channel)
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
        for(size_t idGroup = 0; idGroup < cameraExposures.size(); idGroup++) 
        {   
            std::vector<float> & group = cameraExposures[idGroup];
            
            //Copy !
            std::vector<float> res = cameraExposures[idGroup];
        
            for(int index = 0; index < group.size() - 1; index++)
            {
                std::pair<float, float> exposurePair;
                exposurePair.first = group[index];
                exposurePair.second = group[index + 1];
                double value = exposureParameters[exposurePair];
                res[index + 1] = (res[res.size() - 1] * value);
            }

            cameraExposures[idGroup] = res;
        }
    }
}

} // namespace hdr
} // namespace aliceVision
