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

namespace aliceVision
{
namespace hdr
{

using namespace aliceVision::image;

LaguerreBACalibration::LaguerreBACalibration() {}

template <typename T>
T laguerreFunction(const T& a, const T& x)
{
    // https://www.desmos.com/calculator/ib1y06t4pe
    using namespace boost::math::constants;
    constexpr double c = 2.0 / pi<double>();
    return x + c * atan((a * sin(pi<double>() * x)) / (1.0 - a * cos(pi<double>() * x)));
}
template <typename T>
T laguerreFunctionInv(const T& a, const T& x)
{
    return laguerreFunction(-a, x);
}

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
    double d_atanx_d_denom = -denom / (nom * nom + denom * denom);

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
    /*double atanx = atan(nom / denom);
    double res = x + c * atanx;*/

    double d_res_d_atanx = c;
    double d_atanx_d_nom = denom / (nom * nom + denom * denom);
    double d_atanx_d_denom = -denom / (nom * nom + denom * denom);

    double d_nom_d_sin_m_pi_x = a;
    double d_denom_d_cos_m_pi_x = -a;
    double d_sin_m_pi_x_d_m_pi_x = -cos(m_pi_x);
    double d_cos_m_pi_x_d_m_pi_x = sin(m_pi_x);
    double d_m_pi_x_d_x = M_PI;

    return d_res_d_atanx * (d_atanx_d_nom * d_nom_d_sin_m_pi_x * d_sin_m_pi_x_d_m_pi_x * d_m_pi_x_d_x +
                            d_atanx_d_denom * d_denom_d_cos_m_pi_x * d_cos_m_pi_x_d_m_pi_x * d_m_pi_x_d_x);
}

/**
 *
 */
struct HdrResidual
{
    HdrResidual(double a, double b)
        : _colorA(a)
        , _colorB(b)
    {
    }

    template <typename T>
    bool operator()(const T* const laguerreParam, const T* const expA, const T* const expB, T* residual) const
    {
        T laguerre_param = laguerreParam[0];
        T colorA = T(_colorA);
        T colorB = T(_colorB);

        T errorCost =
            laguerreFunctionInv(laguerre_param, laguerreFunction(laguerre_param, colorA) * (*expB) / (*expA)) - colorB;
        residual[0] = errorCost;

        return true;
    }

private:
    double _colorA;
    double _colorB;
};

class HdrResidualAnalytic : public ceres::SizedCostFunction<1, 1, 1, 1>
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
        double expA = parameters[1][0];
        double expB = parameters[2][0];

        double ratio_exp = (expB) / (expA);
        double a = laguerreFunction(laguerre_param, _colorA) * ratio_exp;
        double errorCost = laguerreFunctionInv(laguerre_param, a) - _colorB;
        residuals[0] = errorCost;

        if(jacobians == nullptr)
        {
            return true;
        }

        if(jacobians[0] != nullptr)
        {
            double d_laguerreFunctionInv_d_laguerre_param = -d_laguerreFunction_d_param(-laguerre_param, a) +
                                                            d_laguerreFunction_d_x(-laguerre_param, a) * ratio_exp *
                                                                d_laguerreFunction_d_param(laguerre_param, _colorA);

            jacobians[0][0] = d_laguerreFunctionInv_d_laguerre_param;
        }

        if(jacobians[1] != nullptr)
        {
            jacobians[1][0] = d_laguerreFunction_d_x(-laguerre_param, a) * laguerreFunction(laguerre_param, _colorA) * (-expB / (expA * expA));
        }

        if(jacobians[2] != nullptr)
        {
            jacobians[2][0] = d_laguerreFunction_d_x(-laguerre_param, a) * laguerreFunction(laguerre_param, _colorA) * (1.0 / expA);
        }

        return true;
    }

private:
    double _colorA;
    double _colorB;
};

void LaguerreBACalibration::process(const std::vector<std::vector<std::string>>& imagePathsGroups,
                                    const std::size_t channelQuantization,
                                    std::vector<std::vector<float>>& cameraExposures, int nbPoints, int imageDownscale,
                                    bool fisheye, bool refineExposures, rgbCurve& response)
{
    ALICEVISION_LOG_DEBUG("Extract color samples");
    std::vector<std::vector<ImageSamples>> samples;
    extractSamples(samples, imagePathsGroups, cameraExposures, nbPoints, imageDownscale, fisheye);

    ALICEVISION_LOG_DEBUG("Create exposure list");
    std::map<std::pair<int, float>, double> exposures;
    for(int i = 0; i < cameraExposures.size(); ++i)
    {
        const std::vector<float>& camExp = cameraExposures[i];
        for(int j = 0; j < camExp.size(); ++j)
        {
            const auto& exp = camExp[j];

            ALICEVISION_LOG_TRACE(" * " << imagePathsGroups[i][j] << ": " << exp);

            // TODO: camId
            exposures[std::make_pair(0, exp)] = exp;
        }
    }
    std::array<double, 3> laguerreParam = {0.0, 0.0, 0.0};
    std::array<double, 3> relativeWB = {1.0, 1.0, 1.0};

    ALICEVISION_LOG_DEBUG("Create BA problem");
    ceres::Problem problem;

    // Should we expose the LOSS function parameter?
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(Square(0.12)); // 0.12 ~= 30/255

    // In the Bundle Adjustment each image is optimized relativelty to the next one. The images are ordered by
    // exposures. But the exposures need to be different between 2 consecutive images to get constraints. So if multiple
    // images have been taken with the same parameters, we search for the next closest exposure (larger or smaller). For
    // example a dataset of 6 images with 4 different exposures:
    //       0        1        2        3        4        5
    //     1/800    1/800 -- 1/400 -- 1/200    1/200 -- 1/100
    //       \_________________/        \________________/
    // Without duplicates the relative indexes would be: [1, 2, 3, 4, 5, 4]
    // In this example with 2 duplicates, the relative indexes are: [2, 2, 3, 5, 5, 3]
    std::vector<std::vector<int>> closestRelativeExpIndex;
    {
        closestRelativeExpIndex.resize(cameraExposures.size());
        for(int i = 0; i < cameraExposures.size(); ++i)
        {
            const std::vector<float>& camExp = cameraExposures[i];
            std::vector<int>& camRelativeExpIndex = closestRelativeExpIndex[i];
            camRelativeExpIndex.resize(camExp.size(), -1);
            for(int j = 0; j < camExp.size(); ++j)
            {
                // Search in the following indexes
                for(int rj = j + 1; rj < camExp.size(); ++rj)
                {
                    if(camExp[rj] != camExp[j])
                    {
                        camRelativeExpIndex[j] = rj;
                        break;
                    }
                }
                if(camRelativeExpIndex[j] != -1)
                    continue;
                // Search in backward direction
                for(int rj = j - 1; rj >= 0; --rj)
                {
                    if(camExp[rj] != camExp[j])
                    {
                        camRelativeExpIndex[j] = rj;
                        break;
                    }
                }
            }
        }
        /*
        // Log relative indexes
        for (int i = 0; i < closestRelativeExpIndex.size(); ++i)
        {
            std::vector<int>& camRelativeExpIndex = closestRelativeExpIndex[i];
            for (int j = 0; j < camRelativeExpIndex.size(); ++j)
            {
                ALICEVISION_LOG_TRACE(" * closestRelativeExpIndex[" << i << "][" << j << "] = " <<
        closestRelativeExpIndex[i][j]);
            }
        }*/
    }

    bool first = true;
    // Convert selected samples into residual blocks
    for(int g = 0; g < samples.size(); ++g)
    {
        std::vector<ImageSamples>& hdrSamples = samples[g];

        ALICEVISION_LOG_TRACE("Group: " << g << ", hdr brakets: " << hdrSamples.size()
                                        << ", nb color samples: " << hdrSamples[0].colors.size());

        for(int i = 0; i < hdrSamples[0].colors.size(); ++i)
        {
            for(int h = 0; h < hdrSamples.size(); ++h)
            {
                int hNext = closestRelativeExpIndex[g][h];
                if(h == hNext)
                    throw std::runtime_error(
                        "Error in exposure chain. Relative exposure refer to itself (h: " + std::to_string(h) + ").");
                if(hNext < h)
                {
                    // The residual cost is bidirectional. So if 2 elements refer to each other, it will be a
                    // duplicates, so we have to skip one of them.
                    int hNextNext = closestRelativeExpIndex[g][hNext];
                    if(hNextNext == h)
                        continue;
                }

                double& expA = exposures[std::make_pair(0, cameraExposures[g][h])];
                double& expB = exposures[std::make_pair(0, cameraExposures[g][hNext])];

                /*problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(
                                            new HdrResidual( hdrSamples[h].colors[i].r(),
                hdrSamples[hNext].colors[i].r())), lossFunction, &(laguerreParam.data()[0]), &expA, &expB);

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(new HdrResidual(
                                             hdrSamples[hNext].colors[i].r(), hdrSamples[h].colors[i].r())),
                                         lossFunction, &(laguerreParam.data()[0]), &expB, &expA);

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(new HdrResidual(
                                             hdrSamples[h].colors[i].g(), hdrSamples[hNext].colors[i].g())),
                                         lossFunction, &(laguerreParam.data()[1]), &expA, &expB);

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(new HdrResidual(
                                             hdrSamples[hNext].colors[i].g(), hdrSamples[h].colors[i].g())),
                                         lossFunction, &(laguerreParam.data()[1]), &expB, &expA);

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(new HdrResidual(
                                             hdrSamples[h].colors[i].b(), hdrSamples[hNext].colors[i].b())),
                                         lossFunction, &(laguerreParam.data()[2]), &expA, &expB);

                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<HdrResidual, 1, 1, 1, 1>(new HdrResidual(
                                             hdrSamples[hNext].colors[i].b(), hdrSamples[h].colors[i].b())),
                                         lossFunction, &(laguerreParam.data()[2]), &expB, &expA);*/

                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[h].colors[i].r(), hdrSamples[hNext].colors[i].r()), lossFunction,
                    &(laguerreParam.data()[0]), &expA, &expB);
                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[hNext].colors[i].r(), hdrSamples[h].colors[i].r()), lossFunction,
                    &(laguerreParam.data()[0]), &expB, &expA);
                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[h].colors[i].g(), hdrSamples[hNext].colors[i].g()), lossFunction,
                    &(laguerreParam.data()[1]), &expA, &expB);
                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[hNext].colors[i].g(), hdrSamples[h].colors[i].g()), lossFunction,
                    &(laguerreParam.data()[1]), &expB, &expA);
                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[h].colors[i].b(), hdrSamples[hNext].colors[i].b()), lossFunction,
                    &(laguerreParam.data()[2]), &expA, &expB);
                problem.AddResidualBlock(
                    new HdrResidualAnalytic(hdrSamples[hNext].colors[i].b(), hdrSamples[h].colors[i].b()), lossFunction,
                    &(laguerreParam.data()[2]), &expB, &expA);

                if(first)
                {
                    problem.SetParameterBlockConstant(&expA);
                    first = false;
                }
            }
        }
    }

    if(!refineExposures)
    {
        for(auto& exp : exposures)
        {
            problem.SetParameterBlockConstant(&exp.second);
        }
    }

    ALICEVISION_LOG_INFO("BA Solve");

    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(solverOptions, &problem, &summary);

    ALICEVISION_LOG_TRACE(summary.BriefReport());

    ALICEVISION_LOG_INFO("Laguerre params: " << laguerreParam);
    ALICEVISION_LOG_INFO("Relative WB: " << relativeWB);
    ALICEVISION_LOG_INFO("Exposures:");
    for(const auto& expIt : exposures)
    {
        ALICEVISION_LOG_INFO(" * [" << expIt.first.first << ", " << expIt.first.second << "]: " << expIt.second);
    }

    for(unsigned int channel = 0; channel < 3; ++channel)
    {
        std::vector<float>& curve = response.getCurve(channel);
        const double step = 1.0 / double(curve.size());
        for(unsigned int i = 0; i < curve.size(); ++i)
        {
            curve[i] = relativeWB[channel] * laguerreFunction(laguerreParam[channel], i * step);
        }
    }

    if(refineExposures)
    {
        {
            // TODO: realign exposures on input values?
        }

        // Update input exposures with optimized exposure values
        for(int i = 0; i < cameraExposures.size(); ++i)
        {
            std::vector<float>& camExp = cameraExposures[i];
            for(int j = 0; j < camExp.size(); ++j)
            {
                camExp[j] = exposures[std::make_pair(0, camExp[j])];
            }
        }
    }
}

} // namespace hdr
} // namespace aliceVision
