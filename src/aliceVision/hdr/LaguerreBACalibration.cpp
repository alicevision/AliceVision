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

LaguerreBACalibration::LaguerreBACalibration()
{
}

template<typename T>
T laguerreFunction(const T& a, const T& x)
{
    // https://www.desmos.com/calculator/ib1y06t4pe
    using namespace boost::math::constants;
    constexpr double c = 2.0 / pi<double>();
    return x + c * atan((a * sin(pi<double>() * x)) / (1.0 - a * cos(pi<double>() * x)));
}
template<typename T>
T laguerreFunctionInv(const T& a, const T& x)
{
    return laguerreFunction(-a, x);
}

/**
 * 
 */
struct HdrResidual
{
    HdrResidual(const Rgb<double>& a, const Rgb<double>& b)
        : _colorA(a)
        , _colorB(b)
    {}

    template <typename T>
    bool operator()(const T* const laguerreParam, const T* const relativeWB_R, const T* const relativeWB_B, const T* const expA, const T* const expB, T* residual) const
    {
        static const int red = 0;
        static const int green = 1;
        static const int blue = 2;
        /*
        {
            // GREEN
            T greenParam = laguerreParam[green];
            T a = laguerreFunction(greenParam, T(_colorA(green)));
            T b = laguerreFunction(greenParam, T(_colorB(green)));
            residual[green] = abs(a * (*expB) / (*expA) - b) + abs(a - b * (*expA) / (*expB));
        }
        {
            // RED
            T redParam = laguerreParam[green] + laguerreParam[red];
            T a = *relativeWB_R + laguerreFunction(redParam, T(_colorA(red)));
            T b = *relativeWB_R + laguerreFunction(redParam, T(_colorB(red)));
            residual[red] = abs(a * (*expB) / (*expA) - b) + abs(a - b * (*expA) / (*expB));
        }
        {
            // BLUE
            T blueParam = laguerreParam[green] + laguerreParam[blue];
            T a = *relativeWB_B + laguerreFunction(blueParam, T(_colorA(blue)));
            T b = *relativeWB_B + laguerreFunction(blueParam, T(_colorB(blue)));
            residual[blue] = abs(a * (*expB) / (*expA) - b) + abs(a - b * (*expA) / (*expB));
        }
        */
        {
            // GREEN
            T greenParam = laguerreParam[green];
            T colorA = T(_colorA(green));
            T colorB = T(_colorB(green));
            // T a = laguerreFunction(greenParam, colorA);
            // T b = laguerreFunction(greenParam, colorB);
            T errorCost = abs(laguerreFunctionInv(greenParam, laguerreFunction(greenParam, colorA) * (*expB) / (*expA)) - colorB)
                        + abs(laguerreFunctionInv(greenParam, laguerreFunction(greenParam, colorB) * (*expA) / (*expB)) - colorA);
            residual[green] = errorCost;
        }
        {
            // RED
            T redParam = laguerreParam[green] + laguerreParam[red];
            T colorA = T(_colorA(red));
            T colorB = T(_colorB(red));
            // T a = *relativeWB_R * laguerreFunction(redParam, colorA);
            // T b = *relativeWB_R * laguerreFunction(redParam, colorB);
            T errorCost = abs(laguerreFunctionInv(redParam, laguerreFunction(redParam, colorA) * (*expB) / (*expA)) - colorB)
                        + abs(laguerreFunctionInv(redParam, laguerreFunction(redParam, colorB) * (*expA) / (*expB)) - colorA);
            residual[red] = errorCost;
        }
        {
            // BLUE
            T blueParam = laguerreParam[green] + laguerreParam[blue];
            T colorA = T(_colorA(blue));
            T colorB = T(_colorB(blue));
            // T a = *relativeWB_B * laguerreFunction(blueParam, colorA);
            // T b = *relativeWB_B * laguerreFunction(blueParam, colorB);
            T errorCost = abs(laguerreFunctionInv(blueParam, laguerreFunction(blueParam, colorA) * (*expB) / (*expA)) - colorB)
                        + abs(laguerreFunctionInv(blueParam, laguerreFunction(blueParam, colorB) * (*expA) / (*expB)) - colorA);
            residual[blue] = errorCost;
        }

        return true;
    }

private:
    const Rgb<double>& _colorA; // TODO: T[3]
    const Rgb<double>& _colorB;
};

void LaguerreBACalibration::process(
                                const std::vector<std::vector<std::string>>& imagePathsGroups,
                                const std::size_t channelQuantization,
                                std::vector<std::vector<float>>& cameraExposures,
                                int nbPoints,
                                int imageDownscale,
                                bool fisheye,
                                bool refineExposures,
                                rgbCurve &response)
{
    ALICEVISION_LOG_DEBUG("Extract color samples");
    std::vector<std::vector<ImageSamples>> samples;
    extractSamples(samples, imagePathsGroups, cameraExposures, nbPoints, imageDownscale, fisheye);

    ALICEVISION_LOG_DEBUG("Create exposure list");
    std::map<std::pair<int, float>, double> exposures;
    for(int i = 0; i < cameraExposures.size(); ++i)
    {
        const std::vector<float>& camExp = cameraExposures[i];
        for (int j = 0; j < camExp.size(); ++j)
        {
            const auto& exp = camExp[j];

            ALICEVISION_LOG_TRACE(" * " << imagePathsGroups[i][j] << ": " << exp);

            // TODO: camId
            exposures[std::make_pair(0, exp)] = exp;
        }
    }
    std::array<double, 3> laguerreParam = { 0.0, 0.0, 0.0 };
    std::array<double, 3> relativeWB = { 1.0, 1.0, 1.0 };

    ALICEVISION_LOG_DEBUG("Create BA problem");
    ceres::Problem problem;

    // Should we expose the LOSS function parameter?
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(Square(0.12)); // 0.12 ~= 30/255

    // In the Bundle Adjustment each image is optimized relativelty to the next one. The images are ordered by exposures.
    // But the exposures need to be different between 2 consecutive images to get constraints.
    // So if multiple images have been taken with the same parameters, we search for the next closest exposure (larger or smaller).
    // For example a dataset of 6 images with 4 different exposures:
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
                for (int rj = j + 1; rj < camExp.size(); ++rj)
                {
                    if (camExp[rj] != camExp[j])
                    {
                        camRelativeExpIndex[j] = rj;
                        break;
                    }
                }
                if(camRelativeExpIndex[j] != -1)
                    continue;
                // Search in backward direction
                for (int rj = j - 1; rj >= 0; --rj)
                {
                    if (camExp[rj] != camExp[j])
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
                ALICEVISION_LOG_TRACE(" * closestRelativeExpIndex[" << i << "][" << j << "] = " << closestRelativeExpIndex[i][j]);
            }
        }*/
    }

    // Convert selected samples into residual blocks
    for (int g = 0; g < samples.size(); ++g)
    {
        std::vector<ImageSamples>& hdrSamples = samples[g];

        ALICEVISION_LOG_TRACE("Group: " << g << ", hdr brakets: " << hdrSamples.size() << ", nb color samples: " << hdrSamples[0].colors.size());

        for (int i = 0; i < hdrSamples[0].colors.size(); ++i)
        {
            for (int h = 0; h < hdrSamples.size(); ++h)
            {
                int hNext = closestRelativeExpIndex[g][h];
                if(h == hNext)
                    throw std::runtime_error("Error in exposure chain. Relative exposure refer to itself (h: " + std::to_string(h) + ").");
                if (hNext < h)
                {
                    // The residual cost is bidirectional. So if 2 elements refer to each other, it will be a duplicates, so we have to skip one of them.
                    int hNextNext = closestRelativeExpIndex[g][hNext];
                    if(hNextNext == h)
                        continue;
                }
                ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<HdrResidual, 3, 3, 1, 1, 1, 1>(
                                                new HdrResidual(hdrSamples[h].colors[i], hdrSamples[hNext].colors[i]));

                double& expA = exposures[std::make_pair(0, cameraExposures[g][h])];
                double& expB = exposures[std::make_pair(0, cameraExposures[g][hNext])];

                problem.AddResidualBlock(cost_function, lossFunction, laguerreParam.data(), &relativeWB[0], &relativeWB[2], &expA, &expB);
            }
        }
    }

    if (!refineExposures)
    {
        for(auto& exp: exposures)
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
    for (const auto& expIt: exposures)
    {
        ALICEVISION_LOG_INFO(" * [" << expIt.first.first << ", " << expIt.first.second << "]: " << expIt.second);
    }

    for(unsigned int channel = 0; channel < 3; ++channel)
    {
        std::vector<float>& curve = response.getCurve(channel);
        const double step = 1.0 / double(curve.size());
        for (unsigned int i = 0; i < curve.size(); ++i)
        {
            const double offset = ((channel == 1) ? 0 : laguerreParam[1]);  // non-green channels are relative to green channel
            curve[i] = relativeWB[channel] * laguerreFunction(offset + laguerreParam[channel], i * step);
        }
    }

    if (refineExposures)
    {
        {
            // TODO: realign exposures on input values?
        }


        // Update input exposures with optimized exposure values
        for (int i = 0; i < cameraExposures.size(); ++i)
        {
            std::vector<float>& camExp = cameraExposures[i];
            for (int j = 0; j < camExp.size(); ++j)
            {
                camExp[j] = exposures[std::make_pair(0, camExp[j])];
            }
        }
    }
}


} // namespace hdr
} // namespace aliceVision
