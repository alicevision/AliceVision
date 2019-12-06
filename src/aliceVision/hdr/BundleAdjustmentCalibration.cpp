// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "BundleAdjustmentCalibration.hpp"

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

BundleAdjustmentCalibration::BundleAdjustmentCalibration()
{
}

struct ImageSamples
{
    std::vector<Rgb<double>> colors;
    int camId = 0;
    double exposure = 0.0;
};

void extractSamples(
    std::vector<std::vector<ImageSamples>>& out_samples,
    const std::vector<std::vector<std::string>>& imagePathsGroups,
    const std::vector< std::vector<float> >& cameraExposures,
    const int nbPoints,
    const bool fisheye
    )
{
    const int nbGroups = imagePathsGroups.size();
    out_samples.resize(nbGroups);

    int averallNbImages = 0;
    for (const auto& imgPaths : imagePathsGroups)
    {
        averallNbImages += imgPaths.size();
    }
    const int samplesPerImage = nbPoints / averallNbImages;

    ALICEVISION_LOG_TRACE("samplesPerImage: " << samplesPerImage);

    #pragma omp parallel for num_threads(3)
    for (int g = 0; g<nbGroups; ++g)
    {
        std::vector<ImageSamples>& out_hdrSamples = out_samples[g];

        const std::vector<std::string > &imagePaths = imagePathsGroups[g];
        out_hdrSamples.resize(imagePaths.size());

        const std::vector<float>& exposures = cameraExposures[g];

        for (unsigned int i = 0; i < imagePaths.size(); ++i)
        {
            out_hdrSamples[i].exposure = exposures[i];
            out_hdrSamples[i].colors.reserve(samplesPerImage);
            std::vector<Rgb<double>>& colors = out_hdrSamples[i].colors;

            Image<RGBfColor> img;
            readImage(imagePaths[i], img, EImageColorSpace::LINEAR);

            const std::size_t width = img.Width();
            const std::size_t height = img.Height();

            const std::size_t minSize = std::min(width, height) * 0.97;
            const Vec2i center(width / 2, height / 2);

            const int xMin = std::ceil(center(0) - minSize / 2);
            const int yMin = std::ceil(center(1) - minSize / 2);
            const int xMax = std::floor(center(0) + minSize / 2);
            const int yMax = std::floor(center(1) + minSize / 2);
            const std::size_t maxDist2 = pow(minSize * 0.5, 2);

            const int step = std::ceil(minSize / sqrt(samplesPerImage));

            // extract samples
            for (int y = yMin; y <= yMax - step; y += step)
            {
                for (int x = xMin; x <= xMax - step; x += step)
                {
                    if (fisheye)
                    {
                        std::size_t dist2 = pow(center(0) - x, 2) + pow(center(1) - y, 2);
                        if (dist2 > maxDist2)
                            continue;
                    }
                    RGBfColor& c = img(y, x);
                    colors.push_back(Rgb<double>(c(0), c(1), c(2)));
                }
            }
        }
    }
}


template<typename T>
T laguerreFunction(const T& a, const T& x)
{
    // https://www.desmos.com/calculator/ib1y06t4pe
    using namespace boost::math::constants;
    return x + 2.0 * pi<double>() * atan((a * sin(pi<double>() * x)) / (1.0 - a * cos(pi<double>() * x)));
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
            T a = laguerreFunction(greenParam, T(_colorA(green)));
            T b = laguerreFunction(greenParam, T(_colorB(green)));
            residual[green] = abs(laguerreFunctionInv(greenParam, laguerreFunction(greenParam, T(_colorA(green))) * (*expB) / (*expA)) - T(_colorB(green)))
                            + abs(laguerreFunctionInv(greenParam, laguerreFunction(greenParam, T(_colorB(green))) * (*expA) / (*expB)) - T(_colorA(green)));
        }
        {
            // RED
            T redParam = laguerreParam[green] + laguerreParam[red];
            T a = *relativeWB_R + laguerreFunction(redParam, T(_colorA(red)));
            T b = *relativeWB_R + laguerreFunction(redParam, T(_colorB(red)));
            residual[red] = abs(laguerreFunctionInv(redParam, laguerreFunction(redParam, T(_colorA(red))) * (*expB) / (*expA)) - T(_colorB(red)))
                            + abs(laguerreFunctionInv(redParam, laguerreFunction(redParam, T(_colorB(red))) * (*expA) / (*expB)) - T(_colorA(red)));
        }
        {
            // BLUE
            T blueParam = laguerreParam[green] + laguerreParam[blue];
            T a = *relativeWB_B + laguerreFunction(blueParam, T(_colorA(blue)));
            T b = *relativeWB_B + laguerreFunction(blueParam, T(_colorB(blue)));
            residual[blue] = abs(laguerreFunctionInv(blueParam, laguerreFunction(blueParam, T(_colorA(blue))) * (*expB) / (*expA)) - T(_colorB(blue)))
                          + abs(laguerreFunctionInv(blueParam, laguerreFunction(blueParam, T(_colorB(blue))) * (*expA) / (*expB)) - T(_colorA(blue)));
        }

        return true;
    }

private:
    const Rgb<double>& _colorA;
    const Rgb<double>& _colorB;
};

void BundleAdjustmentCalibration::process(
                                const std::vector<std::vector<std::string>>& imagePathsGroups,
                                const std::size_t channelQuantization,
                                const std::vector<std::vector<float>>& cameraExposures,
                                int nbPoints,
                                bool fisheye,
                                bool refineExposures,
                                rgbCurve &response)
{
    ALICEVISION_LOG_TRACE("Extract color samples");
    std::vector<std::vector<ImageSamples>> samples;
    extractSamples(samples, imagePathsGroups, cameraExposures, nbPoints, fisheye);

    ALICEVISION_LOG_TRACE("Create exposure list");
    std::map<std::pair<int, float>, double> exposures;
    for(const auto& camExp: cameraExposures)
    {
        for (const auto& exp : camExp)
        {
            // TODO: camId
            exposures[std::make_pair(0, exp)] = exp;
        }
    }
    std::array<double, 3> laguerreParam = { 0.0, 0.0, 0.0 };
    std::array<double, 3> relativeWB = { 0.0, 0.0, 0.0 };

    ALICEVISION_LOG_TRACE("Create BA problem");
    ceres::Problem problem;

    // TODO: make the LOSS function and the parameter an option
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(Square(0.12)); // 0.12 ~= 30/255

    for (int g = 0; g < samples.size(); ++g)
    {
        std::vector<ImageSamples>& hdrSamples = samples[g];

        for (int i = 0; i < hdrSamples[0].colors.size(); ++i)
        {
            for (int h = 0; h < hdrSamples.size() - 1; ++h)
            {
                ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<HdrResidual, 3, 3, 1, 1, 1, 1>(
                                                new HdrResidual(hdrSamples[h].colors[i], hdrSamples[h+1].colors[i]));

                double& expA = exposures[std::make_pair(0, cameraExposures[g][h])];
                double& expB = exposures[std::make_pair(0, cameraExposures[g][h+1])];

                problem.AddResidualBlock(cost_function, lossFunction, laguerreParam.data(), &relativeWB[0], &relativeWB[2], &expA, &expB);
            }
        }
    }

    ALICEVISION_LOG_DEBUG("BA Solve");

    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    solverOptions.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(solverOptions, &problem, &summary);

    ALICEVISION_LOG_DEBUG(summary.BriefReport());

    ALICEVISION_LOG_TRACE("laguerreParam: " << laguerreParam);
    ALICEVISION_LOG_TRACE("relativeWB: " << relativeWB);

    ALICEVISION_LOG_TRACE("Exposures:");
    for (const auto& expIt: exposures)
    {
        ALICEVISION_LOG_TRACE(" * [" << expIt.first.first << ", " << expIt.first.second << "]: " << expIt.second);
    }

    for(unsigned int channel = 0; channel < 3; ++channel)
    {
        std::vector<float>& curve = response.getCurve(channel);
        double step = 1.0f / curve.size();
        for (unsigned int i = 0; i < curve.size(); ++i)
        {
            const double offset = ((channel == 1) ? 0 : laguerreParam[1]);  // non-green channels are relative to green channel
            curve[i] = relativeWB[channel] + laguerreFunction(offset + laguerreParam[channel], i * step);
        }
    }
}


} // namespace hdr
} // namespace aliceVision
