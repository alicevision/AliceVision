// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DebevecCalibrate.hpp"
#include "sampling.hpp"

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

#include <OpenImageIO/imagebufalgo.h>

#include <iostream>
#include <fstream>
#include <cassert>


namespace aliceVision {
namespace hdr {


bool DebevecCalibrate::process(const std::vector<std::vector<ImageSample>>& ldrSamples,
                               const std::vector<std::vector<float>>& times, const std::size_t channelQuantization,
                               const rgbCurve& weight, float lambda, rgbCurve& response)
{
    // Always 3 channels for the input images
    static const std::size_t channelsCount = 3;


    // Count really extracted amount of points (observed in multiple brackets)
    std::vector<size_t> countPointPerGroup;
    size_t totalPoints = 0;
    for(size_t groupId = 0; groupId < ldrSamples.size(); groupId++)
    {
        const std::vector<ImageSample> & group = ldrSamples[groupId];
        totalPoints += group.size();
    }

    ALICEVISION_LOG_INFO("Debevec calibration with " << totalPoints << " samples.");

    // Initialize response
    response = rgbCurve(channelQuantization);

    // Store intermediate data for all three channels

    // Initialize intermediate buffers
    for(unsigned int channel = 0; channel < channelsCount; ++channel)
    {
        Eigen::MatrixXd A(channelQuantization, channelQuantization);
        Eigen::MatrixXd B(channelQuantization, totalPoints);
        Eigen::DiagonalMatrix<double, Eigen::Dynamic> Dinv(totalPoints);
        Eigen::VectorXd h1(channelQuantization);
        Eigen::VectorXd h2(totalPoints);

        // Initialize
        A.fill(0);
        B.fill(0);
        h1.fill(0);
        h2.fill(0);
        Dinv.setZero();

        size_t countPoints = 0;
        for(size_t groupId = 0; groupId < ldrSamples.size(); groupId++)
        {
            /*Process a group of brackets*/
            const std::vector<ImageSample>& group = ldrSamples[groupId];
            const std::vector<float> & local_times = times[groupId];

            for (size_t sampleId = 0; sampleId < group.size(); sampleId++) {
                
                const ImageSample & sample = group[sampleId];
                
                for (size_t bracketPos = 0; bracketPos < sample.descriptions.size(); bracketPos++) {
                    
                    const float time = std::log(sample.descriptions[bracketPos].exposure);

                    const float value = clamp(sample.descriptions[bracketPos].mean(channel), 0.0f, 1.0f);
                    const std::size_t quantizedValue = std::round(value * (channelQuantization - 1));
                    const std::size_t index = quantizedValue;

                    const float w_ij = std::max(1e-6f, weight(value, channel));
                    
                    const std::size_t pospoint = countPoints + sampleId;

                    const double w_ij_2 = w_ij * w_ij;
                    const double w_ij2_time = w_ij_2 * time;

                    Dinv.diagonal()[pospoint] += w_ij_2;
                    A(index, index) += w_ij_2;
                    B(index, pospoint) -= w_ij_2;
                    h1(index) += w_ij2_time;
                    h2(pospoint) += -w_ij2_time;
                }
            }

            countPoints += group.size();
            
        }

        // Make sure the discrete response curve has a minimal second derivative
        for(std::size_t k = 0; k < channelQuantization - 2; k++)
        {
            // Simple derivatives of second derivative wrt to the k+1 element
            // f''(x) = f(x + 1) - 2 * f(x) + f(x - 1)
            const float w = weight.getValue(k + 1, channel);

            const double v1 = lambda * w;
            const double v2 = -2.0f * lambda * w;
            const double v3 = lambda * w;

            A(k, k) += v1 * v1;
            A(k, k + 1) += v1 * v2;
            A(k, k + 2) += v1 * v3;

            A(k + 1, k) += v2 * v1;
            A(k + 1, k + 1) += v2 * v2;
            A(k + 1, k + 2) += v2 * v3;

            A(k + 2, k) += v3 * v1;
            A(k + 2, k + 1) += v3 * v2;
            A(k + 2, k + 2) += v3 * v3;
        }

        //
        // Fix scale
        // Enforce f(0.5) = 0.0
        //
        const size_t pos_middle = std::floor(channelQuantization / 2);
        A(pos_middle, pos_middle) += 1.0f;

        // M is
        //
        // [ATL ATR]   [[Mgradient       ][     0]]
        // [ABL ABR] = [[constraintMiddle][     0]]
        //             [Mleft               Mright]
        //
        // [A B] = [Atl Atr]^T [Atl Atr] = [Atl   0]^T [Atl   0]
        // [C D]   [Abl Abr]   [Abl Abr]   [Abl Abr]   [Abl Abr]
        // [A B] = [Atl^T Abl^T][Atl   0] = [Atl^TAtl + Abl^TAbl Abl^TAbr]
        // [C D]   [0     Abr^T][Abl Abr]   [Abr^TAbl            Abr^TAbr]
        // [h1] = [Atl^T bh + Abl^T bb] = [Abl^T bb]
        // [h2]   [Atr^T bh + Abr^T bb]   [Abr^T bb]
        const Eigen::MatrixXd C = B.transpose();

        for(int i = 0; i < Dinv.rows(); i++)
        {
            Dinv.diagonal()[i] = 1.0 / Dinv.diagonal()[i];
        }

        const Eigen::MatrixXd Bdinv = B * Dinv;
        const Eigen::MatrixXd left = A - Bdinv * C;
        const Eigen::VectorXd right = h1 - Bdinv * h2;

        const Eigen::VectorXd x = left.lu().solve(right);

        // Copy the result to the response curve
        for(std::size_t k = 0; k < channelQuantization; ++k)
        {
            response.setValue(k, channel, x(k));
        }
    }

    return true;
}

} // namespace hdr
} // namespace aliceVision
