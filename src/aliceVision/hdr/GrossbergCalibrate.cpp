// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GrossbergCalibrate.hpp"
#include "QuadProg++.hpp"
#include "sampling.hpp"
#include <Eigen/Dense>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <cassert>
#include <iostream>

namespace aliceVision {
namespace hdr {

GrossbergCalibrate::GrossbergCalibrate(unsigned int dimension)
{
    _dimension = dimension;
}

void GrossbergCalibrate::process(const std::vector<std::vector<ImageSample>>& ldrSamples, const std::vector<std::vector<float>>& times, std::size_t channelQuantization, rgbCurve& response)
{
    const double step = 1.0 / double(channelQuantization - 1);

    // set channels count always RGB
    static const std::size_t channels = 3;

    // initialize response with g0 from invEmor
    response = rgbCurve(channelQuantization);
    response.setEmorInv(0);

    const std::size_t emorSize = std::pow(2, 10);
    if(channelQuantization != emorSize)
    {
        ALICEVISION_LOG_ERROR("Incompatible channel quantization");
        return;
    }

    // finv(f(e1*E)) = finv(f(e2*E))
    // g(Ba) - k*g(Bb) = 0
    // f0(Ba) + sum(c_i * f_i(Ba)) - k*f0(Bb) - k*sum(c_i * f_i(Bb)) = 0
    // sum(c_i * f_i(Ba)) - k*sum(c_i * f_i(Bb)) = k*f0(Bb) - f0(Ba)

    size_t count_measures = 0;
    for(size_t group = 0; group < ldrSamples.size(); group++)
    {
        const std::vector<ImageSample> & groupSamples = ldrSamples[group];

        for (size_t sampleId = 0; sampleId < groupSamples.size(); sampleId++) {
            count_measures += groupSamples[sampleId].descriptions.size() - 1;
        }
    }

    for(int channel = 0; channel < 3; channel++)
    {
        Eigen::MatrixXd E(count_measures, _dimension);
        Eigen::MatrixXd v(count_measures, 1);

        rgbCurve f0(channelQuantization);
        f0.setEmorInv(0);

        for(size_t dim = 0; dim < _dimension; dim++)
        {
            rgbCurve fdim(channelQuantization);
            fdim.setEmorInv(dim + 1);

            size_t rowId = 0;
            for(size_t groupId = 0; groupId < ldrSamples.size(); groupId++)
            {
                const std::vector<ImageSample>& groupSamples = ldrSamples[groupId];

                for(size_t sampleId = 0; sampleId < groupSamples.size(); sampleId++)
                {
                    const ImageSample & sample = groupSamples[sampleId];

                    for(size_t bracketPos = 0; bracketPos < sample.descriptions.size() - 1; bracketPos++)
                    {
                        image::Rgb<float> Ba = sample.descriptions[bracketPos].mean;
                        image::Rgb<float> Bb = sample.descriptions[bracketPos + 1].mean;

                        const double k = sample.descriptions[bracketPos].exposure / sample.descriptions[bracketPos + 1].exposure;

                        float valA = Ba(channel);
                        float valB = Bb(channel);

                        E(rowId, dim) = fdim(valA, 0) - k * fdim(valB, 0);
                        v(rowId, 0) = f0(valA, 0) - k * f0(valB, 0);
                        rowId++;
                    }
                }
            }
        }

        // Get first linear solution
        Eigen::VectorXd c = (E.transpose() * E).inverse() * E.transpose() * -v;
        Eigen::MatrixXd H = E.transpose() * E;
        Eigen::VectorXd d = (E.transpose() * v).col(0);


        // d (f0(val) + sum_i(c_i * f_i(val))) d_val > 0
        // d (f0(val)) + sum_i(d(c_i * f_i(val))) > 0
        // d (f0(val)) + sum_i(c_i * d_f_i(val)) > 0
        //
        // f(x) ~ f(x+1) - f(x)
        // d (f0(val)) + sum_i(c_i * f(val + 1) - c_i * f(val)) > 0

        Eigen::MatrixXd dF0(channelQuantization - 1, 1);
        dF0.setZero();
        for(int i = 0; i < channelQuantization - 1; i++)
        {
            double eval_cur = double(i) * step;
            double eval_next = double(i + 1) * step;

            dF0(i, 0) = (f0(eval_next, channel) - f0(eval_cur, channel)) / step;
        }

        Eigen::MatrixXd D(channelQuantization - 1, _dimension);
        D.setZero();

        for(int dim = 0; dim < _dimension; dim++)
        {
            rgbCurve fdim(channelQuantization);
            fdim.setEmorInv(dim + 1);

            for(int i = 0; i < channelQuantization - 1; i++)
            {
                double eval_cur = double(i) * step;
                double eval_next = double(i + 1) * step;
                D(i, dim) = (c(dim) * (fdim(eval_next, channel) - fdim(eval_cur, channel))) / step;
            }
        }

        Eigen::MatrixXd CE(_dimension, 1);
        for(int i = 0; i < 1; i++)
        {
            for(int j = 0; j < _dimension; j++)
            {
                CE(j, i) = 0.0;
            }
        }

        Eigen::VectorXd ce0(1);
        for(int i = 0; i < 1; i++)
        {
            ce0[i] = 0.0;
        }

        quadprogpp::solve_quadprog(H, d, CE, ce0, D.transpose(), dF0, c);

        // Create final curve
        std::vector<float>& curve = response.getCurve(channel);
        for(unsigned int i = 0; i < curve.size(); ++i)
        {
            rgbCurve f0(channelQuantization);
            f0.setEmorInv(0);

            const double val = double(i) * step;
            double curve_val = f0(val, 0);
            for(int d = 0; d < _dimension; d++)
            {

                rgbCurve fdim(channelQuantization);
                fdim.setEmorInv(d + 1);
                curve_val += c(d) * fdim(val, 0);
            }

            curve[i] = curve_val;
        }
    }
}

} // namespace hdr
} // namespace aliceVision
