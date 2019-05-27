// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GrossbergCalibrate.hpp"
#include <iostream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include "eiquadprog.hpp"


namespace aliceVision {
namespace hdr {


GrossbergCalibrate::GrossbergCalibrate(const unsigned int dimension, const std::size_t channelQuantization)
{
    _dimension = dimension;
    _channelQuantization = channelQuantization;
}

void GrossbergCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                                 const std::vector< std::vector<float> > &times,
                                 const int nbPoints,
                                 const rgbCurve &weight,
                                 rgbCurve &response)
{

    //checks
    for (int g = 0; g < ldrImageGroups.size(); ++g)
    {
      assert(ldrImageGroups[g].size() == times[g].size());
    }

    //set channels count always RGB
    static const std::size_t channels = 3;

    //initialize response with g0 from invEmor
    response = rgbCurve(_channelQuantization);
    rgbCurve temporaryResponse(_channelQuantization);
    const double* ptrf0 = getEmorInvCurve(0);
    std::vector<double> f0;
    f0.assign(ptrf0, ptrf0 + _channelQuantization);

    Mat H(_channelQuantization, _dimension);
    std::vector<double> hCurves[_dimension];

    for(unsigned int i=0; i<_dimension; ++i)
    {
        const double *h = getEmorInvCurve(i+1);
        hCurves[i].assign(h, h + _channelQuantization);
        H.col(i) = Eigen::Map<Vec>(hCurves[i].data(), _channelQuantization);
    }

    for(unsigned int g=0; g<ldrImageGroups.size(); ++g)
    {
        const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups.at(g);
        const std::vector<float> &ldrTimes= times.at(g);

        const int nbImages = ldrImagesGroup.size();
        const int nbPixels = ldrImagesGroup.at(0).Width() * ldrImagesGroup.at(0).Height();
        const int step = std::floor(nbPixels/nbPoints);


        for(unsigned int channel=0; channel<channels; ++channel)
        {
            response.getCurve(channel) = std::vector<float>(f0.begin(), f0.end());
            temporaryResponse.getCurve(channel) = std::vector<float>(f0.begin(), f0.end());

            Mat A = Mat::Zero(nbPoints*(nbImages-1), _dimension);
            Vec b = Vec::Zero(nbPoints*(nbImages-1));

            std::cout << "filling A and b matrices" << std::endl;

            for(unsigned int j=0; j<nbImages-1; ++j)
            {
                const image::Image<image::RGBfColor> &image1 = ldrImagesGroup.at(j);
                const image::Image<image::RGBfColor> &image2 = ldrImagesGroup.at(j+1);
                const double k = ldrTimes.at(j+1)/ldrTimes.at(j);

                // fill A and b matrices with the equations
                for(unsigned int l=0; l<nbPoints; ++l)
                {
                    double sample1 = std::max(0.f, std::min(1.f, image1(step*l)(channel)));
                    double sample2 = std::max(0.f, std::min(1.f, image2(step*l)(channel)));
                    std::size_t index1 = std::round((_channelQuantization-1) * sample1);
                    std::size_t index2 = std::round((_channelQuantization-1) * sample2);

                    double w = std::min(weight(sample1, channel), weight(sample2, channel));
                    if(std::max(sample1, sample2) > 0.95)
                        w = 0.0;

                    b(j*nbPoints + l) = w * (f0.at(index2) - k * f0.at(index1));
                    for(unsigned int i=0; i<_dimension; ++i)
                        A(j*nbPoints + l, i) = w * (H(index1, i) - k * H(index2, i));

                }
            }

            std::cout << "solving Ax=b system" << std::endl;

            // solve the system using QR decomposition
            Eigen::HouseholderQR<Mat> solver(A);
            Vec c = solver.solve(b);

            std::cout << "system solved" << std::endl;

            double relative_error = (A*c - b).norm() / b.norm();
            std::cout << "relative error is : " << relative_error << std::endl;

            for(unsigned int i=0; i<_dimension; ++i)
            {
                std::vector<double> temp_hCurve = hCurves[i];
                for(auto &value : temp_hCurve)
                    value *= c(i);

                std::transform(temporaryResponse.getCurve(channel).begin(), temporaryResponse.getCurve(channel).end(), temp_hCurve.begin(), temporaryResponse.getCurve(channel).begin(), std::plus<float>());
            }

            std::cout << "c : " << c << std::endl;
        }

        for(unsigned int channel=0; channel<channels; ++channel)
        {
            // Imposing monocity of the response function by quadratic programming
            std::vector<double> fdiff(_channelQuantization);
            std::transform(temporaryResponse.getCurve(channel).begin(), temporaryResponse.getCurve(channel).end(), f0.begin(), fdiff.begin(), std::minus<float>());
            //                std::transform(f0.begin(), f0.end(), temporaryResponse.getCurve(channel).begin(), fdiff.begin(), std::minus<float>());
            Vec v = Eigen::Map<Vec>(fdiff.data(), fdiff.size());
            Mat D = Mat::Zero(_channelQuantization-1, _channelQuantization);
            for(std::size_t y=0; y<_channelQuantization-1; ++y)
            {
//                D(y,y) = -1.f;
//                D(y,y+1) = 1.f;
                D(y,y) = -1.0 * _channelQuantization;
                D(y,y+1) = _channelQuantization;
            }

            Mat G = H.transpose() * H;
            Vec g0 = -H.transpose() * v;
            Vec x;
            Mat CE(_dimension, _channelQuantization);
            Vec ce0(_channelQuantization);
            Mat CI = (D*H).transpose();
            Vec ci0 = D * Eigen::Map<Vec>(f0.data(), f0.size());

            double cost = Eigen::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
            if(cost == std::numeric_limits<double>::infinity())     //optimization failed
                return;

            std::cout << "x : " << x << std::endl;

            // Calculate final monotonic response function
            for(unsigned int i=0; i<_dimension; ++i)
            {
                std::vector<double> final_hCurve = hCurves[i];
                for(auto &value : final_hCurve)
                    value *= x(i);

                std::transform(response.getCurve(channel).begin(), response.getCurve(channel).end(), final_hCurve.begin(), response.getCurve(channel).begin(), std::plus<float>());
            }
        }
    }
}


} // namespace hdr
} // namespace aliceVision
