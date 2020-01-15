// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GrossbergCalibrate.hpp"
#include <iostream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>
#include <Eigen/Dense>


namespace aliceVision {
namespace hdr {


GrossbergCalibrate::GrossbergCalibrate(const unsigned int dimension)
{
    _dimension = dimension;
}

void GrossbergCalibrate::process(const std::vector<std::vector<std::string>>& imagePathsGroups,
                                 const std::size_t channelQuantization,
                                 const std::vector< std::vector<float> > &times,
                                 const int nbPoints,
                                 const bool fisheye,
                                 rgbCurve &response)
{
    const int nbGroups = imagePathsGroups.size();
    const int nbImages = imagePathsGroups.front().size();
    const int samplesPerImage = nbPoints / (nbGroups*nbImages);

    //set channels count always RGB
    static const std::size_t channels = 3;

    //initialize response with g0 from invEmor
    response = rgbCurve(channelQuantization);
    response.setEmor();

    const std::size_t emorSize = std::pow(2, 10);

    Mat H(channelQuantization, _dimension);
    std::vector<std::vector<double>> hCurves(_dimension);

    for(unsigned int i=0; i<_dimension; ++i)
    {
        const double *h = getEmorInvCurve(i+1);
        if(emorSize == channelQuantization)
        {
            hCurves[i].assign(h, h + emorSize);
        }
        else if(emorSize > channelQuantization)
        {
            std::vector<double> emorH;
            emorH.assign(h, h + emorSize);
            std::vector<double> h0 = std::vector<double>(emorH.begin(), emorH.end());

            std::size_t step = emorSize/channelQuantization;
            for(std::size_t k = 0; k<channelQuantization; ++k)
                hCurves[i].emplace_back(h0.at(k*step));
        }
        else
        {
            std::vector<double> emorH;
            emorH.assign(h, h + emorSize);
            std::vector<double> h0 = std::vector<double>(emorH.begin(), emorH.end());

            std::size_t step = channelQuantization/emorSize;
            hCurves[i].resize(channelQuantization, 0.0);
            for(std::size_t k = 0; k<emorSize-1; ++k)
            {
                hCurves[i].at(k*step) = h0.at(k);
            }
            hCurves[i].at(emorSize*step-1) = h0.at(emorSize-1);
            std::size_t previousValidIndex = 0;
            for(std::size_t index = 1; index<channelQuantization; ++index)
            {
                if(hCurves[i].at(index) != 0.0f)
                {
                    if(previousValidIndex+1 < index)
                    {
                        const float inter = (hCurves[i].at(index) - hCurves[i].at(previousValidIndex)) / (index - previousValidIndex);
                        for(std::size_t j = previousValidIndex+1; j < index; ++j)
                        {
                            hCurves[i].at(j) = hCurves[i].at(previousValidIndex) + inter * (j-previousValidIndex);
                        }
                    }
                    previousValidIndex = index;
                }
            }
        }
        H.col(i) = Eigen::Map<Vec>(hCurves[i].data(), channelQuantization);
    }

    Mat A = Mat::Zero(samplesPerImage*(nbImages-1)*nbGroups*channels, _dimension);
    Vec b = Vec::Zero(samplesPerImage*(nbImages-1)*nbGroups*channels);

    if(fisheye)
    {
      int count = 0;

      for(unsigned int g=0; g<nbGroups; ++g)
      {
        const std::vector<std::string > &imagePaths = imagePathsGroups[g];
        std::vector<image::Image<image::RGBfColor>> ldrImagesGroup(imagePaths.size());

        for (int i = 0; i < imagePaths.size(); i++)
        {
            image::readImage(imagePaths[i], ldrImagesGroup[i], image::EImageColorSpace::SRGB);
        }

        const std::vector<float> &ldrTimes= times[g];

        const std::size_t width = ldrImagesGroup.front().Width();
        const std::size_t height = ldrImagesGroup.front().Height();

        const std::size_t minSize = std::min(width, height) * 0.97;
        const Vec2i center(width/2, height/2);

        const int xMin = std::ceil(center(0) - minSize/2);
        const int yMin = std::ceil(center(1) - minSize/2);
        const int xMax = std::floor(center(0) + minSize/2);
        const int yMax = std::floor(center(1) + minSize/2);
        const std::size_t maxDist2 = pow(minSize * 0.5, 2);

        ALICEVISION_LOG_TRACE("filling A and b matrices");

        for(unsigned int channel=0; channel<channels; ++channel)
        {
          const int step = std::ceil(minSize / sqrt(samplesPerImage));
          for(unsigned int j=0; j<nbImages-1; ++j)
          {
            const image::Image<image::RGBfColor> &image1 = ldrImagesGroup.at(j);
            const image::Image<image::RGBfColor> &image2 = ldrImagesGroup.at(j+1);
            const double k = ldrTimes.at(j+1)/ldrTimes.at(j);

            // fill A and b matrices with the equations
            for(int y = yMin; y <= yMax-step; y+=step)
            {
              for(int x = xMin; x <= xMax-step; x+=step)
              {
                std::size_t dist2 = pow(center(0)-x, 2) + pow(center(1)-y, 2);
                if(dist2 > maxDist2)
                  continue;

                double sample1 = clamp(image1(y, x)(channel), 0.f, 1.f);
                double sample2 = clamp(image2(y, x)(channel), 0.f, 1.f);

                std::size_t index1 = std::round((channelQuantization-1) * sample1);
                std::size_t index2 = std::round((channelQuantization-1) * sample2);

                b(count) = response.getCurve(channel).at(index2) - k * response.getCurve(channel).at(index1);
                for(unsigned int i=0; i<_dimension; ++i)
                  A(count, i) = k * H(index1, i) - H(index2, i);

                count += 1;
              }
            }
          }
        }
      }

      A.conservativeResize(count, Eigen::NoChange_t::NoChange);
      b.conservativeResize(count);
    }
    else
    {
      for(unsigned int g=0; g<nbGroups; ++g)
      {
        const std::vector<std::string > &imagePaths = imagePathsGroups[g];
        std::vector<image::Image<image::RGBfColor>> ldrImagesGroup(imagePaths.size());

        for (int i = 0; i < imagePaths.size(); i++)
        {
            ALICEVISION_LOG_INFO("Load " << imagePaths[i]);
            image::readImage(imagePaths[i], ldrImagesGroup[i], image::EImageColorSpace::SRGB);
        }

        const std::vector<float> &ldrTimes= times[g];

        const std::size_t width = ldrImagesGroup.front().Width();
        const std::size_t height = ldrImagesGroup.front().Height();

        for(unsigned int channel=0; channel<channels; ++channel)
        {
          const int step = std::floor(width*height / samplesPerImage);
          for(unsigned int j=0; j<nbImages-1; ++j)
          {
            const image::Image<image::RGBfColor> &image1 = ldrImagesGroup.at(j);
            const image::Image<image::RGBfColor> &image2 = ldrImagesGroup.at(j+1);
            const double k = ldrTimes.at(j+1)/ldrTimes.at(j);

            // fill A and b matrices with the equations
            for(unsigned int l=0; l<samplesPerImage; ++l)
            {
                double sample1 = std::max(0.f, std::min(1.f, image1(step*l)(channel)));
                double sample2 = std::max(0.f, std::min(1.f, image2(step*l)(channel)));

                std::size_t index1 = std::round((channelQuantization-1) * sample1);
                std::size_t index2 = std::round((channelQuantization-1) * sample2);

                b(g*channels*(nbImages-1)*samplesPerImage + channel*(nbImages-1)*samplesPerImage + j*samplesPerImage + l) = response.getCurve(channel).at(index2) - k * response.getCurve(channel).at(index1);
                for(unsigned int i=0; i<_dimension; ++i)
                  A(g*channels*(nbImages-1)*samplesPerImage + channel*(nbImages-1)*samplesPerImage + j*samplesPerImage + l, i) = k * H(index1, i) - H(index2, i);
            }
          }
        }
      }
    }

    ALICEVISION_LOG_TRACE("solving Ax=b system");

    // solve the system using QR decomposition
    Eigen::HouseholderQR<Mat> solver(A);
    Vec c = solver.solve(b);

    ALICEVISION_LOG_TRACE("system solved");

    double relative_error = (A*c - b).norm() / b.norm();
    ALICEVISION_LOG_DEBUG("relative error is : " << relative_error);

    ALICEVISION_LOG_DEBUG("emor coefficients are : ");

    for(unsigned int i=0; i<_dimension; ++i)
    {
      std::vector<double> temp_hCurve = hCurves[i];
      for(auto &value : temp_hCurve)
        value *= c(i);

      ALICEVISION_LOG_DEBUG(c(i));

      for(int channel=0; channel<channels; ++channel)
        std::transform(response.getCurve(channel).begin(), response.getCurve(channel).end(), temp_hCurve.begin(), response.getCurve(channel).begin(), std::plus<float>());
    }

}


} // namespace hdr
} // namespace aliceVision
