// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DebevecCalibrate.hpp"
#include <iostream>
#include <fstream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace hdr {

using T = Eigen::Triplet<double>;

void DebevecCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                               const std::size_t channelQuantization,
                               const std::vector< std::vector<float> > &times,
                               const int nbPoints,
                               const bool fisheye,
                               const rgbCurve &weight,
                               const float lambda,
                               rgbCurve &response)
{
  const int nbGroups = ldrImageGroups.size();
  const int nbImages = ldrImageGroups.front().size();
  const int samplesPerImage = nbPoints / (nbGroups*nbImages);

  //set channels count always RGB
  static const std::size_t channels = 3;

  //initialize response
  response = rgbCurve(channelQuantization);

  for(unsigned int channel=0; channel<channels; ++channel)
  {
    Vec b = Vec::Zero(nbPoints + channelQuantization + 1);
    int count = 0;

    std::vector<T> tripletList;
    tripletList.reserve(2 * nbPoints + 1 + 3 * channelQuantization);

    ALICEVISION_LOG_TRACE("filling A and b matrices");

    for(unsigned int g=0; g<nbGroups; ++g)
    {
      const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups[g];
      const std::vector<float> &ldrTimes = times[g];
      const std::size_t width = ldrImagesGroup.front().Width();
      const std::size_t height = ldrImagesGroup.front().Height();

      // include the data-fitting equations
      // if images are fisheye, we take only pixels inside a disk with a radius of image's minimum side
      if(fisheye)
      {
        const std::size_t minSize = std::min(width, height) * 0.97;
        const Vec2i center(width/2, height/2);

        const int xMin = std::ceil(center(0) - minSize/2);
        const int yMin = std::ceil(center(1) - minSize/2);
        const int xMax = std::floor(center(0) + minSize/2);
        const int yMax = std::floor(center(1) + minSize/2);
        const std::size_t maxDist2 = pow(minSize * 0.5, 2);

        const int step = std::ceil(minSize / sqrt(samplesPerImage));
        for(unsigned int j=0; j<nbImages; ++j)
        {
          int countValidPixels = 0;

          const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
          const float time = std::log(ldrTimes.at(j));
          for(int y = yMin; y < yMax-step; y+=step)
          {
            for(int x = xMin; x < xMax-step; x+=step)
            {
              std::size_t dist2 = pow(center(0)-x, 2) + pow(center(1)-y, 2);
              if(dist2 > maxDist2)
                continue;

              float sample = clamp(image(y, x)(channel), 0.f, 1.f);
              float w_ij = weight(sample, channel);
              std::size_t index = std::round(sample * (channelQuantization - 1));

              tripletList.push_back(T(count, index, w_ij));
              tripletList.push_back(T(count, channelQuantization + g*samplesPerImage + countValidPixels, -w_ij));

              b(count) = w_ij * time;
              count += 1;
              countValidPixels += 1;
            }
          }
        }
      }
      else
      {
        const int step = std::floor(width*height / samplesPerImage);
        for(unsigned int j=0; j<nbImages; ++j)
        {
          const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
          const float time = std::log(ldrTimes.at(j));
          for(unsigned int i=0; i<samplesPerImage; ++i)
          {
            float sample = clamp(image(step*i)(channel), 0.f, 1.f);
            float w_ij = weight(sample, channel);
            std::size_t index = std::round(sample * (channelQuantization - 1));

            tripletList.push_back(T(count, index, w_ij));
            tripletList.push_back(T(count, channelQuantization + g*samplesPerImage + i, -w_ij));

            b(count) = w_ij * time;
            count += 1;
          }
        }
      }
    }

    // fix the curve by setting its middle value to zero
    tripletList.push_back(T(count, std::floor(channelQuantization/2), 1.f));
    count += 1;

    // include the smoothness equations
    for(std::size_t k=0; k<channelQuantization-2; ++k)
    {
      float w = weight.getValue(k+1, channel);

      tripletList.push_back(T(count, k, lambda * w));
      tripletList.push_back(T(count, k+1, -2.f * lambda * w));
      tripletList.push_back(T(count, k+2, lambda * w));

      count += 1;
    }

    sMat A(count, channelQuantization + samplesPerImage*nbGroups);
    A.setFromTriplets(tripletList.begin(), tripletList.end());

    b.conservativeResize(count);

    // solve the system using SVD decomposition
    A.makeCompressed();
    Eigen::SparseQR<sMat, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);
    if(solver.info() != Eigen::Success)  return; // decomposition failed
    Vec x = solver.solve(b);
    if(solver.info() != Eigen::Success)  return; // solving failed

    ALICEVISION_LOG_TRACE("system solved");

    double relative_error = (A*x - b).norm() / b.norm();
    ALICEVISION_LOG_DEBUG("relative error is : " << relative_error);

    for(std::size_t k=0; k<channelQuantization; ++k)
      response.setValue(k, channel, x(k));

  }

}

} // namespace hdr
} // namespace aliceVision
