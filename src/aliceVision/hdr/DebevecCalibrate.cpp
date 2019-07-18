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
  //checks
  for (int g = 0; g < ldrImageGroups.size(); ++g)
  {
    assert(ldrImageGroups[g].size() == times[g].size());
  }

  //set channels count always RGB
  static const std::size_t channels = 3;

  //initialize response
  response = rgbCurve(channelQuantization);
  response.setLinear();
  response.normalize();

  for(unsigned int g=0; g<ldrImageGroups.size(); ++g)
  {
    const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups.at(g);
    const std::vector<float> &ldrTimes = times.at(g);
    const int nbImages = ldrImagesGroup.size();
    const std::size_t width = ldrImagesGroup.front().Width();
    const std::size_t height = ldrImagesGroup.front().Height();

    const std::size_t minSize = std::min(width, height) * 0.97;
    const Vec2i center(width/2, height/2);
    const int xMin = std::ceil(center(0) - minSize/2);
    const int yMin = std::ceil(center(1) - minSize/2);
    const int xMax = std::floor(center(0) + minSize/2);
    const int yMax = std::floor(center(1) + minSize/2);
    const std::size_t maxDist2 = pow(minSize * 0.5, 2);

    for(unsigned int channel=0; channel<channels; ++channel)
    {
      Vec b = Vec::Zero(nbPoints*nbImages + channelQuantization + 1);
      int count = 0;
      int nbValidPixels = 0;

      std::vector<T> tripletList;
      tripletList.reserve(2 * nbPoints*nbImages + 1 + 3 * channelQuantization);

      ALICEVISION_LOG_TRACE("filling A and b matrices");

      // include the data-fitting equations
      // if images are fisheye, we take only pixels inside a disk with a radius of image's minimum side
      if(fisheye)
      {
        const int step = std::ceil(sqrt(std::ceil(minSize*minSize / nbPoints)));
        for(unsigned int j=0; j<nbImages; ++j)
        {
          const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
          int iter = 0;
          for(int y = yMin; y <= yMax-step; y+=step)
          {
            for(int x = xMin; x <= xMax-step; x+=step)
            {
              std::size_t dist2 = pow(center(0)-x, 2) + pow(center(1)-y, 2);
              if(dist2 > maxDist2)
                continue;

              float sample = clamp(image(y, x)(channel), 0.f, 1.f);
              float w_ij = weight(sample, channel);
              std::size_t index = std::round(sample * (channelQuantization - 1));

              tripletList.push_back(T(count, index, w_ij));
              tripletList.push_back(T(count, channelQuantization + iter, -w_ij));

              b(count) = w_ij * std::log(ldrTimes.at(j));
              count += 1;
              iter +=1;
            }
          }
          if(j == 0)
            nbValidPixels = iter;
        }
      }
      else
      {
        const int step = std::floor(width*height / nbPoints);
        for(unsigned int j=0; j<nbImages; ++j)
        {
          const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
          for(unsigned int i=0; i<nbPoints; ++i)
          {
            float sample = clamp(image(step*i)(channel), 0.f, 1.f);
            float w_ij = weight(sample, channel);
            std::size_t index = std::round(sample * (channelQuantization - 1));

            tripletList.push_back(T(count, index, w_ij));
            tripletList.push_back(T(count, channelQuantization+i, -w_ij));

            b(count) = w_ij * std::log(ldrTimes.at(j));
            count += 1;
          }
        }
        nbValidPixels = nbPoints;
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

      sMat A(count, channelQuantization + nbValidPixels);
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
}

} // namespace hdr
} // namespace aliceVision
