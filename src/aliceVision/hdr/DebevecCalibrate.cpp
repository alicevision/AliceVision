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
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

#include <OpenImageIO/imagebufalgo.h>


namespace aliceVision {
namespace hdr {

using T = Eigen::Triplet<double>;

bool DebevecCalibrate::process(const std::vector< std::vector<std::string>> & imagePathsGroups,
                               const std::size_t channelQuantization,
                               const std::vector<std::vector<float> > &times,
                               const int nbPoints,
                               const int calibrationDownscale,
                               const bool fisheye,
                               const rgbCurve &weight,
                               const float lambda,
                               rgbCurve &response)
{
  const int nbGroups = imagePathsGroups.size();
  const int nbImages = imagePathsGroups.front().size();
  const int samplesPerImage = nbPoints / (nbGroups*nbImages);

  // Always 3 channels for the input images
  static const std::size_t channelsCount = 3;

  // Initialize response
  response = rgbCurve(channelQuantization);

  // Store intermediate data for all three channels
  Vec b_array[channelsCount];
  std::vector<T> tripletList_array[channelsCount];

  // Initialize intermediate buffers
  for(unsigned int channel=0; channel < channelsCount; ++channel)
  {
    Vec & b = b_array[channel];
    b = Vec::Zero(nbPoints + channelQuantization + 1);
    std::vector<T> & tripletList = tripletList_array[channel];
    tripletList.reserve(2 * nbPoints + 1 + 3 * channelQuantization);
  }

  size_t count = 0;
  for (unsigned int g = 0; g < nbGroups; g++)
  {
    const std::vector<std::string > &imagePaths = imagePathsGroups[g];
    std::vector<image::Image<image::RGBfColor>> ldrImagesGroup(imagePaths.size());
 
    for (int i = 0; i < imagePaths.size(); i++)
    {
      image::readImage(imagePaths[i], ldrImagesGroup[i], image::EImageColorSpace::SRGB);
      if (calibrationDownscale != 1.0f)
      {
          image::Image<image::RGBfColor>& img = ldrImagesGroup[i];
          unsigned int w = img.Width();
          unsigned int h = img.Height();
          unsigned int nw = (unsigned int)(floor(float(w) / calibrationDownscale));
          unsigned int nh = (unsigned int)(floor(float(h) / calibrationDownscale));

          image::Image<image::RGBfColor> rescaled(nw, nh);

          oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::FLOAT);
          oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::FLOAT);
          oiio::ImageBuf bufferOrigin(imageSpecOrigin, img.data());
          oiio::ImageBuf bufferResized(imageSpecResized, rescaled.data());
          oiio::ImageBufAlgo::resample(bufferResized, bufferOrigin);

          const oiio::ImageBuf inBuf(oiio::ImageSpec(w, h, 3, oiio::TypeDesc::FLOAT), img.data());
          oiio::ImageBuf outBuf(oiio::ImageSpec(nw, nh, 3, oiio::TypeDesc::FLOAT), rescaled.data());

          oiio::ImageBufAlgo::resize(outBuf, inBuf);
          img.swap(rescaled);
      }
    }

    const std::vector<float> & ldrTimes = times[g];
    const std::size_t width = ldrImagesGroup.front().Width();
    const std::size_t height = ldrImagesGroup.front().Height();

    // If images are fisheye, we take only pixels inside a disk with a radius of image's minimum side
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
            {
                continue;
            }

            for (int channel = 0; channel < channelsCount; channel++)
            {
              float sample = clamp(image(y, x)(channel), 0.f, 1.f);
              float w_ij = weight(sample, channel);
              std::size_t index = std::round(sample * (channelQuantization - 1));

              tripletList_array[channel].push_back(T(count, index, w_ij));
              tripletList_array[channel].push_back(T(count, channelQuantization + g * samplesPerImage + countValidPixels, -w_ij));

              b_array[channel](count) = w_ij * time;
            }

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
          for (int channel = 0; channel < channelsCount; channel++)
          {
            float sample = clamp(image(step*i)(channel), 0.f, 1.f);
            float w_ij = weight(sample, channel);
            std::size_t index = std::round(sample * (channelQuantization - 1));

            tripletList_array[channel].push_back(T(count, index, w_ij));
            tripletList_array[channel].push_back(T(count, channelQuantization + g*samplesPerImage + i, -w_ij));

            b_array[channel](count) = w_ij * time;
          }

          count += 1;
        }
      }
    }
  }

  // fix the curve by setting its middle value to zero
  for (int channel = 0; channel < channelsCount; channel++)
  {
    tripletList_array[channel].push_back(T(count, std::floor(channelQuantization/2), 1.f));
  }
  count += 1;

  // include the smoothness equations
  for(std::size_t k = 0; k<channelQuantization - 2; k++)
  {
    for (int channel = 0; channel < channelsCount; channel++)
    {
      float w = weight.getValue(k + 1, channel);
      tripletList_array[channel].push_back(T(count, k, lambda * w));
      tripletList_array[channel].push_back(T(count, k + 1, - 2.f * lambda * w));
      tripletList_array[channel].push_back(T(count, k + 2, lambda * w));
    }

    count++;
  }

  for (int channel = 0; channel < channelsCount; channel ++)
  {
    sMat A(count, channelQuantization + samplesPerImage * nbGroups);
    A.setFromTriplets(tripletList_array[channel].begin(), tripletList_array[channel].end());
    b_array[channel].conservativeResize(count);

    // solve the system using SVD decomposition
    A.makeCompressed();
    Eigen::SparseQR<sMat, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);

    // Check solver failure
    if (solver.info() != Eigen::Success)
    {
      return false;
    }

    Vec x = solver.solve(b_array[channel]);

    // Check solver failure
    if(solver.info() != Eigen::Success)
    {
      return false;
    }

    double relative_error = (A*x - b_array[channel]).norm() / b_array[channel].norm();

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
