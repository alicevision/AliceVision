// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RobertsonCalibrate.hpp"
#include <iostream>
#include <fstream>
#include <cassert>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace hdr {

void RobertsonCalibrate::process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
                                 const std::size_t channelQuantization,
                                 const std::vector< std::vector<float> > &times,
                                 const int nbPoints,
                                 const bool fisheye,
                                 const rgbCurve &weight,
                                 rgbCurve &response)
{
  const int nbGroups = ldrImageGroups.size();
  const int nbImages = ldrImageGroups.front().size();
  const int samplesPerImage = nbPoints / (nbGroups*nbImages);

  //set channels count always RGB
  static const std::size_t channels = 3;

  //create radiance vector of image
  _radiance = std::vector< image::Image<image::RGBfColor> >(nbGroups);

  //initialize response
  response = rgbCurve(channelQuantization);
  response.setLinear();
  response.normalize();

  //initialize cardinality
  rgbCurve card(channelQuantization);
  card.setZero();

  //compute cardinal curve
  for(unsigned int g = 0; g < nbGroups; ++g)
  {
    const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups[g];
    const std::size_t width = ldrImagesGroup.front().Width();
    const std::size_t height = ldrImagesGroup.front().Height();

    _radiance[g].resize(width, height, false);

    // if images are fisheye, we take only pixels inside a disk with a radius of image's minimum side
    if(fisheye)
    {
      const std::size_t minSize = std::min(width, height) * 0.97;
      const Vec2i center(width/2, height/2);
      const std::size_t maxDist2 = pow(minSize * 0.5, 2);

      const int xMin = std::ceil(center(0) - minSize/2);
      const int yMin = std::ceil(center(1) - minSize/2);
      const int xMax = std::floor(center(0) + minSize/2);
      const int yMax = std::floor(center(1) + minSize/2);

      const int step = std::ceil(minSize / sqrt(samplesPerImage));

      for(unsigned int j=0; j<nbImages; ++j)
      {
        const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
        for(int y = yMin; y <= yMax-step; y+=step)
        {
          for(int x = xMin; x <= xMax-step; x+=step)
          {
            std::size_t dist2 = pow(center(0)-x, 2) + pow(center(1)-y, 2);
            if(dist2 > maxDist2)
              continue;

            const image::RGBfColor &pixelValue = image(y, x);

            for(std::size_t channel = 0; channel < channels; ++channel)
            {
              //number of pixel with the same value
              std::size_t index = std::round(clamp(pixelValue(channel), 0.f, 1.f) * (channelQuantization - 1));
              card.getValue(index, channel) += 1;
            }
          }
        }
      }
    }
    else
    {
      const int step = std::floor(width * height / samplesPerImage);
      for(unsigned int j=0; j<nbImages; ++j)
      {
        const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);

        for(std::size_t i=0; i<nbPoints; ++i)
        {
            const image::RGBfColor &pixelValue = image(step*i);

            for(std::size_t channel = 0; channel < channels; ++channel)
            {
              //number of pixel with the same value
              std::size_t index = std::round(clamp(pixelValue(channel), 0.f, 1.f) * (channelQuantization - 1));
              card.getValue(index, channel) += 1;
            }
        }
      }
    }
  }
  card.interpolateMissingValues();

  //inverse cardinal curve value (for optimized division in the loop)
  card.inverseAllValues();

  //create merge operator
  hdrMerge merge;

  for(std::size_t iter = 0; iter < _maxIteration; ++iter)
  {
    ALICEVISION_LOG_TRACE("--> iteration : "<< iter);

    ALICEVISION_LOG_TRACE("1) compute radiance ");
    //initialize radiance
    for(std::size_t g = 0; g < nbGroups; ++g)
    {
      merge.process(ldrImageGroups[g], times[g], weight, response, _radiance[g], 1.f);
    }

    ALICEVISION_LOG_TRACE("2) initialization new response ");
    //initialize new response
    rgbCurve newResponse = rgbCurve(channelQuantization);
    newResponse.setZero();

    ALICEVISION_LOG_TRACE("3) compute new response ");
    //compute new response
    for(unsigned int g = 0; g < nbGroups; ++g)
    {
      const std::vector< image::Image<image::RGBfColor> > &ldrImagesGroup = ldrImageGroups[g];
      const std::vector<float> &ldrTimes = times[g];
      const int nbImages = ldrImagesGroup.size();
      const std::size_t width = ldrImagesGroup.front().Width();
      const std::size_t height = ldrImagesGroup.front().Height();

      const image::Image<image::RGBfColor> &radiance = _radiance[g];

      if(fisheye)
      {
        const std::size_t minSize = std::min(width, height) * 0.97;
        const Vec2i center(width/2, height/2);
        const std::size_t maxDist2 = pow(minSize * 0.5, 2);

        const int xMin = std::ceil(center(0) - minSize/2);
        const int yMin = std::ceil(center(1) - minSize/2);
        const int xMax = std::floor(center(0) + minSize/2);
        const int yMax = std::floor(center(1) + minSize/2);

        const int step = std::ceil(minSize / sqrt(samplesPerImage));

        for(unsigned int j=0; j<nbImages; ++j)
        {
          const image::Image<image::RGBfColor> &image = ldrImagesGroup.at(j);
          #pragma omp parallel for
          for(int y = yMin; y <= yMax-step; y+=step)
          {
            for(int x = xMin; x <= xMax-step; x+=step)
            {
              std::size_t dist2 = pow(center(0)-x, 2) + pow(center(1)-y, 2);
              if(dist2 > maxDist2)
                continue;

              const image::RGBfColor &pixelValue = image(y, x);
              const image::RGBfColor &radianceValue = radiance(y, x);

              for(std::size_t channel = 0; channel < channels; ++channel)
              {
                  std::size_t index = std::round(clamp(pixelValue(channel), 0.f, 1.f) * (channelQuantization - 1));
                  newResponse.getValue(index, channel) += ldrTimes.at(j) * (radianceValue(channel));
              }
            }
          }
        }
      }
      else
      {
        const int step = std::floor(width * height / samplesPerImage);
        for(unsigned int j=0; j<nbImages; ++j)
        {
          #pragma omp parallel for
          for(int i=0; i<nbPoints; ++i)
          {
            const image::RGBfColor &pixelValue = ldrImagesGroup.at(j)(step*i);
            const image::RGBfColor &radianceValue = radiance(step*i);

            for(std::size_t channel = 0; channel < channels; ++channel)
            {
                std::size_t index = std::round(clamp(pixelValue(channel), 0.f, 1.f) * (channelQuantization - 1));
                newResponse.getValue(index, channel) += ldrTimes.at(j) * (radianceValue(channel));
            }
          }
        }
      }
    }
    newResponse.interpolateMissingValues();
    //dividing the response by the cardinal curve
    newResponse *= card;

    ALICEVISION_LOG_TRACE("4) normalize response");
    //normalization
    newResponse.normalize();

    ALICEVISION_LOG_TRACE("5) compute difference");
    //calculate difference between the old response and the new one
    rgbCurve responseDiff = newResponse - response;
    responseDiff.setAllAbsolute();

    double diff = rgbCurve::sumAll(responseDiff) / channels;

    //update the response
    response = newResponse;

    ALICEVISION_LOG_TRACE("6) check end condition");
    //check end condition
    if(diff < _threshold)
    {
        ALICEVISION_LOG_ERROR("[BREAK] difference < threshold "" << std::endl");
        break;
    }
    ALICEVISION_LOG_DEBUG("-> difference is " << diff);
  }
}

} // namespace hdr
} // namespace aliceVision
