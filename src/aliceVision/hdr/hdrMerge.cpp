// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "hdrMerge.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>

#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/Logger.hpp>


namespace aliceVision {
namespace hdr {

/**
 * f(x)=min + (max-min) * \frac{1}{1 + e^{10 * (x - mid) / width}}
 * https://www.desmos.com/calculator/xamvguu8zw
 *              ____
 * sigmoid:         \________
 *                sigMid
 */
inline float sigmoid(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((xval - sigMid) / sigwidth))));
}

/**
 * https://www.desmos.com/calculator/cvu8s3rlvy
 *
 *                       ____
 * sigmoid inv:  _______/
 *                    sigMid
 */
inline float sigmoidInv(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((sigMid - xval) / sigwidth))));
}

void hdrMerge::process(const std::vector< image::Image<image::RGBfColor> > &images,
                              const std::vector<float> &times,
                              const rgbCurve &weight,
                              const rgbCurve &response,
                              image::Image<image::RGBfColor> &radiance,
                              float targetTime,
                              bool robCalibrate,
                              float clampedValueCorrection)
{
  //checks
  assert(!response.isEmpty());
  assert(!images.empty());
  assert(images.size() == times.size());

  // get images width, height
  const std::size_t width = images.front().Width();
  const std::size_t height = images.front().Height();

  // resize and reset radiance image to 0.0
  radiance.resize(width, height, true, image::RGBfColor(0.f, 0.f, 0.f));

  ALICEVISION_LOG_TRACE("[hdrMerge] Images to fuse:");
  for(int i = 0; i < images.size(); ++i)
  {
    ALICEVISION_LOG_TRACE(images[i].Width() << "x" << images[i].Height() << ", time: " << times[i]);
  }

  const float maxLum = 1000.0f;

  rgbCurve weightShortestExposure = weight;
  weightShortestExposure.freezeSecondPartValues(); // invertAndScaleSecondPart(clampedValueCorrection * maxLum);

  #pragma omp parallel for
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      //for each pixels
      image::RGBfColor &radianceColor = radiance(y, x);

      double isPixelClamped = 0.0;

      for(std::size_t channel = 0; channel < 3; ++channel)
      {
        double wsum = 0.0;
        double wdiv = 0.0;

        {
            int exposureIndex = 0;

            // for each image
            const double value = images[exposureIndex](y, x)(channel);
            const double time = times[exposureIndex];

            // https://www.desmos.com/calculator/xamvguu8zw
            //                       ____
            // sigmoid inv:  _______/
            //                  0    1
            const float isChannelClamped = sigmoidInv(0.0f, 1.0f, /*sigWidth=*/0.2f,  /*sigMid=*/0.9f, value);
            isPixelClamped += isChannelClamped;

            //
            // weightShortestExposure:          _______
            //                          _______/
            //                                0      1
            double w = std::max(0.f, weightShortestExposure(value, channel));

            const double r = response(value, channel);

            wsum += w * r / time;
            wdiv += w;
        }
        for(std::size_t i = 1; i < images.size(); ++i)
        {
          // for each image
          const double value = images[i](y, x)(channel);
          const double time = times[i];
          //
          // weight:          ____
          //          _______/    \________
          //                0      1
          double w = std::max(0.f, weight(value, channel));

          const double r = response(value, channel);
          wsum += w * r / time;
          wdiv += w;
        }
        //{
        //    int exposureIndex = images.size() - 1;
        //    double lowValue = images[exposureIndex](y, x)(channel);
        //    // https://www.desmos.com/calculator/cvu8s3rlvy
        //    //              ____
        //    // sigmoid:         \________
        //    //                  0    1
        //    double clampedLowValue = sigmoid(0.0f, 1.0f, /*sigWidth=*/0.01f, /*sigMid=*/0.005, lowValue);            
        //}

        radianceColor(channel) = wsum / std::max(0.001, wdiv) * targetTime;
      }
      double clampedValueLuminanceCompensation = isPixelClamped / 3.0;
      double clampedValueLuminanceCompensationInv = (1.0 - clampedValueLuminanceCompensation);
      assert(clampedValueLuminanceCompensation <= 1.0);

      for(std::size_t channel = 0; channel < 3; ++channel)
      {
        radianceColor(channel) = clampedValueLuminanceCompensationInv * radianceColor(channel) +
                                 clampedValueLuminanceCompensation * clampedValueCorrection * maxLum;
      }
    }
  }
}


} // namespace hdr
} // namespace aliceVision
