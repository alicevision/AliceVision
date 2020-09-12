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
                        float targetCameraExposure)
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

  rgbCurve weightShortestExposure = weight;
  weightShortestExposure.freezeSecondPartValues();
  rgbCurve weightLongestExposure = weight;
  weightLongestExposure.freezeFirstPartValues();

  #pragma omp parallel for
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      //for each pixels
      image::RGBfColor &radianceColor = radiance(y, x);

      for(std::size_t channel = 0; channel < 3; ++channel)
      {
        double wsum = 0.0;
        double wdiv = 0.0;

        // Merge shortest exposure
        {
            int exposureIndex = 0;

            // for each image
            const double value = images[exposureIndex](y, x)(channel);
            const double time = times[exposureIndex];
            //
            // weightShortestExposure:          _______
            //                          _______/
            //                                0      1
            double w = std::max(0.001f, weightShortestExposure(value, channel));

            const double r = response(value, channel);

            wsum += w * r / time;
            wdiv += w;
        }
        // Merge intermediate exposures
        for(std::size_t i = 1; i < images.size() - 1; ++i)
        {
          // for each image
          const double value = images[i](y, x)(channel);
          const double time = times[i];
          //
          // weight:          ____
          //          _______/    \________
          //                0      1
          double w = std::max(0.001f, weight(value, channel));

          const double r = response(value, channel);
          wsum += w * r / time;
          wdiv += w;
        }
        // Merge longest exposure
        {
            int exposureIndex = images.size() - 1;

            // for each image
            const double value = images[exposureIndex](y, x)(channel);
            const double time = times[exposureIndex];
            //
            // weightLongestExposure:  ____________
            //                                      \_______
            //                                0      1
            double w = std::max(0.001f, weightLongestExposure(value, channel));

            const double r = response(value, channel);

            wsum += w * r / time;
            wdiv += w;
        }
        radianceColor(channel) = wsum / std::max(0.001, wdiv) * targetCameraExposure;
      }
    }
  }
}

void hdrMerge::postProcessHighlight(const std::vector< image::Image<image::RGBfColor> > &images,
    const std::vector<float> &times,
    const rgbCurve &weight,
    const rgbCurve &response,
    image::Image<image::RGBfColor> &radiance,
    float targetCameraExposure,
    float highlightCorrectionFactor,
    float highlightTargetLux)
{
    //checks
    assert(!response.isEmpty());
    assert(!images.empty());
    assert(images.size() == times.size());

    if (highlightCorrectionFactor == 0.0f)
        return;

    const image::Image<image::RGBfColor>& inputImage = images.front();
    // Target Camera Exposure = 1 for EV-0 (iso=100, shutter=1, fnumber=1) => 2.5 lux
    float highlightTarget = highlightTargetLux * targetCameraExposure * 2.5;

    // get images width, height
    const std::size_t width = inputImage.Width();
    const std::size_t height = inputImage.Height();

    image::Image<float> isPixelClamped(width, height);

#pragma omp parallel for
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            //for each pixels
            image::RGBfColor &radianceColor = radiance(y, x);

            float& isClamped = isPixelClamped(y, x);
            isClamped = 0.0f;

            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                const float value = inputImage(y, x)(channel);

                // https://www.desmos.com/calculator/vpvzmidy1a
                //                       ____
                // sigmoid inv:  _______/
                //                  0    1
                const float isChannelClamped = sigmoidInv(0.0f, 1.0f, /*sigWidth=*/0.08f,  /*sigMid=*/0.95f, value);
                isClamped += isChannelClamped;
            }
            isPixelClamped(y, x) /= 3.0;
        }
    }

    image::Image<float> isPixelClamped_g(width, height);
    image::ImageGaussianFilter(isPixelClamped, 1.0f, isPixelClamped_g, 3, 3);

#pragma omp parallel for
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            image::RGBfColor& radianceColor = radiance(y, x);

            double clampingCompensation = highlightCorrectionFactor * isPixelClamped_g(y, x);
            double clampingCompensationInv = (1.0 - clampingCompensation);
            assert(clampingCompensation <= 1.0);

            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                if(highlightTarget > radianceColor(channel))
                {
                    radianceColor(channel) = float(clampingCompensation * highlightTarget + clampingCompensationInv * radianceColor(channel));
                }
            }
        }
    }
}

} // namespace hdr
} // namespace aliceVision
