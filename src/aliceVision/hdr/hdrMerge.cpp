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
                        const std::vector<double> &times,
                        const rgbCurve &weight,
                        const rgbCurve &response,
                        image::Image<image::RGBfColor> &radiance,
                        float targetCameraExposure,
                        int refImageIndex)
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

  const std::string mergeInfoFilename = "C:/Temp/mergeInfo.csv";
  std::ofstream file(mergeInfoFilename);

  std::vector<double> v_minRatio;
  std::vector<double> v_maxRatio;

  // For each exposure except the longuest,
  //   Compute a ratio between the next and the current exposure values.
  //   Deduce a range of ratii that will be used for enabling input data.
  // Duplicate the last range limits to associate it with the longuest exposure
  for (std::size_t i = 0; i < times.size() - 1; i++)
  {
    const double refRatio = times[i + 1] / times[i];
    v_minRatio.push_back(0.25 * refRatio);
    v_maxRatio.push_back(1.75 * refRatio);
  }
  v_minRatio.push_back(v_minRatio.back());
  v_maxRatio.push_back(v_maxRatio.back());

  const double minValue = 0.05;
  const double maxValue = 0.999;

  #pragma omp parallel for
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      //for each pixels
      image::RGBfColor &radianceColor = radiance(y, x);

      //if ((x%1000 == 780) && (y%1000 == 446))
      //{
      //    if(!file)
      //    {
      //        ALICEVISION_LOG_WARNING("Unable to create file " << mergeInfoFilename << " for storing merging info.");
      //    }
      //    else
      //    {
      //        file << x << "," << y << std::endl;
      //        file << "time,R,resp(R),coeff,resp(R)/time,G,resp(G),coeff,resp(G)/time,B,resp(B),coeff,resp(B)/time," << std::endl;
      //        for(std::size_t i = 0; i < images.size(); i++)
      //        {
      //            const double time = times[i];
      //            file << time << ",";
      //            for(std::size_t channel = 0; channel < 3; ++channel)
      //            {
      //                const double value = images[i](y, x)(channel);
      //                const double r = response(value, channel);
      //                double w = std::max(0.001f, (i == 0) ? weightShortestExposure(value, channel)
      //                                                     : (i == (images.size() - 1) ? weightLongestExposure(value, channel)
      //                                                                                 : weight(value, channel)));

      //                file << value << ",";
      //                file << r << ",";
      //                file << w << ",";
      //                file << r/time << ",";
      //            }
      //            file << std::endl;
      //        }

      //    }
      //}

      const double meanValueHighExp = (images[images.size() - 1](y, x)(0) + images[images.size() - 1](y, x)(1) +
                                       images[images.size() - 1](y, x)(2)) /
                                      3.0;

      const double noiseThreshold = 0.1;

      if(meanValueHighExp < noiseThreshold) // Noise case
      {
          for(std::size_t channel = 0; channel < 3; ++channel)
          {
              radianceColor(channel) = targetCameraExposure * response(meanValueHighExp, channel) / times[images.size() - 1];
          }
      }
      else
      {
          std::vector<std::vector<double>> vv_ratio;
          std::vector<std::vector<double>> vv_coeff;
          std::vector<std::vector<double>> vv_coeff_filt;
          std::vector<double> v_sumCoeff;
          std::vector<std::vector<double>> vv_normalizedValue;

          // Per channel, and per exposition compute a mixing coefficient and the ratio between linearized values at the next and the current exposure. 
          // Keep the coeffcient if the computed ratio is in the predefined range and if the linearized input value is significant enough (higher than a threshold).
          // To deal with highlights, keep the shortest exposure if the second one is saturated.

          for(std::size_t channel = 0; channel < 3; ++channel)
          {
              std::vector<double> v_ratio;
              std::vector<double> v_coeff;
              std::vector<double> v_coeff_filt;
              std::vector<double> v_normalizedValue;

              {
                  const double value = response(images[0](y, x)(channel), channel);
                  const double ratio = (value > 0.0) ? response(images[1](y, x)(channel), channel) / value : 0.0;
                  const double normalizedValue = value / times[0];
                  double coeff = std::max(0.001f, weightShortestExposure(images[0](y, x)(channel), channel));

                  const bool coeffOK = (value > minValue && ratio > v_minRatio[0] && ratio < v_maxRatio[0]) ||
                                       response(images[1](y, x)(channel), channel) > maxValue;

                  v_normalizedValue.push_back(normalizedValue);
                  v_ratio.push_back(ratio);
                  v_coeff.push_back(coeff);
                  v_coeff_filt.push_back(coeffOK ? coeff : 0.0);
              }

              for(std::size_t e = 1; e < images.size() - 1; e++)
              {
                  const double value = response(images[e](y, x)(channel), channel);
                  const double normalizedValue = value / times[e];
                  const double ratio = (value > 0.0) ? response(images[e + 1](y, x)(channel), channel) / value : 0.0;
                  double coeff = std::max(0.001f, weight(value, channel));

                  const bool coeffOK = (value > minValue && ratio > v_minRatio[e] && ratio < v_maxRatio[e]);

                  v_normalizedValue.push_back(normalizedValue);
                  v_ratio.push_back(ratio);
                  v_coeff.push_back(coeff);
                  v_coeff_filt.push_back(coeffOK ? coeff : 0.0);
              }

              {
                  const double value = response(images[images.size() - 1](y, x)(channel), channel);
                  const double ratio = v_ratio.back();
                  const double normalizedValue = value / times[images.size() - 1];
                  double coeff = std::max(
                      0.001f,
                      weightLongestExposure(response(images[images.size() - 1](y, x)(channel), channel), channel));

                  const bool coeffOK = (value < maxValue && ratio > v_minRatio[images.size() - 1] &&
                                        ratio < v_maxRatio[images.size() - 1]);

                  v_normalizedValue.push_back(normalizedValue);
                  //v_ratio.push_back(ratio);
                  v_coeff.push_back(coeff);
                  v_coeff_filt.push_back(coeffOK ? coeff : 0.0);
              }

              //vv_ratio.push_back(v_ratio);
              vv_coeff.push_back(v_coeff);
              vv_coeff_filt.push_back(v_coeff_filt);
              vv_normalizedValue.push_back(v_normalizedValue);
          }

          std::vector<std::vector<double>> vv_coeff_final = vv_coeff_filt;
          v_sumCoeff = {0.0, 0.0, 0.0};

          // Per exposure and per channel,
          //   If the coeff has been discarded for the current channel but is valid for at least one of the two other channels, restore it's original value.
          // Per channel, sum the valid coefficients.
          for(std::size_t e = 0; e < images.size(); e++)
          {
              for(std::size_t channel = 0; channel < 3; ++channel)
              {
                  if(vv_coeff_final[channel][e] == 0.0 &&
                     (vv_coeff_filt[(channel + 1) % 3][e] != 0.0 || vv_coeff_filt[(channel + 2) % 3][e] != 0.0))
                  {
                      vv_coeff_final[channel][e] = vv_coeff[channel][e];
                  }
                  v_sumCoeff[channel] += vv_coeff_final[channel][e];
              }
          }

          // Per channel, if the sum of the coefficients is null, restore the coefficient corresponding to the reference image.
          // Due to the previous step, if the sum of the coefficients is null for a given channel it should be also null for the two other channels.
          if(v_sumCoeff[0] == 0.0)
          {
              for(std::size_t channel = 0; channel < 3; ++channel)
              {
                  vv_coeff_final[channel][refImageIndex] = 1.0;
                  v_sumCoeff[channel] = 1.0;
              }
          }

          // Compute the final result and adjust the exposure to the reference one.
          for(std::size_t channel = 0; channel < 3; ++channel)
          {
                double v = 0.0;
                for(std::size_t i = 0; i < images.size(); ++i)
                {
                    v += vv_coeff_final[channel][i] * vv_normalizedValue[channel][i];
                }
                radianceColor(channel) = targetCameraExposure * (v != 0.0 ? v : vv_normalizedValue[channel][refImageIndex]) / v_sumCoeff[channel];
          }
      }
    }
  }
}

void hdrMerge::postProcessHighlight(const std::vector< image::Image<image::RGBfColor> > &images,
    const std::vector<double> &times,
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
