// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/imageMasking/imageMasking.hpp"
#include "aliceVision/imageMasking/eigen2cvHelpers.hpp"

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/io.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace aliceVision{
namespace imageMasking{

namespace{
  float clamp(float value)
  {
    return std::min(1.f, std::max(0.f, value));
  }

  uint8_t remap_float2uint8(float value)
  {
    return uint8_t(clamp(value) * 255.f + 0.49999f);
  }

  cv::Mat wrapCvMask(OutImage& result)
  {
    return cv::Mat(result.rows(), result.cols(), CV_8UC1, result.data(), result.rowStride());
  }
}

void hsv(OutImage& result, const InImagePath& inputPath, float hue, float hueRange, float minSaturation, float minValue)
{
  image::Image<image::RGBColor> input;
  image::readImage(inputPath, input, image::EImageColorSpace::SRGB);

  cv::Mat input_hsv;
  cv::eigen2cv(input.GetMat(), input_hsv);  // copy the buffer, but a copy is needed to convert to HSV colorspace anyway.
  cv::cvtColor(input_hsv, input_hsv, cv::COLOR_RGB2HSV);

  result.resize(input.Width(), input.Height(), false);  // allocate un-initialized
  const cv::Mat result_cv = wrapCvMask(result);

  // OpenCV hue is in [0, 179] range -> remap [0, 1] -> [0, 179]
  const uint8_t lowH = uint8_t(std::max(0.f, hue - hueRange) * 180.f);
  const uint8_t highH = uint8_t(std::min(1.f, hue + hueRange) * 180.f);
  const uint8_t lowS = remap_float2uint8(minSaturation);
  const uint8_t highS = 255;
  const uint8_t lowV = remap_float2uint8(minValue);
  const uint8_t highV = 255;
  cv::inRange(input_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), result_cv);
};

}//namespace imageMasking
}//namespace aliceVision
