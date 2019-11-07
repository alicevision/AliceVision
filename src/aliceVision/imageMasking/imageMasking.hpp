// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision{

  namespace image{
    template <class> class Image;
  }

namespace imageMasking{

using InImagePath = std::string;
using OutImage = image::Image<unsigned char>;

/**
 * @brief Create a mask based on the hue of each pixel.
 *
 * @param[in] result The resulting binary mask.
 * @param[in] inputPath The path of the image to mask.
 * @param[in] hue Hue value to isolate in [0,1] range. 0 = red, 0.33 = green, 0.66 = blue, 1 = red.
 * @param[in] hueRange Tolerance around the hue value to isolate..
 * @param[in] minSaturation Hue is meaningless if saturation is low. Do not mask pixels below this threshold.
 * @param[in] minValue Hue is meaningless if value is low. Do not mask pixels below this threshold.
 */
void hsv(OutImage& result, const InImagePath& inputPath, float hue, float hueRange, float minSaturation, float minValue);

void postprocess_closing(OutImage& result, int iterations);

}//namespace imageMasking
}//namespace aliceVision
