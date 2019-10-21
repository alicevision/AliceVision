// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/imageMasking/imageMasking.hpp"

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/io.hpp"

namespace aliceVision{
namespace imageMasking{

void hsv(OutImage& result, const InImagePath& inputPath, float hue, float hueRange, float minSaturation, float minValue)
{
  image::Image<image::RGBColor> input;
  image::readImage(inputPath, input, image::EImageColorSpace::AUTO);

  result.resize(input.Width(), input.Height(), true, 127);
};

}//namespace imageMasking
}//namespace aliceVision
