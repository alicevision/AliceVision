// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicsScaleOffset.hpp"
#include "Distortion.hpp"
#include <memory>

namespace aliceVision {
namespace camera {

/**
 * @brief Class with disto
 */
class IntrinsicsScaleOffsetDisto : public IntrinsicsScaleOffset
{
public:
  IntrinsicsScaleOffsetDisto() = default;
  

  IntrinsicsScaleOffsetDisto(unsigned int w, unsigned int h, double scale_x, double scale_y, double offset_x, double offset_y) :
  IntrinsicsScaleOffset(w, h, scale_x, scale_y, offset_x, offset_y) {
  }

protected:
  std::shared_ptr<Distortion> _pDistortion;
};

} // namespace camera
} // namespace aliceVision
