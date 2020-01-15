// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include "rgbCurve.hpp"
#include <aliceVision/image/all.hpp>
#include <cmath>


namespace aliceVision {
namespace hdr {
 
class hdrMerge {
public:

  /**
   * @brief
   * @param images
   * @param radiance
   * @param times
   * @param targetCameraExposure
   * @param response
   */
  void process(const std::vector< image::Image<image::RGBfColor> > &images,
                const std::vector<float> &times,
                const rgbCurve &weight,
                const rgbCurve &response,
                image::Image<image::RGBfColor> &radiance,
                float targetCameraExposure);

  void postProcessHighlight(const std::vector< image::Image<image::RGBfColor> > &images,
      const std::vector<float> &times,
      const rgbCurve &weight,
      const rgbCurve &response,
      image::Image<image::RGBfColor> &radiance,
      float clampedValueCorrection,
      float targetCameraExposure,
      float highlightMaxLumimance);

};

} // namespace hdr
} // namespace aliceVision
