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
   * @param targetTime
   * @param response
   */
  void process(const std::vector< image::Image<image::RGBfColor> > &images,
                const std::vector<float> &times,
                const rgbCurve &weight,
                const rgbCurve &response,
                image::Image<image::RGBfColor> &radiance,
                float targetTime,
                bool robCalibrate = false,
                float threshold = 1.f);
  
  /**
   * @brief This function obtains the "average scene luminance" EV value
   * from an image file.
   *
   * "average scene luminance" is the L (aka B) value mentioned in [1]
   * We return the log2f value to get an EV value.
   * We are using K=12.07488f and the exif-implied value of N=1/3.125 (see [1]).
   * K=12.07488f is the 1.0592f * 11.4f value in pfscalibration's
   * pfshdrcalibrate.cpp file.
   * Based on [3] we can say that the value can also be 12.5 or even 14.
   * Another reference for APEX is [4] where N is 0.3, closer to the APEX
   * specification of 2^(-7/4)=0.2973.
   *
   * [1] http://en.wikipedia.org/wiki/APEX_system
   * [2] http://en.wikipedia.org/wiki/Exposure_value
   * [3] http://en.wikipedia.org/wiki/Light_meter
   * [4] http://doug.kerr.home.att.net/pumpkin/#APEX
   *
   * This function tries first to obtain the shutter speed from either of
   * two exif tags (there is no standard between camera manifacturers):
   * ExposureTime or ShutterSpeedValue.
   * Same thing for f-number: it can be found in FNumber or in ApertureValue.
   *
   * F-number and shutter speed are mandatory in exif data for EV
   * calculation, iso is not.
   *
   * Function from Luminance HDR
   *
   * @param shutter
   * @param iso
   * @param aperture
   * @return "average scene luminance"
   */
  static float getExposure(float shutter, float iso, float aperture)
  {
    //reflected-light meter calibration constant
    const float K = 12.07488f;
    return std::log2((shutter * iso) / (aperture * aperture * K));

    //EV = log2 (pow2(fstop) / shutter time)
    //LV = LV = EV + log2 (ISO / 100) (LV light Value as exposure)
    //return std::log2( ((aperture * aperture)/shutter) * (iso / 100) );
  }
};

} // namespace hdr
} // namespace aliceVision
