// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/all.hpp>
#include "emorCurve.hpp"
#include "rgbCurve.hpp"

namespace aliceVision {
namespace hdr {

class GrossbergCalibrate
{
public:
  GrossbergCalibrate(const unsigned int dimension, const std::size_t channelQuantization = std::pow(2, 10));     // the EMOR model has 2^10 values for each curve


  /**
   * @brief
   * @param[in] groups
   * @param[in] times
   * @param[in] nbPoints (number of samples for calibration)
   * @param[in] weight
   * @param[out] response
   */
  void process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
               const std::vector< std::vector<float> > &times,
               const int nbPoints,
               const rgbCurve &weight,
               rgbCurve &response);

private:
  std::size_t _channelQuantization;
  unsigned int _dimension;
};

} // namespace hdr
} // namespace aliceVision
