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
  explicit GrossbergCalibrate(const unsigned int dimension);


  /**
   * @brief
   * @param[in] LDR images groups
   * @param[in] channel quantization
   * @param[in] exposure times
   * @param[in] number of samples
   * @param[in] calibration weight function
   * @param[out] camera response function
   */
  void process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
               const std::size_t channelQuantization,
               const std::vector< std::vector<float> > &times,
               const int nbPoints,
               const rgbCurve &weight,
               rgbCurve &response);

private:
  /// Dimension of the response ie number of basis vectors to calculate the response function
  unsigned int _dimension;
};

} // namespace hdr
} // namespace aliceVision
