// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "sampling.hpp"
#include "rgbCurve.hpp"

#include <vector>
#include <string>


namespace aliceVision {
namespace hdr {

/**
 * @brief Calibration of the Camera Response Function (CRF) from multiple LDR images.
 *
 * The implementation is based on the following paper:
 * P. Debevec and J. Malik. Recovering high dynamic range radiance maps from photographs.
 * In Proceedings of the 24th Annual Conference on Computer Graphics and Interactive Techniques,
 * SIGGRAPH 97, pages 369-378, 1997.
 *
 * https://dl.acm.org/doi/pdf/10.1145/1401132.1401174
 */
class DebevecCalibrate
{
public:  

  /**
   * @brief
   * @param[in] LDR images groups
   * @param[in] exposure times
   * @param[in] channel quantization
   * @param[in] calibration weight function
   * @param[in] lambda (parameter of smoothness)
   * @param[out] camera response function
   */
  bool process(const std::vector<std::vector<ImageSample>> & ldrSamples,
               const std::vector<std::vector<float>> &times,
               std::size_t channelQuantization,
               const rgbCurve &weight,
               float lambda,
               rgbCurve &response);

};

} // namespace hdr
} // namespace aliceVision
