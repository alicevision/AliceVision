// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "sampling.hpp"
#include "emorCurve.hpp"
#include "rgbCurve.hpp"


namespace aliceVision {
namespace hdr {

/**
 * @brief Calibration of the Camera Response Function (CRF) from multiple LDR images.
 *
 * The implementation is based on the following papers:
 *
 * M. D. Grossberg and S. K. Nayar. Modeling the space of camera response functions.
 * Society Conference on Computer Vision and Pattern Recognition, 2003. Proceedings. (Vol. 2, pp. II-602). IEEE.
 * https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.177.7118&rep=rep1&type=pdf
 *
 * M. D. Grossberg and S. K. Nayar. Modeling the space of camera response functions.
 * IEEE Transactions on Pattern Analysis and Machine Intelligence, 26(10) 1272-1282, 2004.
 * https://www.researchgate.net/profile/Michael_Grossberg/publication/3193780_Modeling_the_space_of_camera_response_functions/links/0deec5214cc89adc87000000/Modeling-the-space-of-camera-response-functions.pdf
 */
class GrossbergCalibrate
{
public:
  explicit GrossbergCalibrate(unsigned int dimension);


  /**
   * @brief
   * @param[in] LDR images groups
   * @param[in] exposure times
   * @param[in] channel quantization
   * @param[out] camera response function
   */
  void process(const std::vector<std::vector<ImageSample>>& ldrSamples,
               const std::vector< std::vector<float> > &times,
               std::size_t channelQuantization,
               rgbCurve &response);

private:
  /// Dimension of the response ie number of basis vectors to calculate the response function
  unsigned int _dimension;
};

} // namespace hdr
} // namespace aliceVision
