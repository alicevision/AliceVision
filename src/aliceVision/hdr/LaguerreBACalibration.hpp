// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "sampling.hpp"
#include "emorCurve.hpp"
#include "rgbCurve.hpp"

#include <boost/math/constants/constants.hpp>


namespace aliceVision {
namespace hdr {

/**
 * @brief Calibration of the Camera Response Function (CRF) from multiple LDR images.
 *
 * The implementation is based on the following paper:
 * "Mapping Colour in Image Stitching Applications", David Hasler, Sabine Susstrunk, 2003, JVCIR-2004
 * https://infoscience.epfl.ch/record/50201/files/hs03_JVCIR.pdf
 *
 * Itself based on:
 * "Radiometric Self Calibration", Tomoo Mitsunaga, Shree K. Nayar, CVPR-1999
 * http://www.cs.columbia.edu/CAVE/publications/pdfs/Mitsunaga_CVPR99.pdf
 *
 * Some precisions are also provided in:
 * "Radiometric alignment and vignetting calibration", Pablo d'Angelo, ICVS 2007
 * http://hugin.sourceforge.net/tech/icvs2007_final.pdf
 */
class LaguerreBACalibration
{
public:
  explicit LaguerreBACalibration() = default;

  /**
   * @brief
   * @param[in] LDR images groups
   * @param[in] channel quantization
   * @param[in] exposure times
   * @param[in] calibration weight function
   * @param[out] camera response function
   */
  void process(
      const std::vector<std::vector<ImageSample>>& ldrSamples,
      std::vector<std::vector<float>>& cameraExposures,
      const std::size_t channelQuantization,
      bool refineExposures,
      rgbCurve &response);
};

template <typename T>
T laguerreFunction(const T& a, const T& x)
{
    // https://www.desmos.com/calculator/ib1y06t4pe
    using namespace boost::math::constants;
    constexpr double c = 2.0 / pi<double>();
    return x + c * atan((a * sin(pi<double>() * x)) / (1.0 - a * cos(pi<double>() * x)));
}

template <typename T>
T laguerreFunctionInv(const T& a, const T& x)
{
    return laguerreFunction(-a, x);
}


} // namespace hdr
} // namespace aliceVision
