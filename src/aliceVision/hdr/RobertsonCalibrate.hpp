// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include <aliceVision/image/all.hpp>
#include "rgbCurve.hpp"
#include "hdrMerge.hpp"


namespace aliceVision {
namespace hdr {

class RobertsonCalibrate {
public:
    
  /**
   * @brief RobertsonCalibrate constructor
   * @param[in] maxIter
   * @param[in] threshold
   */
  RobertsonCalibrate(std::size_t maxIter = 500, double threshold = 0.01f) :
    _maxIteration(maxIter),
    _threshold(threshold)
  {}

  /**
   * @brief Calculate the camera response function according to Robertson method
   * @param[in] LDR images groups
   * @param[in] channel quantization
   * @param[in] exposure times
   * @param[in] number of samples
   * @param[in] calibration weight function
   * @param[in] target exposure time
   * @param[out] camera response function
   */
  void process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
               std::size_t channelQuantization,
               const std::vector< std::vector<float> > &times,
               int nbPoints,
               const rgbCurve &weight,
               float targetTime,
               rgbCurve &response);


  int getMaxIteration() const
  {
    return _maxIteration;
  }

  float getThreshold() const
  {
    return _threshold;
  }

  void setMaxIteration(int value)
  {
    _maxIteration = value;
  }

  void setThreshold(float value)
  {
    _threshold = value;
  }

  const image::Image<image::RGBfColor>& getRadiance(std::size_t group) const
  {
    assert(group < _radiance.size());

    return _radiance[group];
  }

private:
  /// Vector containing the HDR images for each group
  std::vector< image::Image<image::RGBfColor> > _radiance;
  /// If the difference between responses is below the threshold we stop the iteration
  double _threshold;
  /// Maximum number of iteration before stopping the program
  std::size_t _maxIteration;
};

} // namespace hdr
} // namespace aliceVision
