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
   * @brief
   * @param[in] maxIter
   * @param[in] threshold
   */
  RobertsonCalibrate(std::size_t maxIter = 500, double threshold = 0.01f) :
    _maxIteration(maxIter),
    _threshold(threshold)
  {}

  /**
   * @brief
   * @param[in] groups
   * @param[out] response
   * @param[in] times
   */
  void process(const std::vector< std::vector< image::Image<image::RGBfColor> > > &ldrImageGroups,
               const std::size_t channelQuantization,
               const std::vector< std::vector<float> > &times, const int nbPoints,
               rgbCurve &weight,
               rgbCurve &response,
               float targetTime);


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
  std::vector< image::Image<image::RGBfColor> > _radiance;
  double _threshold;
  std::size_t _maxIteration;
};

} // namespace hdr
} // namespace aliceVision
