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
  

  IntrinsicsScaleOffsetDisto(unsigned int w, unsigned int h, double scale_x, double scale_y, double offset_x, double offset_y, std::shared_ptr<Distortion> distortion = nullptr) :
  IntrinsicsScaleOffset(w, h, scale_x, scale_y, offset_x, offset_y),
  _pDistortion(distortion) {
  }

  virtual bool have_disto() const override {  
    return (!(_pDistortion == nullptr));
  }

  virtual Vec2 add_disto(const Vec2& p) const override { 
    if (_pDistortion == nullptr) {
      return p;
    }

    return _pDistortion->add_disto(p); 
  }
  

  virtual Vec2 remove_disto(const Vec2& p) const override { 
    if (_pDistortion == nullptr) {
      return p;
    }

    return _pDistortion->remove_disto(p); 
  }

  /// Return the un-distorted pixel (with removed distortion)
  virtual Vec2 get_ud_pixel(const Vec2& p) const override {
    return cam2ima(remove_disto(ima2cam(p)));
  }

  /// Return the distorted pixel (with added distortion)
  virtual Vec2 get_d_pixel(const Vec2& p) const override {
    return cam2ima(add_disto(ima2cam(p)));
  }

  std::vector<double> getDistortionParams() const
  {
    if (!have_disto()) {
      return std::vector<double>();
    }

    return _pDistortion->getParameters();
  }

  // Data wrapper for non linear optimization (get data)
  std::vector<double> getParams() const override
  {
    std::vector<double> params = {_scale_x, _offset_x, _offset_y};

    if (have_disto()) {

      params.insert(params.end(), _pDistortion->getParameters().begin(), _pDistortion->getParameters().end());
    }
    
    return params;
  }

  void setDistortionParams(const std::vector<double> & distortionParams) {

    int expected = 0;
    if (_pDistortion != nullptr) {
      expected = _pDistortion->getDistortionParametersCount();
    }

    if (distortionParams.size() != expected)
    {
        std::stringstream s;
        s << "IntrinsicsScaleOffsetDisto::setDistortionParams: wrong number of distortion parameters (expected: " << expected << ", given:" << distortionParams.size() << ").";
        throw std::runtime_error(s.str());
    }

    if (_pDistortion) {
      _pDistortion->getParameters() = distortionParams;
    }
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double>& params) override
  {
    if (_pDistortion == nullptr) {
      if (params.size() != 3) {
        return false;
      }
    }
    else {
      if (params.size() != (3 + _pDistortion->getDistortionParametersCount())) {
        return false;
      }
    }

    _scale_x = params[0];
    _scale_y = params[0];
    _offset_x = params[1];
    _offset_y = params[2];


    setDistortionParams({params.begin() + 3, params.end()});

    return true;
  }

  virtual float getMaximalDistortion(double min_radius, double max_radius) const override {

    if (_pDistortion == nullptr) {
      return max_radius;
    }

    return _pDistortion->getUndistortedRadius(max_radius);
  }

protected:
  std::shared_ptr<Distortion> _pDistortion;
};

} // namespace camera
} // namespace aliceVision
