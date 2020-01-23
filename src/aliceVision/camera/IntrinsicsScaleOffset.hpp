// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "IntrinsicBase.hpp"

namespace aliceVision {
namespace camera {

/**
 * @brief Class with "focal" (scale) and center offset
 */
class IntrinsicsScaleOffset: public IntrinsicBase
{
public:
  IntrinsicsScaleOffset() = default;

  IntrinsicsScaleOffset(unsigned int w, unsigned int h, double scale_x, double scale_y, double offset_x, double offset_y) :
  IntrinsicBase(w, h), _scale_x(scale_x), _scale_y(scale_y), _offset_x(offset_x), _offset_y(offset_y) {
  }

  inline double focal() const {
    return _scale_x;
  }

  void setScale(double x, double y) {
    _scale_x = x;
    _scale_y = y;
  } 
  
  inline Vec2 principal_point() const {
    return Vec2(_offset_x, _offset_y);
  }

  double getFocalLengthPix() const { 
    return _scale_x; 
  }

  Vec2 getPrincipalPoint() const { 
    return Vec2(_offset_x, _offset_y);
  }

  void setOffset(double offset_x, double offset_y) {
    _offset_x = offset_x;
    _offset_y = offset_y;
  }

  // Transform a point from the camera plane to the image plane
  virtual Vec2 cam2ima(const Vec2& p) const override
  {
    return focal() * p + principal_point();
  }

  // Transform a point from the image plane to the camera plane
  virtual Vec2 ima2cam(const Vec2& p) const override
  {
    return ( p -  principal_point() ) / focal();
  }

  /**
   * @brief Rescale intrinsics to reflect a rescale of the camera image
   * @param factor a scale factor
   */
  virtual void rescale(float factor) override {

    IntrinsicBase::rescale(factor);

    _scale_x *= factor;
    _scale_y *= factor;
    _offset_x *= factor;
    _offset_y *= factor;
  }

  // Data wrapper for non linear optimization (update from data)
  virtual bool updateFromParams(const std::vector<double>& params) override
  {
    if (params.size() != 3) {
      return false;
    }

    _scale_x = params[0];
    _scale_y = params[0];
    _offset_x = params[1];
    _offset_y = params[2];

    return true;
  }

protected:
  double _scale_x = 1.0;
  double _scale_y = 1.0;
  double _offset_x = 0.0;
  double _offset_y = 0.0;
};

} // namespace camera
} // namespace aliceVision
