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

  IntrinsicsScaleOffset(unsigned int w, unsigned int h, double scaleX, double scaleY, double offsetX, double offsetY)
    : IntrinsicBase(w, h)
    , _scale(scaleX, scaleY)
    , _offset(offsetX, offsetY)
  {}

  ~IntrinsicsScaleOffset() override = default;

  void copyFrom(const IntrinsicsScaleOffset& other)
  {
    *this = other;
  }

  void setScale(double x, double y)
  {
    _scale(0) = x;
    _scale(1) = y;
  }

  inline Vec2 getScale() const { return _scale; }

  void setOffset(double offset_x, double offset_y)
  {
    _offset(0) = offset_x;
    _offset(1) = offset_y;
  }

  inline Vec2 getOffset() const 
  { 
    return _offset; 
  }

  // Transform a point from the camera plane to the image plane
  Vec2 cam2ima(const Vec2& p) const override
  {
    return p.cwiseProduct(_scale) + _offset;
  }

  virtual Eigen::Matrix2d getDerivativeCam2ImaWrtScale(const Vec2& p) const
  {
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = p(0);
    M(1, 1) = p(1);
      
    return M;
  }

  virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPoint() const
  {
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = _scale(0);
    M(1, 1) = _scale(1);

    return M;
  }

  virtual Eigen::Matrix2d getDerivativeCam2ImaWrtPrincipalPoint() const
  {
    return Eigen::Matrix2d::Identity();
  }

  // Transform a point from the image plane to the camera plane
  Vec2 ima2cam(const Vec2& p) const override
  {
    Vec2 np;

    np(0) = (p(0) - _offset(0)) / _scale(0);
    np(1) = (p(1) - _offset(1)) / _scale(1);

    return np;
  }

  virtual Eigen::Matrix<double, 2, 2> getDerivativeIma2CamWrtScale(const Vec2& p) const
  {
      Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

      M(0, 0) = -(p(0) - _offset(0)) / (_scale(0) * _scale(0));
      M(1, 1) = -(p(1) - _offset(1)) / (_scale(1) * _scale(1));

      return M;
  }

  virtual Eigen::Matrix2d getDerivativeIma2CamWrtPoint() const
  {
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = 1.0 / _scale(0);
    M(1, 1) = 1.0 / _scale(1);

    return M;
  }

  virtual Eigen::Matrix2d getDerivativeIma2CamWrtPrincipalPoint() const
  {
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = - 1.0 / _scale(0);
    M(1, 1) = - 1.0 / _scale(1);

    return M;
  }

  /**
   * @brief Rescale intrinsics to reflect a rescale of the camera image
   * @param factor a scale factor
   */
  void rescale(float factor) override
  {
    IntrinsicBase::rescale(factor);

    _scale *= factor;
    _offset *= factor;
  }

  // Data wrapper for non linear optimization (update from data)
  bool updateFromParams(const std::vector<double>& params) override
  {
    if (params.size() != 4)
    {
      return false;
    }    

    _scale(0) = params[0];
    _scale(1) = params[1];
    _offset(0) = params[2];
    _offset(1) = params[3];

    return true;
  }

  /**
   * @brief Set initial Scale (for constraining minimization)
   */
  inline void setInitialScale(double initialScale)
  {
    _initialScale = initialScale;
  }

  /**
   * @brief Get the intrinsic initial scale
   * @return The intrinsic initial scale
   */
  inline double initialScale() const
  {
    return _initialScale;
  }

protected:
  Vec2 _scale{1.0, 1.0};
  Vec2 _offset{0.0, 0.0};
  double _initialScale{-1};
};

} // namespace camera
} // namespace aliceVision
