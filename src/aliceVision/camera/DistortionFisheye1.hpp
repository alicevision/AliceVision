// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>

namespace aliceVision {
namespace camera {

class DistortionFisheye1 : public Distortion
{
public:

  DistortionFisheye1()
  {
    _distortionParams = {0.0};
  }

  explicit DistortionFisheye1(double p1)
  {
    _distortionParams = {p1};
  }

  DistortionFisheye1* clone() const override { return new DistortionFisheye1(*this); }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  Vec2 addDistortion(const Vec2 & p) const override
  {
    const double k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;
    return  p * coef;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  Vec2 removeDistortion(const Vec2& p) const override {
    const double k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = 0.5 * std::tan(r * k1) / (std::tan(0.5 * k1) * r);
    return  p * coef;
  }

  ~DistortionFisheye1() override  = default;
};

} // namespace camera
} // namespace aliceVision
