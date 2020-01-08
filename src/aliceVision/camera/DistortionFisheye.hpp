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

class DistortionFisheye : public Distortion {
public:
  DistortionFisheye() {
    _distortionParams = {0.0, 0.0, 0.0, 0.0};
  }

  DistortionFisheye(double p1, double p2, double p3, double p4) {
    _distortionParams = {p1, p2, p3, p4};
  }

  /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
  virtual Vec2 add_disto(const Vec2 & p) const override
  {
    const double eps = 1e-8;
    const double k1 = _distortionParams.at(0), k2 = _distortionParams.at(1), k3 = _distortionParams.at(2), k4 = _distortionParams.at(3);
    const double r = std::hypot(p(0), p(1));
    const double theta = std::atan(r);
    const double theta2 = theta*theta;
    const double theta3 = theta2*theta;
    const double theta4 = theta2*theta2;
    const double theta5 = theta4*theta;
    const double theta6 = theta3*theta3;
    const double theta7 = theta6*theta;
    const double theta8 = theta4*theta4;
    const double theta9 = theta8*theta;
    const double theta_dist = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;
    const double inv_r = r > eps ? 1.0/r : 1.0;
    const double cdist = r > eps ? theta_dist * inv_r : 1.0;

    return  p * cdist;
  }

  /// Remove distortion (return p' such that disto(p') = p)
  virtual Vec2 remove_disto(const Vec2& p) const override {
    const double eps = 1e-8;
    double scale = 1.0;
    const double theta_dist = std::hypot(p[0], p[1]);
    if (theta_dist > eps)
    {
      double theta = theta_dist;
      for (int j = 0; j < 10; ++j)
      {
        const double theta2 = theta*theta;
        const double theta4 = theta2*theta2;
        const double theta6 = theta4*theta2;
        const double theta8 = theta6*theta2;
        theta = theta_dist / (1 + _distortionParams.at(0) * theta2 + _distortionParams.at(1) * theta4 + _distortionParams.at(2) * theta6 + _distortionParams.at(3) * theta8);
      }
      scale = std::tan(theta) / theta_dist;
    }
    return p * scale;
  }
};

} // namespace camera
} // namespace aliceVision
