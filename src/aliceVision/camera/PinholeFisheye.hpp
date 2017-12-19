// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/**
 * Implement a simple Fish-eye camera model
 *
 * This is an adaptation of the Fisheye distortion model implemented in OpenCV:
 * https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/fisheye.cpp
 */
class PinholeFisheye : public Pinhole
{
  public:

  PinholeFisheye(
    int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double k4 = 0.0)
        :Pinhole(w, h, focal, ppx, ppy, {k1, k2, k3, k4})
  {
  }

  PinholeFisheye* clone() const { return new PinholeFisheye(*this); }
  void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeFisheye&>(other); }

  EINTRINSIC getType() const { return PINHOLE_CAMERA_FISHEYE; }

  virtual bool have_disto() const { return true;}

  virtual Vec2 add_disto(const Vec2 & p) const
  {
    const std::vector<double>& distortionParams = getDistortionParams();
    const double eps = 1e-8;
    const double k1 = _distortionParams.at(0), k2 = _distortionParams.at(1), k3 = _distortionParams.at(2), k4 = _distortionParams.at(3);
    const double r = std::hypot(p(0), p(1));
    const double theta = std::atan(r);
    const double
      theta2 = theta*theta,
      theta3 = theta2*theta,
      theta4 = theta2*theta2,
      theta5 = theta4*theta,
      theta6 = theta3*theta3,
      theta7 = theta6*theta,
      theta8 = theta4*theta4,
      theta9 = theta8*theta;
    const double theta_dist = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;
    const double inv_r = r > eps ? 1.0/r : 1.0;
    const double cdist = r > eps ? theta_dist * inv_r : 1.0;
    return  p*cdist;
  }

  virtual Vec2 remove_disto(const Vec2 & p) const
  {
    const double eps = 1e-8;
    double scale = 1.0;
    const double theta_dist = std::hypot(p[0], p[1]);
    if (theta_dist > eps)
    {
      double theta = theta_dist;
      for (int j = 0; j < 10; ++j)
      {
        const double
          theta2 = theta*theta,
          theta4 = theta2*theta2,
          theta6 = theta4*theta2,
          theta8 = theta6*theta2;
        theta = theta_dist /
          (1 + _distortionParams.at(0) * theta2
             + _distortionParams.at(1) * theta4
             + _distortionParams.at(2) * theta6
             + _distortionParams.at(3) * theta8);
      }
      scale = std::tan(theta) / theta_dist;
    }
    return p * scale;
  }

  /// Return the un-distorted pixel (with removed distortion)
  virtual Vec2 get_ud_pixel(const Vec2& p) const
  {
    return cam2ima( remove_disto(ima2cam(p)) );
  }

  /// Return the distorted pixel (with added distortion)
  virtual Vec2 get_d_pixel(const Vec2& p) const
  {
    return cam2ima( add_disto(ima2cam(p)) );
  }
};

} // namespace camera
} // namespace aliceVision
