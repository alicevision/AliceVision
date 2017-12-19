// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
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
 * Implement a simple Fish-eye camera model with only one parameter
 * 
 * Fredreric Devernay and Olivier Faugeras. 2001. Straight lines have to be 
 * straight: automatic calibration and removal of distortion from scenes of 
 * structured environments. Mach. Vision Appl. 13, 1 (August 2001), 14-24. 
 * DOI: 10.1007/PL00013269 https://hal.inria.fr/inria-00267247/document
 */
class PinholeFisheye1 : public Pinhole
{
public:

  PinholeFisheye1(
    int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0)
        :Pinhole(w, h, focal, ppx, ppy, {k1})
  {
  }

  PinholeFisheye1* clone() const { return new PinholeFisheye1(*this); }
  void assign(const IntrinsicBase& other) { *this = dynamic_cast<const PinholeFisheye1&>(other); }

  EINTRINSIC getType() const { return PINHOLE_CAMERA_FISHEYE1; }

  virtual bool have_disto() const { return true;}

  virtual Vec2 add_disto(const Vec2 & p) const
  {
    const double k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = (std::atan(2.0 * r * std::tan(0.5 * k1)) / k1) / r;
    return  p * coef;
  }

  virtual Vec2 remove_disto(const Vec2 & p) const
  {
    const double k1 = _distortionParams.at(0);
    const double r = std::hypot(p(0), p(1));
    const double coef = 0.5 * std::tan(r * k1) / (std::tan(0.5 * k1) * r);
    return  p * coef;
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
