// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/// Implement a Pinhole camera with a 1 radial distortion coefficient.
/// x_d = x_u (1 + K_1 r^2)
class EquiDistantRadialK1 : public EquiDistant
{
  public:

  EquiDistantRadialK1(int w = 0, int h = 0, double focal = 0.0, double ppx = 0, double ppy = 0, double radius = 1980.0, double k1 = 0.0)
  :EquiDistant(w, h, focal, ppx, ppy, radius, std::shared_ptr<Distortion>(new DistortionRadialK1(k1)))
  {
  }

  EquiDistantRadialK1* clone() const override { return new EquiDistantRadialK1(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const EquiDistantRadialK1&>(other); }

  EINTRINSIC getType() const override { return EQUIDISTANT_CAMERA_RADIAL1; }
};

/// Implement a Pinhole camera with a 3 radial distortion coefficients.
/// x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6)
class EquiDistantRadialK3 : public EquiDistant
{
  public:

  EquiDistantRadialK3(int w = 0, int h = 0, double focal = 0.0, double ppx = 0, double ppy = 0, double radius = 1980.0, double k1 = 0.0, double k2 = 0.0, double k3 = 0.0)
  : EquiDistant(w, h, focal, ppx, ppy, radius, std::shared_ptr<Distortion>(new DistortionRadialK3PT(k1, k2, k3)))
  {
  }

  EquiDistantRadialK3* clone() const override { return new EquiDistantRadialK3(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const EquiDistantRadialK3&>(other); }

  EINTRINSIC getType() const override { return EQUIDISTANT_CAMERA_RADIAL3; }
};

} // namespace camera
} // namespace aliceVision
