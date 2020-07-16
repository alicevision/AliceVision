// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
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

/**
 * @brief EquiDistantRadialK3 is a camera model used for fisheye optics with 3 radial distortion coefficients.
 * x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6)
 *
 * See https://en.wikipedia.org/wiki/Fisheye_lens
 */
class EquiDistantRadialK3 : public EquiDistant
{
public:
  EquiDistantRadialK3() = default;

  explicit EquiDistantRadialK3(int w, int h, double focalLengthPix, double ppx, double ppy,
                               double radius = 0.0, double k1 = 0.0, double k2 = 0.0, double k3 = 0.0)
    : EquiDistant(w, h, focalLengthPix, ppx, ppy, (radius != 0.0 ? radius : std::min(w, h) * 0.5), std::shared_ptr<Distortion>(new DistortionRadialK3PT(k1, k2, k3)))
  {
  }

  EquiDistantRadialK3* clone() const override { return new EquiDistantRadialK3(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const EquiDistantRadialK3&>(other); }

  EINTRINSIC getType() const override { return EQUIDISTANT_CAMERA_RADIAL3; }

  ~EquiDistantRadialK3() override = default;
};

} // namespace camera
} // namespace aliceVision
