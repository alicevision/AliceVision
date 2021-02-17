// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/// Implement a Pinhole camera with a 1 radial distortion coefficient.
/// x_d = x_u (1 + K_1 r^2)
class PinholeRadialK1 : public Pinhole
{
public:

  explicit PinholeRadialK1(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double k1 = 0.0)
  :Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionRadialK1(k1)))
  {
  }

  PinholeRadialK1* clone() const override { return new PinholeRadialK1(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeRadialK1&>(other); }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_RADIAL1; }

  ~PinholeRadialK1() override = default;
};

/// Implement a Pinhole camera with a 3 radial distortion coefficients.
/// x_d = x_u (1 + K_1 r^2 + K_2 r^4 + K_3 r^6)
class PinholeRadialK3 : public Pinhole
{
  public:

  explicit PinholeRadialK3(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double k1 = 0.0, double k2 = 0.0, double k3 = 0.0)
  : Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionRadialK3(k1, k2, k3)))
  {
  }

  PinholeRadialK3* clone() const override { return new PinholeRadialK3(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeRadialK3&>(other); }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_RADIAL3; }

  ~PinholeRadialK3() override = default;
};

/// Implement a Pinhole camera with 3DE radial4 model
class PinholeRadial3DE : public Pinhole
{
  public:

  explicit PinholeRadial3DE(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double c2 = 0.0, double c4 = 0.0, double u1 = 0.0, double v1 = 0.0, double u2 = 0.0, double v2 = 0.0)
  : Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionRadial3DE(c2, c4, u1, v1, u2, v2)))
  {
  }

  PinholeRadial3DE* clone() const override { return new PinholeRadial3DE(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeRadial3DE&>(other); }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_RADIAL3DE; }

  ~PinholeRadial3DE() override = default;
};

/// Implement a Pinhole camera with a 4 anamorphic distortion coefficients.
class PinholeAnamorphic4 : public Pinhole
{
  public:

  explicit PinholeAnamorphic4(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double cxx = 0.0, double cxy = 0.0, double cyx = 0.0, double cyy = 0.0)
  : Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionAnamorphic4(cxx, cxy, cyx, cyy)))
  {
  }

  PinholeAnamorphic4 * clone() const override { return new PinholeAnamorphic4(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeAnamorphic4&>(other); }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_ANAMORPHIC4; }

  ~PinholeAnamorphic4() override = default;
};

/// Implement a Pinhole camera with a 10 anamorphic distortion coefficients.
class PinholeAnamorphic10 : public Pinhole
{
  public:

  explicit PinholeAnamorphic10(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double cxx = 0.0, double cxy = 0.0, double cxxx = 0.0, double cxxy = 0.0, double cxyy = 0.0, double cyx = 0.0, double cyy = 0.0, double cyxx = 0.0, double cyxy = 0.0, double cyyy = 0.0)
  : Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionAnamorphic10(cxx, cxy, cxxx, cxxy, cxyy, cyx, cyy, cyxx, cyxy, cyyy)))
  {
  }

  PinholeAnamorphic10 * clone() const override { return new PinholeAnamorphic10(*this); }
  void assign(const IntrinsicBase& other) override { *this = dynamic_cast<const PinholeAnamorphic10&>(other); }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_ANAMORPHIC10; }

  ~PinholeAnamorphic10() override = default;
};

} // namespace camera
} // namespace aliceVision
