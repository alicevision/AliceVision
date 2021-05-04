// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>

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

  explicit PinholeFisheye(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double k1 = 0.0, double k2 = 0.0, double k3 = 0.0, double k4 = 0.0)
  :Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionFisheye(k1, k2, k3, k4)))
  {
  }

  PinholeFisheye* clone() const override { return new PinholeFisheye(*this); }
  void assign(const IntrinsicBase& other) override
  {
      *this = dynamic_cast<const PinholeFisheye&>(other);
  }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_FISHEYE; }

  bool isVisibleRay(const Vec3 & ray) const override
  {
      return ray(2) >= 0.0;
  }

  ~PinholeFisheye() override = default;
};

} // namespace camera
} // namespace aliceVision
