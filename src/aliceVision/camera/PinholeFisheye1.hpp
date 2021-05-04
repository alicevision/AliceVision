// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>

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

  explicit PinholeFisheye1(int w = 0, int h = 0, double focalLengthPixX = 0.0, double focalLengthPixY = 0.0, double ppx = 0, double ppy = 0, double k1 = 0.0)
  :Pinhole(w, h, focalLengthPixX, focalLengthPixY, ppx, ppy, std::shared_ptr<Distortion>(new DistortionFisheye1(k1)))
  {
  }

  PinholeFisheye1* clone() const override
  {
      return new PinholeFisheye1(*this);
  }

  void assign(const IntrinsicBase& other) override
  {
      *this = dynamic_cast<const PinholeFisheye1&>(other);
  }

  EINTRINSIC getType() const override { return EINTRINSIC::PINHOLE_CAMERA_FISHEYE1; }

  ~PinholeFisheye1() override = default;
};

} // namespace camera
} // namespace aliceVision
