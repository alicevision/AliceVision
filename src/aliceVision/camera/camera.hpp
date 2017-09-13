// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/camera/cameraCommon.hpp"
#include "aliceVision/camera/IntrinsicBase.hpp"
#include "aliceVision/camera/Pinhole.hpp"
#include "aliceVision/camera/PinholeRadial.hpp"
#include "aliceVision/camera/PinholeBrown.hpp"
#include "aliceVision/camera/PinholeFisheye.hpp"
#include "aliceVision/camera/PinholeFisheye1.hpp"
#include "aliceVision/camera/cameraUndistortImage.hpp"

namespace aliceVision {
namespace camera {

inline std::shared_ptr<Pinhole> createPinholeIntrinsic(EINTRINSIC intrinsicType,
    unsigned int w = 0, unsigned int h = 0,
    double focal_length_pix = 0.0,
    double ppx = 0.0, double ppy = 0.0)
{
  switch(intrinsicType)
  {
    case EINTRINSIC::PINHOLE_CAMERA:
      return std::make_shared<Pinhole>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
      return std::make_shared<PinholeRadialK1>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
      return std::make_shared<PinholeRadialK3>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
      return std::make_shared<PinholeBrownT2>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
      return std::make_shared<PinholeFisheye>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
      return std::make_shared<PinholeFisheye1>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_END:
    case EINTRINSIC::PINHOLE_CAMERA_START:
      break;
  }
  throw std::out_of_range("Unrecognized Intrinsic Enum");
}

} // namespace camera
} // namespace aliceVision
