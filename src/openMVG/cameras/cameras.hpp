// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_CAMERAS_HPP
#define ALICEVISION_CAMERAS_HPP

#include "aliceVision/cameras/Camera_Common.hpp"
#include "aliceVision/cameras/Camera_Intrinsics.hpp"
#include "aliceVision/cameras/Camera_Pinhole.hpp"
#include "aliceVision/cameras/Camera_Pinhole_Radial.hpp"
#include "aliceVision/cameras/Camera_Pinhole_Brown.hpp"
#include "aliceVision/cameras/Camera_Pinhole_Fisheye.hpp"
#include "aliceVision/cameras/Camera_Pinhole_Fisheye1.hpp"
#include "aliceVision/cameras/Camera_undistort_image.hpp"

namespace aliceVision {
namespace cameras {

inline std::shared_ptr<Pinhole_Intrinsic> createPinholeIntrinsic(EINTRINSIC intrinsicType,
    unsigned int w = 0, unsigned int h = 0,
    double focal_length_pix = 0.0,
    double ppx = 0.0, double ppy = 0.0)
{
  switch(intrinsicType)
  {
    case EINTRINSIC::PINHOLE_CAMERA:
      return std::make_shared<Pinhole_Intrinsic>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
      return std::make_shared<Pinhole_Intrinsic_Radial_K1>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
      return std::make_shared<Pinhole_Intrinsic_Radial_K3>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
      return std::make_shared<Pinhole_Intrinsic_Brown_T2>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
      return std::make_shared<Pinhole_Intrinsic_Fisheye>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
      return std::make_shared<Pinhole_Intrinsic_Fisheye1>(w, h, focal_length_pix, ppx, ppy);
    case EINTRINSIC::PINHOLE_CAMERA_END:
    case EINTRINSIC::PINHOLE_CAMERA_START:
      break;
  }
  throw std::out_of_range("Unrecognized Intrinsic Enum");
}

} // namespace cameras
} // namespace aliceVision

#endif // ALICEVISION_CAMERAS_HPP
