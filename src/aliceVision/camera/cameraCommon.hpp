// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <stdexcept>
#include <algorithm>

namespace aliceVision {
namespace camera {

enum EINTRINSIC
{
  PINHOLE_CAMERA = 0,          // no distortion
  PINHOLE_CAMERA_RADIAL1 = 1,  // radial distortion K1
  PINHOLE_CAMERA_RADIAL3 = 2,  // radial distortion K1,K2,K3
  PINHOLE_CAMERA_BROWN = 3,    // radial distortion K1,K2,K3, tangential distortion T1,T2
  PINHOLE_CAMERA_FISHEYE = 4,  // a simple Fish-eye distortion model with 4 distortion coefficients
  PINHOLE_CAMERA_FISHEYE1 = 5, // a simple Fish-eye distortion model with 1 distortion coefficient
  EQUIDISTANT_CAMERA = 6,
  EQUIDISTANT_CAMERA_RADIAL3 = 7,
  CAMERA_END
};

inline std::string EINTRINSIC_enumToString(EINTRINSIC intrinsic)
{
  switch(intrinsic)
  {
    case PINHOLE_CAMERA:          return "pinhole";
    case PINHOLE_CAMERA_RADIAL1:  return "radial1";
    case PINHOLE_CAMERA_RADIAL3:  return "radial3";
    case PINHOLE_CAMERA_BROWN:    return "brown";
    case PINHOLE_CAMERA_FISHEYE:  return "fisheye4";
    case PINHOLE_CAMERA_FISHEYE1: return "fisheye1";
    case EQUIDISTANT_CAMERA:      return "equidistant";
    case EQUIDISTANT_CAMERA_RADIAL3:      return "equidistant_r3";
    case CAMERA_END:
      break;
  }
  throw std::out_of_range("Invalid Intrinsic Enum");
}

inline EINTRINSIC EINTRINSIC_stringToEnum(const std::string& intrinsic)
{
  std::string type = intrinsic;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

  if(type == "pinhole")  return PINHOLE_CAMERA;
  if(type == "radial1")  return PINHOLE_CAMERA_RADIAL1;
  if(type == "radial3")  return PINHOLE_CAMERA_RADIAL3;
  if(type == "brown")    return PINHOLE_CAMERA_BROWN;
  if(type == "fisheye4") return PINHOLE_CAMERA_FISHEYE;
  if(type == "fisheye1") return PINHOLE_CAMERA_FISHEYE1;
  if(type == "equidistant") return EQUIDISTANT_CAMERA;
  if(type == "equidistant_r3") return EQUIDISTANT_CAMERA_RADIAL3;

  throw std::out_of_range(intrinsic);
}

// Return if the camera type is a valid enum
inline bool isValid(EINTRINSIC eintrinsic)
{
  return eintrinsic > PINHOLE_CAMERA && eintrinsic < CAMERA_END;
}

inline bool isPinhole(EINTRINSIC eintrinsic)
{
  switch (eintrinsic) {
  case PINHOLE_CAMERA:
  case PINHOLE_CAMERA_RADIAL1:
  case PINHOLE_CAMERA_RADIAL3:
  case PINHOLE_CAMERA_BROWN:
  case PINHOLE_CAMERA_FISHEYE:
  case PINHOLE_CAMERA_FISHEYE1:
    return true;
  default:
    return false;
  }
}

inline bool isEquidistant(EINTRINSIC eintrinsic)
{
  switch (eintrinsic) {
  case EQUIDISTANT_CAMERA:
  case EQUIDISTANT_CAMERA_RADIAL3:
    return true;
  default:
    return false;
  }
}

} // namespace camera
} // namespace aliceVision
