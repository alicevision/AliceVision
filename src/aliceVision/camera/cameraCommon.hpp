// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/detail/bitmask.hpp>

#include <string>
#include <stdexcept>
#include <algorithm>

namespace aliceVision {
namespace camera {

enum EINTRINSIC
{
  PINHOLE_CAMERA_START = (1u << 0),
  PINHOLE_CAMERA = (1u << 1),  // no distortion
  PINHOLE_CAMERA_RADIAL1 = (1u << 2), // radial distortion K1
  PINHOLE_CAMERA_RADIAL3 = (1u << 3),  // radial distortion K1,K2,K3
  PINHOLE_CAMERA_BROWN = (1u << 4),    // radial distortion K1,K2,K3, tangential distortion T1,T2
  PINHOLE_CAMERA_FISHEYE = (1u << 5),  // a simple Fish-eye distortion model with 4 distortion coefficients
  PINHOLE_CAMERA_FISHEYE1 = (1u << 6), // a simple Fish-eye distortion model with 1 distortion coefficient
  PINHOLE_CAMERA_END = (1u << 7)
};

BOOST_BITMASK(EINTRINSIC)

inline std::string EINTRINSIC_enumToString(EINTRINSIC intrinsic)
{
  switch(intrinsic)
  {
      case EINTRINSIC::PINHOLE_CAMERA: return "pinhole";
      case EINTRINSIC::PINHOLE_CAMERA_RADIAL1: return "radial1";
      case EINTRINSIC::PINHOLE_CAMERA_RADIAL3: return "radial3";
      case EINTRINSIC::PINHOLE_CAMERA_BROWN: return "brown";
      case EINTRINSIC::PINHOLE_CAMERA_FISHEYE: return "fisheye4";
      case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1: return "fisheye1";
      case EINTRINSIC::PINHOLE_CAMERA_START:
      case EINTRINSIC::PINHOLE_CAMERA_END:
      break;
  }
  throw std::out_of_range("Invalid Intrinsic Enum");
}

inline EINTRINSIC EINTRINSIC_stringToEnum(const std::string& intrinsic)
{
  std::string type = intrinsic;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

  if(type == "pinhole") return EINTRINSIC::PINHOLE_CAMERA;
  if(type == "radial1") return EINTRINSIC::PINHOLE_CAMERA_RADIAL1;
  if(type == "radial3") return EINTRINSIC::PINHOLE_CAMERA_RADIAL3;
  if(type == "brown") return EINTRINSIC::PINHOLE_CAMERA_BROWN;
  if(type == "fisheye4") return EINTRINSIC::PINHOLE_CAMERA_FISHEYE;
  if(type == "fisheye1") return EINTRINSIC::PINHOLE_CAMERA_FISHEYE1;

  throw std::out_of_range(intrinsic);
}

inline std::ostream& operator<<(std::ostream& os, EINTRINSIC e)
{
    return os << EINTRINSIC_enumToString(e);
}

inline std::istream& operator>>(std::istream& in, EINTRINSIC& e)
{
    std::string token;
    in >> token;
    e = EINTRINSIC_stringToEnum(token);
    return in;
}

// Return if the camera type is a valid enum
inline bool isValid(EINTRINSIC eintrinsic)
{
    return eintrinsic > EINTRINSIC::PINHOLE_CAMERA_START && eintrinsic < EINTRINSIC::PINHOLE_CAMERA_END;
}

inline bool isPinhole(EINTRINSIC eintrinsic)
{
    return eintrinsic > EINTRINSIC::PINHOLE_CAMERA_START && eintrinsic < EINTRINSIC::PINHOLE_CAMERA_END;
}

} // namespace camera
} // namespace aliceVision
