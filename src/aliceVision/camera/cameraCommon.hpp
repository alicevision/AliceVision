// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/detail/bitmask.hpp>
#include <boost/algorithm/string.hpp>

#include <string>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <string>

namespace aliceVision {
namespace camera {

enum EINTRINSIC
{
    UNKNOWN = (1u << 0),
    PINHOLE_CAMERA = (1u << 1),          // no distortion
    PINHOLE_CAMERA_RADIAL1 = (1u << 2),  // radial distortion K1
    PINHOLE_CAMERA_RADIAL3 = (1u << 3),  // radial distortion K1,K2,K3
    PINHOLE_CAMERA_BROWN = (1u << 4),    // radial distortion K1,K2,K3, tangential distortion T1,T2
    PINHOLE_CAMERA_FISHEYE = (1u << 5),  // a simple Fish-eye distortion model with 4 distortion coefficients
    PINHOLE_CAMERA_FISHEYE1 = (1u << 6), // a simple Fish-eye distortion model with 1 distortion coefficient
    PINHOLE_CAMERA_3DEANAMORPHIC4 = (1u << 7), // a simple anamorphic distortion model
    PINHOLE_CAMERA_3DECLASSICLD = (1u << 8), // a simple anamorphic distortion model
    PINHOLE_CAMERA_3DERADIAL4 = (1u << 9), // a simple anamorphic distortion model
    EQUIDISTANT_CAMERA = (1u << 10),      // an equidistant model
    EQUIDISTANT_CAMERA_RADIAL3 = (1u << 11),  // an equidistant model with radial distortion
    VALID_PINHOLE = PINHOLE_CAMERA | PINHOLE_CAMERA_RADIAL1 | PINHOLE_CAMERA_RADIAL3 | PINHOLE_CAMERA_3DERADIAL4 | PINHOLE_CAMERA_BROWN |
                    PINHOLE_CAMERA_3DEANAMORPHIC4 | PINHOLE_CAMERA_3DECLASSICLD| PINHOLE_CAMERA_FISHEYE | PINHOLE_CAMERA_FISHEYE1,
    VALID_EQUIDISTANT = EQUIDISTANT_CAMERA | EQUIDISTANT_CAMERA_RADIAL3,
    VALID_CAMERA_MODEL = VALID_PINHOLE | VALID_EQUIDISTANT,
};

BOOST_BITMASK(EINTRINSIC)

inline std::string EINTRINSIC_enumToString(EINTRINSIC intrinsic)
{
  switch(intrinsic)
  {
      case EINTRINSIC::PINHOLE_CAMERA: return "pinhole";
      case EINTRINSIC::PINHOLE_CAMERA_RADIAL1: return "radial1";
      case EINTRINSIC::PINHOLE_CAMERA_RADIAL3: return "radial3";
      case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4: return "3deradial4";
      case EINTRINSIC::PINHOLE_CAMERA_BROWN: return "brown";
      case EINTRINSIC::PINHOLE_CAMERA_FISHEYE: return "fisheye4";
      case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1: return "fisheye1";
      case EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4: return "3deanamorphic4";
      case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD: return "3declassicld";
      case EINTRINSIC::EQUIDISTANT_CAMERA: return "equidistant";
      case EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3: return "equidistant_r3";
      case EINTRINSIC::UNKNOWN:
      case EINTRINSIC::VALID_PINHOLE:
      case EINTRINSIC::VALID_EQUIDISTANT:
      case EINTRINSIC::VALID_CAMERA_MODEL:
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
  if(type == "3deradial4") return EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4;
  if(type == "brown") return EINTRINSIC::PINHOLE_CAMERA_BROWN;
  if(type == "fisheye4") return EINTRINSIC::PINHOLE_CAMERA_FISHEYE;
  if(type == "fisheye1") return EINTRINSIC::PINHOLE_CAMERA_FISHEYE1;
  if(type == "3deanamorphic4") return EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4;
  if(type == "3declassicld") return EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD;
  if(type == "equidistant") return EINTRINSIC::EQUIDISTANT_CAMERA;
  if(type == "equidistant_r3") return EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3;

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
    return EINTRINSIC::VALID_CAMERA_MODEL & eintrinsic;
}

inline bool isPinhole(EINTRINSIC eintrinsic)
{
    return EINTRINSIC::VALID_PINHOLE & eintrinsic;
}

inline bool isEquidistant(EINTRINSIC eintrinsic)
{
    return EINTRINSIC::VALID_EQUIDISTANT & eintrinsic;
}

inline EINTRINSIC EINTRINSIC_parseStringToBitmask(const std::string& str, const std::string& joinChar = ",")
{   
    std::vector<std::string> intrinsicsVec;
    boost::split(intrinsicsVec, str, boost::is_any_of(joinChar));
    if(!intrinsicsVec.size())
    {
        throw std::invalid_argument("'" + str + "'can't be parsed to EINTRINSIC Bitmask."); 
    }
    EINTRINSIC mask = EINTRINSIC_stringToEnum(intrinsicsVec[0]);
    for(const std::string& intrinsic : intrinsicsVec)
    {
        mask |= EINTRINSIC_stringToEnum(intrinsic);
    }
    return mask;
}

} // namespace camera
} // namespace aliceVision
