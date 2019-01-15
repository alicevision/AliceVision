// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace aliceVision {
namespace camera {

/**
 * @brief Intrinsic init mode enum
 */
enum class EIntrinsicInitMode : std::uint8_t
{
  CALIBRATED
  , COMPUTED_FROM_METADATA
  , ESTIMATED_FROM_METADATA
  , SET_FROM_DEFAULT_FOV
};

/**
 * @brief convert an enum EIntrinsicInitMode to its corresponding string
 * @param EIntrinsicInitMode
 * @return String
 */
inline std::string EIntrinsicInitMode_enumToString(EIntrinsicInitMode intrinsicInitMode)
{
  switch(intrinsicInitMode)
  {
    case EIntrinsicInitMode::CALIBRATED:               return "calibrated";
    case EIntrinsicInitMode::COMPUTED_FROM_METADATA:   return "computed_from_metadata";
    case EIntrinsicInitMode::ESTIMATED_FROM_METADATA:  return "estimated_from_metadata";
    case EIntrinsicInitMode::SET_FROM_DEFAULT_FOV:     return "set_from_default_fov";
  }
  throw std::out_of_range("Invalid Intrinsic init mode enum");
}

/**
 * @brief convert a string Intrinsic init mode to its corresponding enum EIntrinsicInitMode
 * @param String
 * @return EIntrinsicInitMode
 */
inline EIntrinsicInitMode EIntrinsicInitMode_stringToEnum(const std::string& intrinsicInitMode)
{
  std::string mode = intrinsicInitMode;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower); //tolower

  if(mode == "calibrated")               return EIntrinsicInitMode::CALIBRATED;
  if(mode == "computed_from_metadata")   return EIntrinsicInitMode::COMPUTED_FROM_METADATA;
  if(mode == "estimated_from_metadata")  return EIntrinsicInitMode::ESTIMATED_FROM_METADATA;
  if(mode == "set_from_default_fov")     return EIntrinsicInitMode::SET_FROM_DEFAULT_FOV;

  throw std::out_of_range("Invalid Intrinsic init mode: " + intrinsicInitMode);
}

inline std::ostream& operator<<(std::ostream& os, const EIntrinsicInitMode intrinsicInitMode)
{
  os << EIntrinsicInitMode_enumToString(intrinsicInitMode);
  return os;
}

inline std::istream& operator>>(std::istream& in, EIntrinsicInitMode &intrinsicInitMode)
{
  std::string token;
  in >> token;
  intrinsicInitMode = EIntrinsicInitMode_stringToEnum(token);
  return in;
}

} // namespace camera
} // namespace aliceVision
