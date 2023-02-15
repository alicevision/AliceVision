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
 * @brief Camera Intrinsic initialization mode
 */
enum class EIntrinsicInitMode : std::uint8_t
{
  NONE = 0, //< Value not set
  CALIBRATED, //< External calibration
  ESTIMATED, //< Estimated, in the standard pipeline it is estimated from metadata information (FocalLength + sensor width or FocalLengthIn35mm with integer approximation)
  UNKNOWN //< The camera parameters are unknown (can still have a default value guess)
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
    case EIntrinsicInitMode::CALIBRATED: return "calibrated";
    case EIntrinsicInitMode::ESTIMATED:  return "estimated";
    case EIntrinsicInitMode::UNKNOWN:    return "unknown";
    case EIntrinsicInitMode::NONE:       return "none";
  }
  throw std::out_of_range("Invalid Intrinsic init mode enum: " + std::to_string(int(intrinsicInitMode)));
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

  if(mode == "calibrated") return EIntrinsicInitMode::CALIBRATED;
  if(mode == "estimated")  return EIntrinsicInitMode::ESTIMATED;
  if(mode == "unknown")    return EIntrinsicInitMode::UNKNOWN;
  if(mode == "none")       return EIntrinsicInitMode::NONE;

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

/**
 * @brief Camera Intrinsic Disto initialization mode
 */
enum class EIntrinsicDistoInitMode : std::uint8_t
{
    NONE = 0, //< Value not set
    CALIBRATED, //< Full calibration
    ESTIMATED, //< Estimated from LCP profile
    UNKNOWN //< The camera parameters are unknown (can still have a default value guess)
};

/**
 * @brief convert an enum EIntrinsicDistoInitMode to its corresponding string
 * @param EIntrinsicDistoInitMode
 * @return String
 */
inline std::string EIntrinsicDistoInitMode_enumToString(EIntrinsicDistoInitMode intrinsicDistoInitMode)
{
    switch (intrinsicDistoInitMode)
    {
    case EIntrinsicDistoInitMode::CALIBRATED: return "calibrated";
    case EIntrinsicDistoInitMode::ESTIMATED:  return "estimated";
    case EIntrinsicDistoInitMode::UNKNOWN:    return "unknown";
    case EIntrinsicDistoInitMode::NONE:       return "none";
    }
    throw std::out_of_range("Invalid Disto init mode enum: " + std::to_string(int(intrinsicDistoInitMode)));
}

/**
 * @brief convert a string Disto init mode to its corresponding enum EIntrinsicDistoInitMode
 * @param String
 * @return EIntrinsicDistoInitMode
 */
inline EIntrinsicDistoInitMode EIntrinsicDistoInitMode_stringToEnum(const std::string& intrinsicDistoInitMode)
{
    std::string mode = intrinsicDistoInitMode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower); //tolower

    if (mode == "calibrated") return EIntrinsicDistoInitMode::CALIBRATED;
    if (mode == "estimated")  return EIntrinsicDistoInitMode::ESTIMATED;
    if (mode == "unknown")    return EIntrinsicDistoInitMode::UNKNOWN;
    if (mode == "none")       return EIntrinsicDistoInitMode::NONE;

    throw std::out_of_range("Invalid Disto init mode: " + intrinsicDistoInitMode);
}

inline std::ostream& operator<<(std::ostream& os, const EIntrinsicDistoInitMode intrinsicDistoInitMode)
{
    os << EIntrinsicDistoInitMode_enumToString(intrinsicDistoInitMode);
    return os;
}

inline std::istream& operator>>(std::istream& in, EIntrinsicDistoInitMode& intrinsicDistoInitMode)
{
    std::string token;
    in >> token;
    intrinsicDistoInitMode = EIntrinsicDistoInitMode_stringToEnum(token);
    return in;
}

} // namespace camera
} // namespace aliceVision
