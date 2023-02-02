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
enum class EInitMode : std::uint8_t
{
    NONE = 0, //< Value not set
    CALIBRATED, //< External calibration
    ESTIMATED, //< Estimated, in the standard pipeline it is estimated from metadata information (FocalLength + sensor width or FocalLengthIn35mm with integer approximation)
    UNKNOWN //< The camera parameters are unknown (can still have a default value guess)
};

/**
 * @brief convert an enum EInitMode to its corresponding string
 * @param EInitMode
 * @return String
 */
inline std::string EInitMode_enumToString(EInitMode intrinsicInitMode)
{
    switch (intrinsicInitMode)
    {
    case EIntrinsicInitMode::CALIBRATED: return "calibrated";
    case EIntrinsicInitMode::ESTIMATED:  return "estimated";
    case EIntrinsicInitMode::UNKNOWN:    return "unknown";
    case EIntrinsicInitMode::NONE:       return "none";
    }
    throw std::out_of_range("Invalid Intrinsic init mode enum: " + std::to_string(int(intrinsicInitMode)));
}

/**
 * @brief convert a string Intrinsic init mode to its corresponding enum EInitMode
 * @param String
 * @return EInitMode
 */
inline EInitMode EInitMode_stringToEnum(const std::string& intrinsicInitMode)
{
    std::string mode = intrinsicInitMode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower); //tolower

    if (mode == "calibrated") return EIntrinsicInitMode::CALIBRATED;
    if (mode == "estimated")  return EIntrinsicInitMode::ESTIMATED;
    if (mode == "unknown")    return EIntrinsicInitMode::UNKNOWN;
    if (mode == "none")       return EIntrinsicInitMode::NONE;

    throw std::out_of_range("Invalid Intrinsic init mode: " + intrinsicInitMode);
}

inline std::ostream& operator<<(std::ostream& os, const EInitMode intrinsicInitMode)
{
    os << EIntrinsicInitMode_enumToString(intrinsicInitMode);
    return os;
}

inline std::istream& operator>>(std::istream& in, EInitMode &intrinsicInitMode)
{
    std::string token;
    in >> token;
    intrinsicInitMode = EIntrinsicInitMode_stringToEnum(token);
    return in;
}

} // namespace camera
} // namespace aliceVision
