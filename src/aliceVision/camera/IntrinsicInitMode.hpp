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
 * @brief Camera Intrinsic and Distortion initialization mode
 */
enum class EInitMode : std::uint8_t
{
    NONE = 0,    //< Value not set
    CALIBRATED,  //< External calibration
    ESTIMATED,   //< Estimated, in the standard pipeline it is estimated from metadata information (FocalLength + sensor width or FocalLengthIn35mm
                 // with integer approximation)
    UNKNOWN      //< The camera parameters are unknown (can still have a default value guess)
};

/**
 * @brief convert an enum EInitMode to its corresponding string
 * @param EInitMode
 * @return String
 */
inline std::string EInitMode_enumToString(EInitMode initMode)
{
    switch (initMode)
    {
        case EInitMode::CALIBRATED:
            return "calibrated";
        case EInitMode::ESTIMATED:
            return "estimated";
        case EInitMode::UNKNOWN:
            return "unknown";
        case EInitMode::NONE:
            return "none";
    }
    throw std::out_of_range("Invalid init mode enum: " + std::to_string(int(initMode)));
}

/**
 * @brief convert a string init mode to its corresponding enum EInitMode
 * @param String
 * @return EInitMode
 */
inline EInitMode EInitMode_stringToEnum(const std::string& initMode)
{
    std::string mode = initMode;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);  // tolower

    if (mode == "calibrated")
        return EInitMode::CALIBRATED;
    if (mode == "estimated")
        return EInitMode::ESTIMATED;
    if (mode == "unknown")
        return EInitMode::UNKNOWN;
    if (mode == "none")
        return EInitMode::NONE;

    throw std::out_of_range("Invalid init mode: " + initMode);
}

inline std::ostream& operator<<(std::ostream& os, const EInitMode initMode)
{
    os << EInitMode_enumToString(initMode);
    return os;
}

inline std::istream& operator>>(std::istream& in, EInitMode& initMode)
{
    std::string token;
    in >> token;
    initMode = EInitMode_stringToEnum(token);
    return in;
}

}  // namespace camera
}  // namespace aliceVision
