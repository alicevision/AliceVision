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

enum EDISTORTION
{
    DISTORTION_NONE,
    DISTORTION_RADIALK1,
    DISTORTION_RADIALK3,
    DISTORTION_RADIALK3PT,
    DISTORTION_BROWN,
    DISTORTION_FISHEYE,
    DISTORTION_FISHEYE1,
    DISTORTION_3DERADIAL4,
    DISTORTION_3DEANAMORPHIC4,
    DISTORTION_3DECLASSICLD
};

enum EUNDISTORTION
{
    UNDISTORTION_NONE,
    UNDISTORTION_RADIALK3,
    UNDISTORTION_3DEANAMORPHIC4
};

enum EINTRINSIC
{
    UNKNOWN = (1u << 0),
    PINHOLE_CAMERA = (1u << 1),                 // plain pinhole model
    EQUIDISTANT_CAMERA = (1u << 2),            // plain equidistant model
};

BOOST_BITMASK(EINTRINSIC);

inline std::string EINTRINSIC_enumToString(EINTRINSIC intrinsic)
{
    switch (intrinsic)
    {
        case EINTRINSIC::PINHOLE_CAMERA:
            return "pinhole";
        case EINTRINSIC::EQUIDISTANT_CAMERA:
            return "equidistant";
        case EINTRINSIC::UNKNOWN:
            break;
    }
    throw std::out_of_range("Invalid Intrinsic Enum");
}

inline EINTRINSIC EINTRINSIC_stringToEnum(const std::string& intrinsic)
{
    std::string type = intrinsic;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);  // tolower

    if (type == "pinhole")
    {
        return EINTRINSIC::PINHOLE_CAMERA;
    }

    if (type == "equidistant")
    {
        return EINTRINSIC::EQUIDISTANT_CAMERA;
    }

    throw std::out_of_range(intrinsic);
}

inline std::ostream& operator<<(std::ostream& os, EINTRINSIC e) { return os << EINTRINSIC_enumToString(e); }

inline std::istream& operator>>(std::istream& in, EINTRINSIC& e)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    e = EINTRINSIC_stringToEnum(token);
    return in;
}

inline bool isPinhole(EINTRINSIC eintrinsic) { return EINTRINSIC::PINHOLE_CAMERA & eintrinsic; }

inline bool isEquidistant(EINTRINSIC eintrinsic) { return EINTRINSIC::EQUIDISTANT_CAMERA & eintrinsic; }

// Return if the camera type is a valid enum
inline bool isValid(EINTRINSIC eintrinsic) { return isPinhole(eintrinsic) || isEquidistant(eintrinsic); }

inline EDISTORTION EDISTORTION_stringToEnum(const std::string& distortion)
{
    std::string type = distortion;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);  // tolower

    if (type == "radialk1")
    {
        return EDISTORTION::DISTORTION_RADIALK1;
    }
    else if (type == "radialk3")
    {
        return EDISTORTION::DISTORTION_RADIALK3;
    }
    else if (type == "radialk3pt")
    {
        return EDISTORTION::DISTORTION_RADIALK3PT;
    }
    else if (type == "brown")
    {
        return EDISTORTION::DISTORTION_BROWN;
    }
    else if (type == "fisheye")
    {
        return EDISTORTION::DISTORTION_FISHEYE;
    }
    else if (type == "fisheye1")
    {
        return EDISTORTION::DISTORTION_FISHEYE1;
    }
    else if (type == "3deradial4")
    {
        return EDISTORTION::DISTORTION_3DERADIAL4;
    }
    else if (type == "3deanamorphic4")
    {
        return EDISTORTION::DISTORTION_3DEANAMORPHIC4;
    }
    else if (type == "3declassicld")
    {
        return EDISTORTION::DISTORTION_3DECLASSICLD;
    }
    else if (type == "none")
    {
        return EDISTORTION::DISTORTION_NONE;
    }

    throw std::out_of_range(distortion);
}

inline std::string EDISTORTION_enumToString(EDISTORTION distortion)
{
    switch (distortion)
    {
        case EDISTORTION::DISTORTION_RADIALK1:
            return "radialk1";
        case EDISTORTION::DISTORTION_RADIALK3:
            return "radialk3";
        case EDISTORTION::DISTORTION_RADIALK3PT:
            return "radialk3pt";
        case EDISTORTION::DISTORTION_BROWN:
            return "brown";
        case EDISTORTION::DISTORTION_FISHEYE:
            return "fisheye";
        case EDISTORTION::DISTORTION_FISHEYE1:
            return "fisheye1";
        case EDISTORTION::DISTORTION_3DERADIAL4:
            return "3deradial4";
        case EDISTORTION::DISTORTION_3DEANAMORPHIC4:
            return "3deanamorphic4";
        case EDISTORTION::DISTORTION_3DECLASSICLD:
            return "3declassicld";
        case EDISTORTION::DISTORTION_NONE:
            return "none";
    }
    throw std::out_of_range("Invalid Undistortion Enum");
}

inline EUNDISTORTION EUNDISTORTION_stringToEnum(const std::string& undistortion)
{
    std::string type = undistortion;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);  // tolower

    
    if (type == "3deanamorphic4")
    {
        return EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4;
    }
    else if (type == "radialk3")
    {
        return EUNDISTORTION::UNDISTORTION_RADIALK3;
    }
    else if (type == "none")
    {
        return EUNDISTORTION::UNDISTORTION_NONE;
    }

    throw std::out_of_range(undistortion);
}

inline std::string EUNDISTORTION_enumToString(EUNDISTORTION undistortion)
{
    switch (undistortion)
    {
        case EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4:
            return "3deanamorphic4";
        case EUNDISTORTION::UNDISTORTION_RADIALK3:
            return "radialk3";
        case EUNDISTORTION::UNDISTORTION_NONE:
            return "none";
    }
    throw std::out_of_range("Invalid Undistortion Enum");
}

inline void compatibilityStringToEnums(const std::string& intrinsic, EINTRINSIC & intrinsicType, EDISTORTION & distortionType, EUNDISTORTION & undistortionType)
{
    std::string type = intrinsic;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);  // tolower

    if (type == "pinhole")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_NONE;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "radial1")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_RADIALK1;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "radial3")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_RADIALK3;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "3deradial4")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_3DERADIAL4;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "brown")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_BROWN;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "fisheye4")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_FISHEYE;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "fisheye1")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_FISHEYE1;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "3deanamorphic4")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_NONE;
        undistortionType = EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4;
    }
    else if (type == "3declassicld")
    {
        intrinsicType = EINTRINSIC::PINHOLE_CAMERA;
        distortionType = EDISTORTION::DISTORTION_3DECLASSICLD;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "equidistant")
    {
        intrinsicType = EINTRINSIC::EQUIDISTANT_CAMERA;
        distortionType = EDISTORTION::DISTORTION_NONE;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else if (type == "equidistant_r3")
    {
        intrinsicType = EINTRINSIC::EQUIDISTANT_CAMERA;
        distortionType = EDISTORTION::DISTORTION_RADIALK3PT;
        undistortionType = EUNDISTORTION::UNDISTORTION_NONE;
    }
    else 
    {
        throw std::out_of_range(intrinsic);
    }
}

}  // namespace camera
}  // namespace aliceVision
