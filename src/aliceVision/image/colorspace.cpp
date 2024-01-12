// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorspace.hpp"

#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/color.h>

#include <boost/algorithm/string.hpp>

#include <filesystem>

namespace aliceVision {
namespace image {

namespace {
oiio::ColorConfig colorConfigOCIO(getDefaultColorConfigFilePath());
}

namespace fs = std::filesystem;

oiio::ColorConfig& getGlobalColorConfigOCIO() { return colorConfigOCIO; }

std::string getColorConfigFilePathFromSourceCode()
{
    const fs::path moduleFolder = fs::path(__FILE__).parent_path();
    return (moduleFolder / "share/aliceVision/config.ocio").string();
}

std::string getDefaultColorConfigFilePath()
{
    std::string configOCIOFilePath = "";

    char const* ALICEVISION_OCIO = std::getenv("ALICEVISION_OCIO");
    if (ALICEVISION_OCIO != NULL)
    {
        configOCIOFilePath = std::string(ALICEVISION_OCIO);
        if (fs::exists(configOCIOFilePath))
        {
            // Check if a sRGB linear color space named "scene-linear Rec.709-sRGB" is present and set as scene_linear role
            oiio::ColorConfig colorConfig(configOCIOFilePath);
            const std::string linearColorSpace = colorConfig.getColorSpaceNameByRole("scene_linear");
            if (linearColorSpace == "scene-linear Rec.709-sRGB")
            {
                ALICEVISION_LOG_TRACE("ALICEVISION_OCIO configuration file: '" << configOCIOFilePath << "' found.");
                return configOCIOFilePath;
            }
            else
            {
                ALICEVISION_LOG_WARNING("ALICEVISION_OCIO configuration file is not valid: '"
                                        << configOCIOFilePath
                                        << "'.\n"
                                           "The scene_linear role named \"scene-linear Rec.709-sRGB\" is required.\n"
                                           "Skip this OCIO configuration file and use the embedded one.");
            }
        }
        else if (configOCIOFilePath.empty())
        {
            ALICEVISION_LOG_TRACE("ALICEVISION_OCIO is empty.");
        }
        else
        {
            ALICEVISION_LOG_WARNING("ALICEVISION_OCIO is defined but does not point to an existing file: '" << configOCIOFilePath << "'");
        }
    }

    // To be enabled if we decide to take OCIO env var in consideration before using the enbedded config file
    if (false)
    {
        char const* OCIO = std::getenv("OCIO");
        if (OCIO != NULL)
        {
            configOCIOFilePath = std::string(OCIO);
            if (fs::exists(configOCIOFilePath))
            {
                ALICEVISION_LOG_TRACE("OCIO configuration file: '" << configOCIOFilePath << "' found.");
                return configOCIOFilePath;
            }
            else if (configOCIOFilePath == "")
            {
                ALICEVISION_LOG_TRACE("OCIO is empty. Use embedded config...");
            }
            else
            {
                ALICEVISION_LOG_TRACE("OCIO does not point to an existing file. Use embedded config...");
            }
        }
    }

    char const* ALICEVISION_ROOT = std::getenv("ALICEVISION_ROOT");
    if (ALICEVISION_ROOT == NULL)
    {
        const std::string configFromSource = getColorConfigFilePathFromSourceCode();
        if (fs::exists(configFromSource))
        {
            ALICEVISION_LOG_DEBUG("ALICEVISION_ROOT is not defined, use embedded OCIO config file from source code: " << configFromSource);
            return configFromSource;
        }
        // Output message with logging before throw as this function could be called before main.
        ALICEVISION_LOG_ERROR("ALICEVISION_ROOT is not defined, embedded OCIO config file cannot be accessed.");
        ALICEVISION_THROW_ERROR("ALICEVISION_ROOT is not defined, embedded OCIO config file cannot be accessed.");
    }
    configOCIOFilePath = std::string(ALICEVISION_ROOT);
    configOCIOFilePath.append("/share/aliceVision/config.ocio");

    if (!fs::exists(configOCIOFilePath))
    {
        const std::string configFromSource = getColorConfigFilePathFromSourceCode();
        if (fs::exists(configFromSource))
        {
            ALICEVISION_LOG_DEBUG("Embedded OCIO config file in ALICEVISION_ROOT does not exist, use config from source code: " << configFromSource);
            return configFromSource;
        }
        ALICEVISION_LOG_ERROR("Embedded OCIO configuration file: '" << configOCIOFilePath << "' cannot be accessed.");
        ALICEVISION_THROW_ERROR("Embedded OCIO configuration file: '" << configOCIOFilePath << "' cannot be accessed.");
    }
    ALICEVISION_LOG_TRACE("Embedded OCIO configuration file: '" << configOCIOFilePath << "' found.");

    return configOCIOFilePath;
}

void initColorConfigOCIO(const std::string& colorConfigFilePath)
{
    colorConfigOCIO.reset(colorConfigFilePath);
    if (!colorConfigOCIO.supportsOpenColorIO())
    {
        ALICEVISION_THROW_ERROR("OpenImageIO has not been compiled with OCIO.");
    }
    const std::string error = colorConfigOCIO.geterror();
    if (!error.empty())
    {
        ALICEVISION_THROW_ERROR("Erroneous OCIO config file " << colorConfigFilePath << ":" << std::endl << error);
    }
    int ocioVersion = colorConfigOCIO.OpenColorIO_version_hex();
    int ocioMajor = (ocioVersion & 0xFF000000) >> 24;
    int ocioMinor = (ocioVersion & 0x00FF0000) >> 16;
    int ocioPatch = (ocioVersion & 0x0000FF00) >> 8;
    ALICEVISION_LOG_INFO("OCIO color config initialized with OCIO version: " << ocioMajor << "." << ocioMinor << "." << ocioPatch);
}

std::string EImageColorSpace_informations()
{
    return EImageColorSpace_enumToString(EImageColorSpace::AUTO) + ", " + EImageColorSpace_enumToString(EImageColorSpace::LINEAR) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::SRGB) + ", " + EImageColorSpace_enumToString(EImageColorSpace::ACES2065_1) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::ACEScg) + ", " + EImageColorSpace_enumToString(EImageColorSpace::REC709) +
           " (ODT.Academy.Rec709_100nits), " + EImageColorSpace_enumToString(EImageColorSpace::LAB) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::XYZ) + ", " + EImageColorSpace_enumToString(EImageColorSpace::NO_CONVERSION);
}

EImageColorSpace EImageColorSpace_stringToEnum(const std::string& dataType)
{
    const std::string type = boost::to_lower_copy(dataType);

    if (type == "auto")
        return EImageColorSpace::AUTO;
    if (type == "linear")
        return EImageColorSpace::LINEAR;
    if (type == "srgb")
        return EImageColorSpace::SRGB;
    if (type == "aces2065-1")
        return EImageColorSpace::ACES2065_1;
    if (type == "acescg")
        return EImageColorSpace::ACEScg;
    if ((type == "aces_lut") || (type == "rec709"))
        return EImageColorSpace::REC709;
    if (type == "lab")
        return EImageColorSpace::LAB;
    if (type == "xyz")
        return EImageColorSpace::XYZ;
    if (type == "no_conversion")
        return EImageColorSpace::NO_CONVERSION;

    throw std::out_of_range("Invalid EImageColorSpace: " + dataType);
}

std::string EImageColorSpace_enumToString(const EImageColorSpace dataType)
{
    switch (dataType)
    {
        case EImageColorSpace::AUTO:
            return "auto";
        case EImageColorSpace::LINEAR:
            return "linear";
        case EImageColorSpace::SRGB:
            return "srgb";
        case EImageColorSpace::ACES2065_1:
            return "aces2065-1";
        case EImageColorSpace::ACEScg:
            return "acescg";
        case EImageColorSpace::REC709:
            return "rec709";
        case EImageColorSpace::LAB:
            return "lab";
        case EImageColorSpace::XYZ:
            return "xyz";
        case EImageColorSpace::NO_CONVERSION:
            return "no_conversion";
    }
    throw std::out_of_range("Invalid EImageColorSpace enum");
}

std::string EImageColorSpace_enumToOIIOString(const EImageColorSpace colorSpace)
{
    switch (colorSpace)
    {
        case EImageColorSpace::SRGB:
            return "sRGB";
        case EImageColorSpace::LINEAR:
            return "Linear";
        case EImageColorSpace::ACES2065_1:
            return "aces2065-1";
        case EImageColorSpace::ACEScg:
            return "ACEScg";
        case EImageColorSpace::REC709:
            return "rec709";
        default:;
    }
    throw std::out_of_range("No string defined for EImageColorSpace to OIIO conversion: " + std::to_string(int(colorSpace)));
}

EImageColorSpace EImageColorSpace_OIIOstringToEnum(const std::string& colorspace)
{
    const std::string cs = boost::to_lower_copy(colorspace);

    if (cs == "linear")
        return EImageColorSpace::LINEAR;
    if (cs == "srgb")
        return EImageColorSpace::SRGB;
    if (cs == "aces2065-1")
        return EImageColorSpace::ACES2065_1;
    if (cs == "acescg")
        return EImageColorSpace::ACEScg;
    if ((cs == "rec709") || (colorspace == "aces_lut"))
        return EImageColorSpace::REC709;

    throw std::out_of_range("No EImageColorSpace defined for string: " + colorspace);
}

bool EImageColorSpace_isSupportedOIIOEnum(const EImageColorSpace& colorspace)
{
    switch (colorspace)
    {
        case EImageColorSpace::SRGB:
            return true;
        case EImageColorSpace::LINEAR:
            return true;
        case EImageColorSpace::ACES2065_1:
            return true;
        case EImageColorSpace::ACEScg:
            return true;
        case EImageColorSpace::REC709:
            return true;
        default:
            return false;
    }
}

bool EImageColorSpace_isSupportedOIIOstring(const std::string& colorspace)
{
    const OIIO::ColorConfig& ocioConf = getGlobalColorConfigOCIO();

    std::vector<std::string> knownColorSpaces = {"linear"};

    for (auto cs : ocioConf.getColorSpaceNames())
    {
        knownColorSpaces.push_back(boost::to_lower_copy(cs));
        for (auto alias : ocioConf.getAliases(cs))
        {
            knownColorSpaces.push_back(boost::to_lower_copy(alias));
        }
    }

    return (std::find(knownColorSpaces.begin(), knownColorSpaces.end(), boost::to_lower_copy(colorspace)) != knownColorSpaces.end());
}

std::ostream& operator<<(std::ostream& os, EImageColorSpace dataType) { return os << EImageColorSpace_enumToString(dataType); }

std::istream& operator>>(std::istream& in, EImageColorSpace& dataType)
{
    std::string token;
    in >> token;
    dataType = EImageColorSpace_stringToEnum(token);
    return in;
}

}  // namespace image
}  // namespace aliceVision
