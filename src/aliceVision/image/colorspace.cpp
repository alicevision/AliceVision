// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorspace.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <OpenImageIO/color.h>

#include <boost/algorithm/string.hpp>

#include <filesystem>
#include <vector>

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
        if (utils::exists(configOCIOFilePath))
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
            if (utils::exists(configOCIOFilePath))
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
        if (utils::exists(configFromSource))
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

    if (!utils::exists(configOCIOFilePath))
    {
        const std::string configFromSource = getColorConfigFilePathFromSourceCode();
        if (utils::exists(configFromSource))
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
           " (ODT.Academy.Rec709_100nits), " + EImageColorSpace_enumToString(EImageColorSpace::Linear_ARRI_Wide_Gamut_3) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::ARRI_LogC3_EI800) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Linear_ARRI_Wide_Gamut_4) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::ARRI_LogC4) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Linear_BMD_WideGamut_Gen5) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::BMDFilm_WideGamut_Gen5) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::CanonLog2_CinemaGamut_D55) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::CanonLog3_CinemaGamut_D55) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Linear_CinemaGamut_D55) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Linear_V_Gamut) + ", " + EImageColorSpace_enumToString(EImageColorSpace::V_Log_V_Gamut) +
           ", " + EImageColorSpace_enumToString(EImageColorSpace::Linear_REDWideGamutRGB) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Log3G10_REDWideGamutRGB) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::Linear_Venice_S_Gamut3_Cine) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::S_Log3_Venice_S_Gamut3_Cine) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::LAB) + ", " + EImageColorSpace_enumToString(EImageColorSpace::XYZ) + ", " +
           EImageColorSpace_enumToString(EImageColorSpace::NO_CONVERSION);
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
    if ((type == "linear arri wide gamut 3") || (type == "lineararriwidegamut3") || (type == "lin_arri_wide_gamut_3") ||
        (type == "input - arri - linear - alexa wide gamut") || (type == "lin_alexawide"))
        return EImageColorSpace::Linear_ARRI_Wide_Gamut_3;
    if ((type == "arri logc3 (ei800)") || (type == "arrilogc3(ei800)") || (type == "arri_logc3_ei800") ||
        (type == "input - arri - v3 logc(ei800) - wide gamut") || (type == "logc3ei800_alexawide"))
        return EImageColorSpace::ARRI_LogC3_EI800;
    if ((type == "linear arri wide gamut 4") || (type == "lineararriwidegamut4") || (type == "lin_arri_wide_gamut_4") || (type == "lin_awg4"))
        return EImageColorSpace::Linear_ARRI_Wide_Gamut_4;
    if ((type == "arri logc4") || (type == "arrilogc4") || (type == "arri_logc4"))
        return EImageColorSpace::ARRI_LogC4;
    if ((type == "linear bmd widegamut gen5") || (type == "linearbmdwidegamutgen5") || (type == "lin_bmd_widegamut_gen5"))
        return EImageColorSpace::Linear_BMD_WideGamut_Gen5;
    if ((type == "bmdfilm widegamut gen5") || (type == "bmdfilm_widegamut_gen5"))
        return EImageColorSpace::BMDFilm_WideGamut_Gen5;
    if ((type == "canonlog2 cinemagamut d55") || (type == "canonlog2cinemagamutd55") || (type == "canonlog2_cinemagamut_d55") ||
        (type == "input - canon - canon - log2 - cinema gamut daylight") || (type == "canonlog2_cgamutday"))
        return EImageColorSpace::CanonLog2_CinemaGamut_D55;
    if ((type == "canonlog3 cinemagamut d55") || (type == "canonlog3cinemagamutd55") || (type == "canonlog3_cinemagamut_d55") ||
        (type == "input - canon - canon - log3 - cinema gamut daylight") || (type == "canonlog3_cgamutday"))
        return EImageColorSpace::CanonLog3_CinemaGamut_D55;
    if ((type == "linear cinemagamut d55") || (type == "linearcinemagamutd55") || (type == "lin_cinemagamut_d55") ||
        (type == "input - canon - linear - canon cinema gamut daylight") || (type == "lin_canoncgamutday"))
        return EImageColorSpace::Linear_CinemaGamut_D55;
    if ((type == "linear v-gamut") || (type == "linearv-gamut") || (type == "lin_vgamut") || (type == "input - panasonic - linear - v - gamut"))
        return EImageColorSpace::Linear_V_Gamut;
    if ((type == "v-log v-gamut") || (type == "v-logv-gamut") || (type == "vlog_vgamut") || (type == "input - panasonic - v - log - v - gamut"))
        return EImageColorSpace::V_Log_V_Gamut;
    if ((type == "linear redwidegamutrgb") || (type == "linearredwidegamutrgb") || (type == "lin_redwidegamutrgb") ||
        (type == "input - red - linear - redwidegamutrgb") || (type == "lin_rwg"))
        return EImageColorSpace::Linear_REDWideGamutRGB;
    if ((type == "log3g10 redwidegamutrgb") || (type == "log3g10redwidegamutrgb") || (type == "log3g10_redwidegamutrgb") ||
        (type == "input - red - redlog3g10 - redwidegamutrgb") || (type == "rl3g10_rwg"))
        return EImageColorSpace::Log3G10_REDWideGamutRGB;
    if ((type == "linear venice s-gamut3.cine") || (type == "linearvenices-gamut3.cine") || (type == "lin_venice_sgamut3cine") ||
        (type == "input - sony - linear - venice s-gamut3.cine"))
        return EImageColorSpace::Linear_Venice_S_Gamut3_Cine;
    if ((type == "s-log3 venice s-gamut3.cine") || (type == "s-log3venices-gamut3.cine") || (type == "slog3_venice_sgamut3cine") ||
        (type == "input - sony - s-log3 - venice s-gamut3.cine") || (type == "slog3_venice_sgamutcine"))
        return EImageColorSpace::S_Log3_Venice_S_Gamut3_Cine;
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
    const std::map<EImageColorSpace, std::string> enumToString = {{EImageColorSpace::AUTO, "auto"},
                                                                  {EImageColorSpace::LINEAR, "linear"},
                                                                  {EImageColorSpace::SRGB, "srgb"},
                                                                  {EImageColorSpace::ACES2065_1, "aces2065-1"},
                                                                  {EImageColorSpace::ACEScg, "acescg"},
                                                                  {EImageColorSpace::REC709, "rec709"},
                                                                  {EImageColorSpace::Linear_ARRI_Wide_Gamut_3, "Linear ARRI Wide Gamut 3"},
                                                                  {EImageColorSpace::ARRI_LogC3_EI800, "ARRI LogC3 (EI800)"},
                                                                  {EImageColorSpace::Linear_ARRI_Wide_Gamut_4, "Linear ARRI Wide Gamut 4"},
                                                                  {EImageColorSpace::ARRI_LogC4, "ARRI LogC4"},
                                                                  {EImageColorSpace::Linear_BMD_WideGamut_Gen5, "Linear BMD WideGamut Gen5"},
                                                                  {EImageColorSpace::BMDFilm_WideGamut_Gen5, "BMDFilm WideGamut Gen5"},
                                                                  {EImageColorSpace::CanonLog2_CinemaGamut_D55, "CanonLog2 CinemaGamut D55"},
                                                                  {EImageColorSpace::CanonLog3_CinemaGamut_D55, "CanonLog3 CinemaGamut D55"},
                                                                  {EImageColorSpace::Linear_CinemaGamut_D55, "Linear CinemaGamut D55"},
                                                                  {EImageColorSpace::Linear_V_Gamut, "Linear V-Gamut"},
                                                                  {EImageColorSpace::V_Log_V_Gamut, "V-Log V-Gamut"},
                                                                  {EImageColorSpace::Linear_REDWideGamutRGB, "Linear REDWideGamutRGB"},
                                                                  {EImageColorSpace::Log3G10_REDWideGamutRGB, "Log3G10 REDWideGamutRGB"},
                                                                  {EImageColorSpace::Linear_Venice_S_Gamut3_Cine, "Linear Venice S-Gamut3.Cine"},
                                                                  {EImageColorSpace::S_Log3_Venice_S_Gamut3_Cine, "S-Log3 Venice S-Gamut3.Cine"},
                                                                  {EImageColorSpace::LAB, "lab"},
                                                                  {EImageColorSpace::XYZ, "xyz"},
                                                                  {EImageColorSpace::NO_CONVERSION, "no_conversion"}};

    if (enumToString.find(dataType) != enumToString.end())
    {
        return enumToString.at(dataType);
    }
    else
    {
        throw std::out_of_range("Invalid EImageColorSpace enum");
    }
}

std::string EImageColorSpace_enumToOIIOString(const EImageColorSpace colorSpace)
{
    const std::map<EImageColorSpace, std::string> enumToString = {{EImageColorSpace::AUTO, "auto"},
                                                                  {EImageColorSpace::LINEAR, "linear"},
                                                                  {EImageColorSpace::SRGB, "srgb"},
                                                                  {EImageColorSpace::ACES2065_1, "aces2065-1"},
                                                                  {EImageColorSpace::ACEScg, "acescg"},
                                                                  {EImageColorSpace::REC709, "rec709"},
                                                                  {EImageColorSpace::Linear_ARRI_Wide_Gamut_3, "Linear ARRI Wide Gamut 3"},
                                                                  {EImageColorSpace::ARRI_LogC3_EI800, "ARRI LogC3 (EI800)"},
                                                                  {EImageColorSpace::Linear_ARRI_Wide_Gamut_4, "Linear ARRI Wide Gamut 4"},
                                                                  {EImageColorSpace::ARRI_LogC4, "ARRI LogC4"},
                                                                  {EImageColorSpace::Linear_BMD_WideGamut_Gen5, "Linear BMD WideGamut Gen5"},
                                                                  {EImageColorSpace::BMDFilm_WideGamut_Gen5, "BMDFilm WideGamut Gen5"},
                                                                  {EImageColorSpace::CanonLog2_CinemaGamut_D55, "CanonLog2 CinemaGamut D55"},
                                                                  {EImageColorSpace::CanonLog3_CinemaGamut_D55, "CanonLog3 CinemaGamut D55"},
                                                                  {EImageColorSpace::Linear_CinemaGamut_D55, "Linear CinemaGamut D55"},
                                                                  {EImageColorSpace::Linear_V_Gamut, "Linear V-Gamut"},
                                                                  {EImageColorSpace::V_Log_V_Gamut, "V-Log V-Gamut"},
                                                                  {EImageColorSpace::Linear_REDWideGamutRGB, "Linear REDWideGamutRGB"},
                                                                  {EImageColorSpace::Log3G10_REDWideGamutRGB, "Log3G10 REDWideGamutRGB"},
                                                                  {EImageColorSpace::Linear_Venice_S_Gamut3_Cine, "Linear Venice S-Gamut3.Cine"},
                                                                  {EImageColorSpace::S_Log3_Venice_S_Gamut3_Cine, "S-Log3 Venice S-Gamut3.Cine"},
                                                                  {EImageColorSpace::NO_CONVERSION, "no_conversion"}};

    if (enumToString.find(colorSpace) != enumToString.end())
    {
        return enumToString.at(colorSpace);
    }
    else
    {
        throw std::out_of_range("Invalid OIIO EImageColorSpace enum");
    }
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
    if ((colorspace == "linear arri wide gamut 3") || (colorspace == "lin_arri_wide_gamut_3") ||
        (colorspace == "input - arri - linear - alexa wide gamut") || (colorspace == "lin_alexawide"))
        return EImageColorSpace::Linear_ARRI_Wide_Gamut_3;
    if ((colorspace == "arri logc3 (ei800)") || (colorspace == "arri_logc3_ei800") || (colorspace == "input - arri - v3 logc(ei800) - wide gamut") ||
        (colorspace == "logc3ei800_alexawide"))
        return EImageColorSpace::ARRI_LogC3_EI800;
    if ((colorspace == "linear arri wide gamut 4") || (colorspace == "lin_arri_wide_gamut_4") || (colorspace == "lin_awg4"))
        return EImageColorSpace::Linear_ARRI_Wide_Gamut_4;
    if ((colorspace == "arri logc4") || (colorspace == "arri_logc4"))
        return EImageColorSpace::ARRI_LogC4;
    if ((colorspace == "linear bmd widegamut gen5") || (colorspace == "lin_bmd_widegamut_gen5"))
        return EImageColorSpace::Linear_BMD_WideGamut_Gen5;
    if ((colorspace == "bmdfilm widegamut gen5") || (colorspace == "bmdfilm_widegamut_gen5"))
        return EImageColorSpace::BMDFilm_WideGamut_Gen5;
    if ((colorspace == "canonlog2 cinemagamut d55") || (colorspace == "canonlog2_cinemagamut_d55") ||
        (colorspace == "input - canon - canon - log2 - cinema gamut daylight") || (colorspace == "canonlog2_cgamutday"))
        return EImageColorSpace::CanonLog2_CinemaGamut_D55;
    if ((colorspace == "canonlog3 cinemagamut d55") || (colorspace == "canonlog3_cinemagamut_d55") ||
        (colorspace == "input - canon - canon - log3 - cinema gamut daylight") || (colorspace == "canonlog3_cgamutday"))
        return EImageColorSpace::CanonLog3_CinemaGamut_D55;
    if ((colorspace == "linear cinemagamut d55") || (colorspace == "lin_cinemagamut_d55") ||
        (colorspace == "input - canon - linear - canon cinema gamut daylight") || (colorspace == "lin_canoncgamutday"))
        return EImageColorSpace::Linear_CinemaGamut_D55;
    if ((colorspace == "linear v-gamut") || (colorspace == "lin_vgamut") || (colorspace == "input - panasonic - linear - v - gamut"))
        return EImageColorSpace::Linear_V_Gamut;
    if ((colorspace == "v-log v-gamut") || (colorspace == "vlog_vgamut") || (colorspace == "input - panasonic - v - log - v - gamut"))
        return EImageColorSpace::V_Log_V_Gamut;
    if ((colorspace == "linear redwidegamutrgb") || (colorspace == "lin_redwidegamutrgb") ||
        (colorspace == "input - red - linear - redwidegamutrgb") || (colorspace == "lin_rwg"))
        return EImageColorSpace::Linear_REDWideGamutRGB;
    if ((colorspace == "log3g10 redwidegamutrgb") || (colorspace == "log3g10_redwidegamutrgb") ||
        (colorspace == "input - red - redlog3g10 - redwidegamutrgb") || (colorspace == "rl3g10_rwg"))
        return EImageColorSpace::Log3G10_REDWideGamutRGB;
    if ((colorspace == "linear venice s-gamut3.cine") || (colorspace == "lin_venice_sgamut3cine") ||
        (colorspace == "input - sony - linear - venice s-gamut3.cine"))
        return EImageColorSpace::Linear_Venice_S_Gamut3_Cine;
    if ((colorspace == "s-log3 venice s-gamut3.cine") || (colorspace == "slog3_venice_sgamut3cine") ||
        (colorspace == "input - sony - s-log3 - venice s-gamut3.cine") || (colorspace == "slog3_venice_sgamutcine"))
        return EImageColorSpace::S_Log3_Venice_S_Gamut3_Cine;

    throw std::out_of_range("No EImageColorSpace defined for string: " + colorspace);
}

bool EImageColorSpace_isSupportedOIIOEnum(const EImageColorSpace& colorspace)
{
    switch (colorspace)
    {
        case EImageColorSpace::LAB:
            return false;
        case EImageColorSpace::XYZ:
            return false;
        case EImageColorSpace::NO_CONVERSION:
            return false;
        default:
            return true;
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
    std::string token(std::istreambuf_iterator<char>(in), {});
    dataType = EImageColorSpace_stringToEnum(token);
    return in;
}

}  // namespace image
}  // namespace aliceVision
