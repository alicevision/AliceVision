// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <istream>
#include <ostream>

#include <OpenImageIO/color.h>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {

/**
 * @brief Available image color space for pipeline input
 */
enum class EImageColorSpace
{
    AUTO,
    LINEAR,
    SRGB,
    ACES2065_1,
    ACEScg,
    REC709,
    LAB,
    XYZ,
    NO_CONVERSION
};

std::string EImageColorSpace_informations();
EImageColorSpace EImageColorSpace_stringToEnum(const std::string& dataType);
std::string EImageColorSpace_enumToString(const EImageColorSpace dataType);
std::string EImageColorSpace_enumToOIIOString(const EImageColorSpace colorSpace);
EImageColorSpace EImageColorSpace_OIIOstringToEnum(const std::string& colorspace);
bool EImageColorSpace_isSupportedOIIOEnum(const EImageColorSpace& colorspace);
bool EImageColorSpace_isSupportedOIIOstring(const std::string& colorspace);
std::ostream& operator<<(std::ostream& os, EImageColorSpace dataType);
std::istream& operator>>(std::istream& in, EImageColorSpace& dataType);

std::string getDefaultColorConfigFilePath();
void initColorConfigOCIO(const std::string& colorConfigFilePath);
oiio::ColorConfig& getGlobalColorConfigOCIO();

}  // namespace image
}  // namespace aliceVision
