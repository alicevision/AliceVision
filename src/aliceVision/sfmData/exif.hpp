
// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace sfmData {

/**
 * @brief EXIF Orientation to names
 * https://jdhao.github.io/2019/07/31/image_rotation_exif_info/
 */
enum class EEXIFOrientation : std::int8_t
{
    NONE = 1,
    REVERSED = 2,
    UPSIDEDOWN = 3,
    UPSIDEDOWN_REVERSED = 4,
    LEFT_REVERSED = 5,
    LEFT = 6,
    RIGHT_REVERSED = 7,
    RIGHT = 8,
    UNKNOWN = -1
};

struct GPSExifTags
{
    static std::string latitude();
    static std::string latitudeRef();
    static std::string longitude();
    static std::string longitudeRef();
    static std::string altitude();
    static std::string altitudeRef();
    static std::vector<std::string> all();
};

}  // namespace sfmData
}  // namespace aliceVision