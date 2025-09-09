// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/image/pixelTypes.hpp>
#include <Vec_generated.h>

namespace aliceVision {
namespace sfmDataIO {

/**
 * @brief Convert a vec2 to a flatbuffers vec2
 * @param obj an input vec2
 * @return a flatbuffer object containing the vec2
*/
AliceVisionIO::Vec2 Pack(const Vec2 & obj)
{
    return AliceVisionIO::Vec2(obj.x(), obj.y());
}

/**
 * @brief Convert a vec3 to a flatbuffers vec3
 * @param obj an input vec3
 * @return a flatbuffer object containing the vec3
*/
AliceVisionIO::Vec3 Pack(const Vec3 & obj)
{
    return AliceVisionIO::Vec3(obj.x(), obj.y(), obj.z());
}

/**
 * @brief Convert a flatbuffers vec3 to a vec3
 * @param obj an input flatbuffers vec3
 * @return a vec3
*/
Vec3 Unpack(const AliceVisionIO::Vec3 & obj)
{
    return Vec3(obj.x(), obj.y(), obj.z());
}

/**
 * @brief Convert a flatbuffers vec2 to a vec2
 * @param obj an input flatbuffers vec2
 * @return a vec2
*/
Vec2 Unpack(const AliceVisionIO::Vec2 & obj)
{
    return Vec2(obj.x(), obj.y());
}

}
}