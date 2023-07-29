// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision
{

namespace SphericalMapping
{
/**
 * Map from equirectangular to spherical coordinates
 * @param equirectangular equirectangular coordinates
 * @param width number of pixels used to represent longitude
 * @param height number of pixels used to represent latitude
 * @return spherical coordinates
 */
Vec3 fromEquirectangular(const Vec2& equirectangular, int width, int height);

/**
 * Map from Spherical to equirectangular coordinates
 * @param spherical spherical coordinates
 * @param width number of pixels used to represent longitude
 * @param height number of pixels used to represent latitude
 * @return equirectangular coordinates
 */
Vec2 toEquirectangular(const Vec3& spherical, int width, int height);

} // namespace SphericalMapping
} // namespace aliceVision