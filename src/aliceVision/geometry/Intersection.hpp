// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace geometry {

/**
 * @brief find the intersection of a ray (composed of a starting point and a direction) with the unit sphere centered on (0,0,0).
 * @param coordinates the coordinates of the intersection closest to the starting point
 * @param start the starting point of the ray
 * @param direction the direction of the ray
 * @return true if an intersection is found
 */
bool rayIntersectUnitSphere(Vec3& coordinates, const Vec3& start, const Vec3& direction);

}  // namespace geometry
}  // namespace aliceVision
