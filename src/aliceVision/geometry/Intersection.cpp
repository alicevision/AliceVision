// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Intersection.hpp"

namespace aliceVision {
namespace geometry {

bool rayIntersectUnitSphere(Vec3& coordinates, const Vec3& start, const Vec3& direction)
{
    /*
    "Which point on the sphere relates to this point ?"
    (lambda * directionx + startx)^2
    + (lambda * directiony + starty)^2
    + (lambda * directionz + startz)^2=1

    (lambda * directionx)^2 + startx^2 + 2.0 * lambda * directionoriginx * directionx
    + (lambda * directiony)^2 + starty^2 + 2.0 * lambda * directionoriginy * directiony
    + (lambda * directionz)^2 + startz^2 + 2.0 * lambda * directionoriginz * directionz
    = 1

    (lambda^2) * (directionx^2 + directiony^2 + directionz^2)
    + lambda * (2.0 * directionoriginx * directionx + 2.0 * directionoriginy * directiony + 2.0 * directionoriginz * directionz)
    + (startx^2 + startx^2 + startx^2) - 1 = 0
    */

    double a = direction.dot(direction);
    double b = direction.dot(start) * 2.0;
    double c = start.dot(start) - 1.0;
    double det = b * b - 4.0 * a * c;

    if (det < 0)
    {
        return false;
    }

    double x1 = (-b + sqrt(det)) / (2.0 * a);
    double x2 = (-b - sqrt(det)) / (2.0 * a);
    double lambda = std::min(x1, x2);

    coordinates = start + lambda * direction;

    return true;
}

}  // namespace geometry
}  // namespace aliceVision
