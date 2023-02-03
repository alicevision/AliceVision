// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DistortionBrown.hpp"

#include <vector>

namespace aliceVision {
namespace camera {

Vec2 DistortionBrown::distoFunction(const std::vector<double>& params, const Vec2& p)
{
    const double& k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
    const double r2 = p(0) * p(0) + p(1) * p(1);
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
    const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
    const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
    Vec2 d(p(0) * k_diff + t_x, p(1) * k_diff + t_y);

    return d;
}

Vec2 DistortionBrown::addDistortion(const Vec2& p) const
{
    return (p + distoFunction(_distortionParams, p));
}

Vec2 DistortionBrown::removeDistortion(const Vec2& p) const
{
    const double epsilon = 1e-8;  // criteria to stop the iteration
    Vec2 p_u = p;

    while ((addDistortion(p_u) - p).lpNorm<1>() > epsilon)  // manhattan distance between the two points
    {
        p_u = p - distoFunction(_distortionParams, p_u);
    }

    return p_u;
}

} // namespace camera
} // namespace aliceVision
