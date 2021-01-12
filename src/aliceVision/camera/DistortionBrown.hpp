// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/Distortion.hpp>

namespace aliceVision {
namespace camera {

class DistortionBrown : public Distortion
{
public:
    DistortionBrown()
    {
        _distortionParams = {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    DistortionBrown(double p1, double p2, double p3, double p4, double p5)
    {
        _distortionParams = {p1, p2, p3, p4, p5};
    }

    DistortionBrown* clone() const override
    {
        return new DistortionBrown(*this);
    }

    /// Add distortion to the point p (assume p is in the camera frame [normalized coordinates])
    Vec2 addDistortion(const Vec2& p) const override
    {
        return (p + distoFunction(_distortionParams, p));
    }

    /// Remove distortion (return p' such that disto(p') = p)
    Vec2 removeDistortion(const Vec2& p) const override
    {
        const double epsilon = 1e-8; // criteria to stop the iteration
        Vec2 p_u = p;

        while((addDistortion(p_u) - p).lpNorm<1>() > epsilon) // manhattan distance between the two points
        {
            p_u = p - distoFunction(_distortionParams, p_u);
        }

        return p_u;
    }

    // Functor to calculate distortion offset accounting for both radial and tangential distortion
    static Vec2 distoFunction(const std::vector<double>& params, const Vec2& p)
    {
        const double k1 = params[0], k2 = params[1], k3 = params[2], t1 = params[3], t2 = params[4];
        const double r2 = p(0) * p(0) + p(1) * p(1);
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double k_diff = (k1 * r2 + k2 * r4 + k3 * r6);
        const double t_x = t2 * (r2 + 2 * p(0) * p(0)) + 2 * t1 * p(0) * p(1);
        const double t_y = t1 * (r2 + 2 * p(1) * p(1)) + 2 * t2 * p(0) * p(1);
        Vec2 d(p(0) * k_diff + t_x, p(1) * k_diff + t_y);

        return d;
    }

    ~DistortionBrown() override = default;
};

} // namespace camera
} // namespace aliceVision
