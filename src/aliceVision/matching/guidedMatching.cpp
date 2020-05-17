// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "guidedMatching.hpp"

namespace aliceVision {
namespace matching {

unsigned int pix_to_bucket(const Vec2i& x, int W, int H)
{
    if(x(1) == 0)
        return x(0); // Top border
    if(x(0) == W - 1)
        return W - 1 + x(1); // Right border
    if(x(1) == H - 1)
        return 2 * W + H - 3 - x(0); // Bottom border
    return 2 * (W + H - 2) - x(1);   // Left border
}

bool line_to_endPoints(const Vec3& line, int W, int H, Vec2& x0, Vec2& x1)
{
    const double a = line(0);
    const double b = line(1);
    const double c = line(2);

    float r1, r2;
    // Intersection with Y axis (0 or W-1)
    if(b != 0)
    {
        const double x = (b < 0) ? 0 : W - 1;
        double y = -(a * x + c) / b;
        if(y < 0)
            y = 0.;
        else if(y >= H)
            y = H - 1;
        r1 = fabs(a * x + b * y + c);
        x0 << x, y;
    }
    else
    {
        return false;
    }

    // Intersection with X axis (0 or H-1)
    if(a != 0)
    {
        const double y = (a < 0) ? H - 1 : 0;
        double x = -(b * y + c) / a;
        if(x < 0)
            x = 0.;
        else if(x >= W)
            x = W - 1;
        r2 = fabs(a * x + b * y + c);
        x1 << x, y;
    }
    else
    {
        return false;
    }

    // Choose x0 to be as close as the intersection axis
    if(r1 > r2)
        std::swap(x0, x1);

    return true;
}

}
}