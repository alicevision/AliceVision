// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {

struct rgb
{
    unsigned char r = 0;
    unsigned char g = 0;
    unsigned char b = 0;

    rgb() {}
    rgb(unsigned char _r, unsigned char _g, unsigned char _b)
    {
        r = _r;
        g = _g;
        b = _b;
    }

    inline rgb operator-(const rgb& _p) const
    {
        return rgb(r - _p.r, g - _p.g, b - _p.b);
    }

    inline rgb operator+(const rgb& _p) const
    {
        return rgb(r + _p.r, g + _p.g, b + _p.b);
    }
};

} // namespace aliceVision
