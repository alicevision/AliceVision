// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point2d.hpp>

namespace aliceVision {

struct Pixel
{
    union {
        struct
        {
            int x, y;
        };
        int m[2];
    };

    Pixel()
    {
        x = 0;
        y = 0;
    }

    explicit Pixel(const Point2d& p)
    {

        x = (int)floor(p.x + 0.5);
        y = (int)floor(p.y + 0.5);
    }

    Pixel(const int _x, const int _y)
    {
        x = _x;
        y = _y;
    }

    inline int& operator[](const int index)
    {
        return m[index];
    }

    inline Pixel& operator=(const Pixel& param)
    {
        x = param.x;
        y = param.y;
        return *this;
    }

    inline bool operator==(const Pixel& param)
    {
        return ((x == param.x) && (y == param.y));
    }

    inline Pixel operator-(const Pixel& _p)
    {
        return Pixel(x - _p.x, y - _p.y);
    }

    inline Pixel operator+(const Pixel& _p)
    {
        return Pixel(x + _p.x, y + _p.y);
    }

    inline Pixel operator*(const int& d)
    {
        return Pixel(x * d, y * d);
    }

    inline Pixel operator/(int d)
    {
        if(d == 0)
        {
            return Pixel(0, 0);
        }
        else
        {
            Pixel p;
            p.x = (int)floor((float)x / (float)d + 0.5);
            p.y = (int)floor((float)y / (float)d + 0.5);
            return p;
        };
    }

    inline double size()
    {
        return sqrt((double)(x * x + y * y));
    }
    inline int size2()
    {
        return x * x + y * y;
    }

    friend int dot(const Pixel& p1, const Pixel& p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
    }
};

} // namespace aliceVision
