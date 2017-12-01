// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class point2d
{
public:
    union {
        struct
        {
            double x, y;
        };
        double m[2];
    };

    inline point2d()
    {
        x = 0.0;
        y = 0.0;
    }

    inline point2d(const double _x, const double _y)
    {
        x = _x;
        y = _y;
    }

    inline point2d(const int _x, const int _y)
    {
        x = (double)_x;
        y = (double)_y;
    }

    inline point2d& operator=(const point2d& param)
    {
        x = param.x;
        y = param.y;
        return *this;
    }

    inline point2d operator-(const point2d& _p) const
    {
        return point2d(x - _p.x, y - _p.y);
    }

    inline point2d operator+(const point2d& _p) const
    {
        return point2d(x + _p.x, y + _p.y);
    }

    inline point2d operator*(const double d) const
    {
        return point2d(x * d, y * d);
    }

    inline point2d operator+(const double d) const
    {
        return point2d(x + d, y + d);
    }

    inline point2d operator/(const double d) const
    {
        return point2d(x / d, y / d);
    }

    inline point2d normalize() const
    {
        double d = std::sqrt(x * x + y * y);
        return point2d(x / d, y / d);
    }

    inline double size() const
    {
        return std::sqrt(x * x + y * y);
    }

    friend double dot(const point2d& p1, const point2d& p2);
};

inline double dot(const point2d& p1, const point2d& p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}
