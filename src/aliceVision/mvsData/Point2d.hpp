// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cmath>
#include <ostream>

namespace aliceVision {

class Point2d
{
public:
    union {
        struct
        {
            double x, y;
        };
        double m[2];
    };

    inline Point2d()
    {
        x = 0.0;
        y = 0.0;
    }

    inline Point2d(const double _x, const double _y)
    {
        x = _x;
        y = _y;
    }

    inline Point2d(const int _x, const int _y)
    {
        x = (double)_x;
        y = (double)_y;
    }

    inline Point2d& operator=(const Point2d& param)
    {
        x = param.x;
        y = param.y;
        return *this;
    }

    inline Point2d operator-(const Point2d& _p) const
    {
        return Point2d(x - _p.x, y - _p.y);
    }

    inline Point2d operator+(const Point2d& _p) const
    {
        return Point2d(x + _p.x, y + _p.y);
    }

    inline Point2d operator*(const double d) const
    {
        return Point2d(x * d, y * d);
    }

    inline Point2d operator+(const double d) const
    {
        return Point2d(x + d, y + d);
    }

    inline Point2d operator/(const double d) const
    {
        return Point2d(x / d, y / d);
    }

    inline Point2d normalize() const
    {
        double d = std::sqrt(x * x + y * y);
        return Point2d(x / d, y / d);
    }

    inline double size() const
    {
        return std::sqrt(x * x + y * y);
    }

    friend double dot(const Point2d& p1, const Point2d& p2);
};

inline double dot(const Point2d& p1, const Point2d& p2)
{
    return p1.x * p2.x + p1.y * p2.y;
}

inline std::ostream& operator<<(std::ostream& stream, const Point2d& p)
{
    stream << p.x << "," << p.y;
    return stream;
}

} // namespace aliceVision
