// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <ostream>

namespace aliceVision {

class Point4d
{
public:
    union {
        struct
        {
            float x, y, z, w;
        };
        float m[4];
    };

    Point4d()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        w = 0.0;
    }

    Point4d(const float _x, const float _y, const float _z, const float _w)
    {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }

    inline Point4d& operator=(const Point4d& param)
    {
        x = param.x;
        y = param.y;
        z = param.z;
        w = param.w;
        return *this;
    }

    inline bool operator==(const Point4d& param) const
    {
        return (x == param.x) && (y == param.y) && (z == param.z) && (w == param.w);
    }

    inline Point4d operator-(const Point4d& _p) const
    {
        return Point4d(x - _p.x, y - _p.y, z - _p.z, w - _p.w);
    }

    inline Point4d operator-() const
    {
        return Point4d(-x, -y, -z, -w);
    }

    inline Point4d operator+(const Point4d& _p) const
    {
        return Point4d(x + _p.x, y + _p.y, z + _p.z, w + _p.w);
    }

    inline Point4d operator*(const float d) const
    {
        return Point4d(x * d, y * d, z * d, w * d);
    }

    inline Point4d operator/(const float d) const
    {
        return Point4d(x / d, y / d, z / d, w / d);
    }

    inline Point4d normalize() const
    {
        float d = sqrt(x * x + y * y + z * z + w * w);
        return Point4d(x / d, y / d, z / d, w / d);
    }

    inline float size() const
    {
        float d = x * x + y * y + z * z + w * w;
        if(d == 0.0f)
            return 0.0f;
        return sqrt(d);
    }

    friend float dot(const Point4d& p1, const Point4d& p2)
    {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z + p1.w * p2.w;
    }

    friend Point4d proj(Point4d& e, Point4d& a)
    {
        return e * (dot(e, a) / dot(e, e));
    }
};

inline std::ostream& operator<<(std::ostream& stream, const Point4d& p)
{
    stream << p.x << "," << p.y << "," << p.z << "," << p.w;
    return stream;
}

} // namespace aliceVision
