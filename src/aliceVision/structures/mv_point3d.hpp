// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class point3d
{
public:
    union {
        struct
        {
            double x, y, z;
        };
        double m[3];
    };

    inline point3d()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    inline point3d(double _x, double _y, double _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    inline point3d& operator=(const point3d& param)
    {
        x = param.x;
        y = param.y;
        z = param.z;
        return *this;
    }

    inline bool operator==(const point3d& param)
    {
        return (x == param.x) && (y == param.y) && (z == param.z);
    }

    inline point3d operator-(const point3d& _p) const
    {
        return point3d(x - _p.x, y - _p.y, z - _p.z);
    }

    inline point3d operator-() const
    {
        return point3d(-x, -y, -z);
    }

    inline point3d operator+(const point3d& _p) const
    {
        return point3d(x + _p.x, y + _p.y, z + _p.z);
    }

    inline point3d operator*(const double d) const
    {
        return point3d(x * d, y * d, z * d);
    }

    inline point3d operator/(const double d) const
    {
        return point3d(x / d, y / d, z / d);
    }

    inline point3d normalize() const
    {
        double d = std::sqrt(x * x + y * y + z * z);
        return point3d(x / d, y / d, z / d);
    }

    inline double size() const
    {
        double d = x * x + y * y + z * z;
        if(d == 0.0)
        {
            return 0.0;
        }

        return sqrt(d);
    }

    inline double size2() const
    {
        return x * x + y * y + z * z;
    }

    void doprintf() const;
    void saveToFile(const std::string& fileName) const;
    void loadFromFile(const std::string& fileName);

    friend double dot(const point3d& p1, const point3d& p2);
    friend point3d cross(const point3d& a, const point3d& b);
    friend point3d proj(point3d& e, point3d& a);
};

inline double dot(const point3d& p1, const point3d& p2)
{
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

inline point3d cross(const point3d& a, const point3d& b)
{
    point3d vc;
    vc.x = a.y * b.z - a.z * b.y;
    vc.y = a.z * b.x - a.x * b.z;
    vc.z = a.x * b.y - a.y * b.x;

    return vc;
}

inline point3d proj(point3d& e, point3d& a)
{
    return e * (dot(e, a) / dot(e, e));
}
