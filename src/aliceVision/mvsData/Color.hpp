// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <cmath>

namespace aliceVision {

class ColorRGBf
{
public:
    union {
        struct
        {
            float r, g, b;
        };
        float m[3];
    };

    inline ColorRGBf()
    {
        r = 0.0;
        g = 0.0;
        b = 0.0;
    }

    inline ColorRGBf(float _r, float _g, float _b)
    {
        r = _r;
        g = _g;
        b = _b;
    }

    inline ColorRGBf& operator=(const ColorRGBf& param)
    {
        r = param.r;
        g = param.g;
        b = param.b;
        return *this;
    }

    inline bool operator==(const ColorRGBf& param)
    {
        return (r == param.r) && (g == param.g) && (b == param.b);
    }

    inline ColorRGBf operator-(const ColorRGBf& _p) const
    {
        return ColorRGBf(r - _p.r, g - _p.g, b - _p.b);
    }

    inline ColorRGBf operator-() const
    {
        return ColorRGBf(-r, -g, -b);
    }

    inline ColorRGBf operator+(const ColorRGBf& _p) const
    {
        return ColorRGBf(r + _p.r, g + _p.g, b + _p.b);
    }

    inline ColorRGBf operator*(const float d) const
    {
        return ColorRGBf(r * d, g * d, b * d);
    }

    inline ColorRGBf operator/(const float d) const
    {
        return ColorRGBf(r / d, g / d, b / d);
    }

    inline ColorRGBf& operator+=(const ColorRGBf& _p)
    {
        r += _p.r;
        g += _p.g;
        b += _p.b;
        return *this;
    }

    inline ColorRGBf& operator-=(const ColorRGBf& _p)
    {
        r -= _p.r;
        g -= _p.g;
        b -= _p.b;
        return *this;
    }

    inline ColorRGBf& operator/=(const int d)
    {
        r /= d;
        g /= d;
        b /= d;
        return *this;
    }

    inline ColorRGBf normalize() const
    {
        float d = std::sqrt(r * r + g * g + b * b);
        return ColorRGBf(r / d, g / d, b / d);
    }

    inline float size() const
    {
        float d = r * r + g * g + b * b;
        if(d == 0.0)
        {
            return 0.0;
        }

        return sqrt(d);
    }

    inline float size2() const
    {
        return r * r + g * g + b * b;
    }

    friend float dot(const ColorRGBf& p1, const ColorRGBf& p2);
    friend ColorRGBf cross(const ColorRGBf& a, const ColorRGBf& b);
};

inline float dot(const ColorRGBf& p1, const ColorRGBf& p2)
{
    return p1.r * p2.r + p1.g * p2.g + p1.b * p2.b;
}

inline ColorRGBf cross(const ColorRGBf& a, const ColorRGBf& b)
{
    ColorRGBf vc;
    vc.r = a.g * b.b - a.b * b.g;
    vc.g = a.b * b.r - a.r * b.b;
    vc.b = a.r * b.g - a.g * b.r;

    return vc;
}

} // namespace aliceVision
