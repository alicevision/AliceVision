// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <cmath>

namespace aliceVision {

class Color
{
public:
    union {
        struct
        {
            float r, g, b;
        };
        float m[3];
    };

    inline Color()
    {
        r = 0.0;
        g = 0.0;
        b = 0.0;
    }

    inline Color(float _r, float _g, float _b)
    {
        r = _r;
        g = _g;
        b = _b;
    }

    inline Color& operator=(const Color& param)
    {
        r = param.r;
        g = param.g;
        b = param.b;
        return *this;
    }

    inline bool operator==(const Color& param)
    {
        return (r == param.r) && (g == param.g) && (b == param.b);
    }

    inline Color operator-(const Color& _p) const
    {
        return Color(r - _p.r, g - _p.g, b - _p.b);
    }

    inline Color operator-() const
    {
        return Color(-r, -g, -b);
    }

    inline Color operator+(const Color& _p) const
    {
        return Color(r + _p.r, g + _p.g, b + _p.b);
    }

    inline Color operator*(const float d) const
    {
        return Color(r * d, g * d, b * d);
    }

    inline Color operator/(const float d) const
    {
        return Color(r / d, g / d, b / d);
    }

    inline Color& operator+=(const Color& _p)
    {
        r += _p.r;
        g += _p.g;
        b += _p.b;
        return *this;
    }

    inline Color& operator-=(const Color& _p)
    {
        r -= _p.r;
        g -= _p.g;
        b -= _p.b;
        return *this;
    }

    inline Color& operator/=(const int d)
    {
        r /= d;
        g /= d;
        b /= d;
        return *this;
    }

    inline Color normalize() const
    {
        float d = std::sqrt(r * r + g * g + b * b);
        return Color(r / d, g / d, b / d);
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

    friend float dot(const Color& p1, const Color& p2);
    friend Color cross(const Color& a, const Color& b);
};

inline float dot(const Color& p1, const Color& p2)
{
    return p1.r * p2.r + p1.g * p2.g + p1.b * p2.b;
}

inline Color cross(const Color& a, const Color& b)
{
    Color vc;
    vc.r = a.g * b.b - a.b * b.g;
    vc.g = a.b * b.r - a.r * b.b;
    vc.b = a.r * b.g - a.g * b.r;

    return vc;
}

} // namespace aliceVision
