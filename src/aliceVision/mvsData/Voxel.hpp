// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cmath>

namespace aliceVision {

struct Voxel
{
    union {
        struct
        {
            int x, y, z;
        };
        int m[3];
    };

    Voxel()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    Voxel(const int _x, const int _y, const int _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    int& operator[](const int index)
    {
        return m[index];
    }

    Voxel& operator=(const Voxel& param)
    {
        x = param.x;
        y = param.y;
        z = param.z;
        return *this;
    }

    Voxel operator-(const Voxel& _p) const
    {
        Voxel p;
        p.x = x - _p.x;
        p.y = y - _p.y;
        p.z = z - _p.z;
        return p;
    }

    Voxel operator+(const Voxel& _p) const
    {
        Voxel p;
        p.x = x + _p.x;
        p.y = y + _p.y;
        p.z = z + _p.z;
        return p;
    }

    Voxel operator+(int _p) const
    {
        Voxel p;
        p.x = x + _p;
        p.y = y + _p;
        p.z = z + _p;
        return p;
    }

    Voxel operator*(const Voxel& _p) const
    {
        Voxel p;
        p.x = x * _p.x;
        p.y = y * _p.y;
        p.z = z * _p.z;
        return p;
    }

    Voxel operator*(int d) const
    {
        Voxel p;
        p.x = x * d;
        p.y = y * d;
        p.z = z * d;
        return p;
    }

    Voxel operator/(int d) const
    {
        if(d == 0)
            return Voxel(0, 0, 0);

        Voxel p;
        p.x = static_cast<int>(std::floor(static_cast<float>(x) / static_cast<float>(d) + 0.5f));
        p.y = static_cast<int>(std::floor(static_cast<float>(y) / static_cast<float>(d) + 0.5f));
        p.z = static_cast<int>(std::floor(static_cast<float>(z) / static_cast<float>(d) + 0.5f));
        return p;
    }

    float size() const
    {
        float d = static_cast<float>(x * x + y * y + z * z);
        if(d == 0.0f)
            return 0.0f;
        return std::sqrt(d);
    }

    bool operator==(const Voxel& param) const
    {
        return (x == param.x) && (y == param.y) && (z == param.z);
    }

    bool operator!=(const Voxel& param) const
    {
        return (x != param.x) || (y != param.y) || (z != param.z);
    }
};

} // namespace aliceVision
