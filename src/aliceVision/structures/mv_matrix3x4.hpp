// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_staticVector.hpp"
#include "mv_matrix3x3.hpp"
#include "mv_point3d.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class matrix3x4
{
public:
    union {
        struct
        {
            double m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
        };
        double m[12];
    };

    inline matrix3x4()
    {
        m11 = 0.0;
        m12 = 0.0;
        m13 = 0.0;
        m14 = 0.0;
        m21 = 0.0;
        m22 = 0.0;
        m23 = 0.0;
        m24 = 0.0;
        m31 = 0.0;
        m32 = 0.0;
        m33 = 0.0;
        m34 = 0.0;
    }

    inline matrix3x4& operator=(const matrix3x4& m) = default;

    inline double deteminant() const
    {
        return m11 * m22 * m33 - m11 * m23 * m32 - m12 * m21 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m13 * m22 * m31;
    }

    inline matrix3x3 sub3x3() const
    {
        matrix3x3 m;
        m.m11 = m11;
        m.m12 = m12;
        m.m13 = m13;
        m.m21 = m21;
        m.m22 = m22;
        m.m23 = m23;
        m.m31 = m31;
        m.m32 = m32;
        m.m33 = m33;
        return m;
    }

    inline point3d lastColumn() const
    {
        point3d m;
        m.x = m14;
        m.y = m24;
        m.z = m34;
        return m;
    }

    void doprintf() const;
    void saveToFile(std::string fileName) const;
    void loadFromFile(std::string fileName);
    void decomposeProjectionMatrix(matrix3x3& K, matrix3x3& R, point3d& C) const;
};
