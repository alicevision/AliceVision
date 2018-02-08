// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_matrix2x2.hpp"
#include "mv_matrix3x3.hpp"
#include "mv_matrix3x4.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct matrix2x3
{
    union {
        struct
        {
            double m11, m12, m13, m21, m22, m23;
        };
        double m[6];
    };

    inline matrix2x3& operator=(const matrix2x3& m)
    {
        m11 = m.m11;
        m12 = m.m12;
        m13 = m.m13;
        m21 = m.m21;
        m22 = m.m22;
        m23 = m.m23;
        return *this;
    };
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct matrix4x3
{
    union {
        struct
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33, m41, m42, m43;
        };
        double m[12];
    };

    inline matrix4x3& operator=(const matrix4x3& m)
    {
        m11 = m.m11;
        m12 = m.m12;
        m13 = m.m13;
        m21 = m.m21;
        m22 = m.m22;
        m23 = m.m23;
        m31 = m.m31;
        m32 = m.m32;
        m33 = m.m33;
        m41 = m.m41;
        m42 = m.m42;
        m43 = m.m43;
        return *this;
    };
};

struct matrix4x4
{
    union {
        struct
        {
            double m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, m41, m42, m43, m44;
        };
        double m[16];
    };

    inline matrix4x4& operator=(const matrix4x4& m)
    {
        m11 = m.m11;
        m12 = m.m12;
        m13 = m.m13;
        m14 = m.m14;
        m21 = m.m21;
        m22 = m.m22;
        m23 = m.m23;
        m24 = m.m24;
        m31 = m.m31;
        m32 = m.m32;
        m33 = m.m33;
        m34 = m.m34;
        m41 = m.m41;
        m42 = m.m42;
        m43 = m.m43;
        m44 = m.m44;
        return *this;
    };

    inline matrix3x3 minor11()
    {
        matrix3x3 m;
        m.m11 = m22;
        m.m12 = m23;
        m.m13 = m24;
        m.m21 = m32;
        m.m22 = m33;
        m.m23 = m34;
        m.m31 = m42;
        m.m32 = m43;
        m.m33 = m44;
        return m;
    };

    inline matrix3x3 minor12()
    {
        matrix3x3 m;
        m.m11 = m21;
        m.m12 = m23;
        m.m13 = m24;
        m.m21 = m31;
        m.m22 = m33;
        m.m23 = m34;
        m.m31 = m41;
        m.m32 = m43;
        m.m33 = m44;
        return m;
    };

    inline matrix3x3 minor13()
    {
        matrix3x3 m;
        m.m11 = m21;
        m.m12 = m22;
        m.m13 = m24;
        m.m21 = m31;
        m.m22 = m32;
        m.m23 = m34;
        m.m31 = m41;
        m.m32 = m42;
        m.m33 = m44;
        return m;
    };

    inline matrix3x3 minor14()
    {
        matrix3x3 m;
        m.m11 = m21;
        m.m12 = m22;
        m.m13 = m23;
        m.m21 = m31;
        m.m22 = m32;
        m.m23 = m33;
        m.m31 = m41;
        m.m32 = m42;
        m.m33 = m43;
        return m;
    };

    inline double determinant()
    {
        return m11 * minor11().deteminant() - m12 * minor12().deteminant() + m13 * minor13().deteminant() -
               m14 * minor14().deteminant();
    };

    void doprintf();
};


inline matrix3x4 operator*(const matrix3x3& M1, const matrix3x4& M2)
{
    matrix3x4 m;
    m.m11 = M1.m11 * M2.m11 + M1.m12 * M2.m21 + M1.m13 * M2.m31;
    m.m12 = M1.m11 * M2.m12 + M1.m12 * M2.m22 + M1.m13 * M2.m32;
    m.m13 = M1.m11 * M2.m13 + M1.m12 * M2.m23 + M1.m13 * M2.m33;
    m.m14 = M1.m11 * M2.m14 + M1.m12 * M2.m24 + M1.m13 * M2.m34;

    m.m21 = M1.m21 * M2.m11 + M1.m22 * M2.m21 + M1.m23 * M2.m31;
    m.m22 = M1.m21 * M2.m12 + M1.m22 * M2.m22 + M1.m23 * M2.m32;
    m.m23 = M1.m21 * M2.m13 + M1.m22 * M2.m23 + M1.m23 * M2.m33;
    m.m24 = M1.m21 * M2.m14 + M1.m22 * M2.m24 + M1.m23 * M2.m34;

    m.m31 = M1.m31 * M2.m11 + M1.m32 * M2.m21 + M1.m33 * M2.m31;
    m.m32 = M1.m31 * M2.m12 + M1.m32 * M2.m22 + M1.m33 * M2.m32;
    m.m33 = M1.m31 * M2.m13 + M1.m32 * M2.m23 + M1.m33 * M2.m33;
    m.m34 = M1.m31 * M2.m14 + M1.m32 * M2.m24 + M1.m33 * M2.m34;

    return m;
}

inline matrix3x3 operator*(const matrix3x4& M1, const matrix4x3& M2)
{
    matrix3x3 m;
    m.m11 = M1.m11 * M2.m11 + M1.m12 * M2.m21 + M1.m13 * M2.m31 + M1.m14 * M2.m41;
    m.m12 = M1.m11 * M2.m12 + M1.m12 * M2.m22 + M1.m13 * M2.m32 + M1.m14 * M2.m42;
    m.m13 = M1.m11 * M2.m13 + M1.m12 * M2.m23 + M1.m13 * M2.m33 + M1.m14 * M2.m43;

    m.m21 = M1.m21 * M2.m11 + M1.m22 * M2.m21 + M1.m23 * M2.m31 + M1.m24 * M2.m41;
    m.m22 = M1.m21 * M2.m12 + M1.m22 * M2.m22 + M1.m23 * M2.m32 + M1.m24 * M2.m42;
    m.m23 = M1.m21 * M2.m13 + M1.m22 * M2.m23 + M1.m23 * M2.m33 + M1.m24 * M2.m43;

    m.m31 = M1.m31 * M2.m11 + M1.m32 * M2.m21 + M1.m33 * M2.m31 + M1.m34 * M2.m41;
    m.m32 = M1.m31 * M2.m12 + M1.m32 * M2.m22 + M1.m33 * M2.m32 + M1.m34 * M2.m42;
    m.m33 = M1.m31 * M2.m13 + M1.m32 * M2.m23 + M1.m33 * M2.m33 + M1.m34 * M2.m43;

    return m;
}

inline matrix4x3 operator*(const matrix4x4& M1, const matrix4x3& M2)
{
    matrix4x3 m;
    m.m11 = M1.m11 * M2.m11 + M1.m12 * M2.m21 + M1.m13 * M2.m31 + M1.m14 * M2.m41;
    m.m12 = M1.m11 * M2.m12 + M1.m12 * M2.m22 + M1.m13 * M2.m32 + M1.m14 * M2.m42;
    m.m13 = M1.m11 * M2.m13 + M1.m12 * M2.m23 + M1.m13 * M2.m33 + M1.m14 * M2.m43;

    m.m21 = M1.m21 * M2.m11 + M1.m22 * M2.m21 + M1.m23 * M2.m31 + M1.m24 * M2.m41;
    m.m22 = M1.m21 * M2.m12 + M1.m22 * M2.m22 + M1.m23 * M2.m32 + M1.m24 * M2.m42;
    m.m23 = M1.m21 * M2.m13 + M1.m22 * M2.m23 + M1.m23 * M2.m33 + M1.m24 * M2.m43;

    m.m31 = M1.m31 * M2.m11 + M1.m32 * M2.m21 + M1.m33 * M2.m31 + M1.m34 * M2.m41;
    m.m32 = M1.m31 * M2.m12 + M1.m32 * M2.m22 + M1.m33 * M2.m32 + M1.m34 * M2.m42;
    m.m33 = M1.m31 * M2.m13 + M1.m32 * M2.m23 + M1.m33 * M2.m33 + M1.m34 * M2.m43;

    m.m41 = M1.m41 * M2.m11 + M1.m42 * M2.m21 + M1.m43 * M2.m31 + M1.m44 * M2.m41;
    m.m42 = M1.m41 * M2.m12 + M1.m42 * M2.m22 + M1.m43 * M2.m32 + M1.m44 * M2.m42;
    m.m43 = M1.m41 * M2.m13 + M1.m42 * M2.m23 + M1.m43 * M2.m33 + M1.m44 * M2.m43;

    return m;
}
