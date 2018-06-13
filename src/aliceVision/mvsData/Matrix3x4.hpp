// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>

namespace aliceVision {

class Matrix3x4
{
public:
    union {
        struct
        {
            double m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34;
        };
        double m[12];
    };

    inline Matrix3x4()
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

    inline Matrix3x4& operator=(const Matrix3x4& m) = default;

    inline Point3d operator*(const Point3d& _p) const
    {
        return Point3d(m11 * _p.x + m12 * _p.y + m13 * _p.z + m14,
                       m21 * _p.x + m22 * _p.y + m23 * _p.z + m24,
                       m31 * _p.x + m32 * _p.y + m33 * _p.z + m34);
    }

    inline double deteminant() const
    {
        return m11 * m22 * m33 - m11 * m23 * m32 - m12 * m21 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m13 * m22 * m31;
    }

    inline Matrix3x3 sub3x3() const
    {
        Matrix3x3 m;
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

    inline Point3d lastColumn() const
    {
        Point3d m;
        m.x = m14;
        m.y = m24;
        m.z = m34;
        return m;
    }


    void decomposeProjectionMatrix(Matrix3x3& K, Matrix3x3& R, Point3d& C) const
    {
        Matrix3x3 H = sub3x3();
        H.RQ(K, R);

        const bool cam_affine = (K.m33 == 0);

        if(!cam_affine)
        {
            K = K / fabs(K.m33);
        }
        else
        {
            std::cout << m11 << m12 << m13 << m14 << std::endl;
            std::cout << m21 << m22 << m23 << m24 << std::endl;
            std::cout << m31 << m32 << m33 << m34 << std::endl;
            throw std::runtime_error("Matrix3x4::decomposeProjectionMatrix: affine camera.");
        }

        if(K.m11 < 0.0f)
        {
            Matrix3x3 D = diag3x3(-1.0f, -1.0f, 1.0f);
            K = K * D;
            R = D * R;
        }

        if(K.m22 < 0.0f)
        {
            Matrix3x3 D = diag3x3(1.0f, -1.0f, -1.0f);
            K = K * D;
            R = D * R;
        }

        C = (-sub3x3()).mldivide(lastColumn());
    }
};

inline Matrix3x4 operator*(const Matrix3x3& M1, const Matrix3x4& M2)
{
    Matrix3x4 m;
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

inline Matrix3x4 operator|(const Matrix3x3& M, const Point3d& p)
{
    Matrix3x4 m;
    m.m11 = M.m11;
    m.m12 = M.m12;
    m.m13 = M.m13;
    m.m14 = p.x;
    m.m21 = M.m21;
    m.m22 = M.m22;
    m.m23 = M.m23;
    m.m24 = p.y;
    m.m31 = M.m31;
    m.m32 = M.m32;
    m.m33 = M.m33;
    m.m34 = p.z;

    return m;
}

} // namespace aliceVision
