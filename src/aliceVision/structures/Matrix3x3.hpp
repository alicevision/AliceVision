// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>

#include <boost/math/constants/constants.hpp>

#include <string>


class Matrix3x3
{
public:
    union {
        struct
        {
            double m11, m12, m13, m21, m22, m23, m31, m32, m33;
        };
        double m[9];
    };

    inline Matrix3x3()
    {
        m11 = 0.0;
        m12 = 0.0;
        m13 = 0.0;
        m21 = 0.0;
        m22 = 0.0;
        m23 = 0.0;
        m31 = 0.0;
        m32 = 0.0;
        m33 = 0.0;
    }

    inline Matrix3x3& operator=(const Matrix3x3& m) = default;

    inline Matrix3x3 operator-(double d) const
    {
        Matrix3x3 m;
        m.m11 = m11 - d;
        m.m12 = m12 - d;
        m.m13 = m13 - d;
        m.m21 = m21 - d;
        m.m22 = m22 - d;
        m.m23 = m23 - d;
        m.m31 = m31 - d;
        m.m32 = m32 - d;
        m.m33 = m33 - d;

        return m;
    }

    inline Matrix3x3 operator/(double d) const
    {
        Matrix3x3 m;
        m.m11 = m11 / d;
        m.m12 = m12 / d;
        m.m13 = m13 / d;
        m.m21 = m21 / d;
        m.m22 = m22 / d;
        m.m23 = m23 / d;
        m.m31 = m31 / d;
        m.m32 = m32 / d;
        m.m33 = m33 / d;

        return m;
    }

    inline Matrix3x3 operator-() const
    {
        Matrix3x3 m;
        m.m11 = -m11;
        m.m12 = -m12;
        m.m13 = -m13;
        m.m21 = -m21;
        m.m22 = -m22;
        m.m23 = -m23;
        m.m31 = -m31;
        m.m32 = -m32;
        m.m33 = -m33;

        return m;
    }

    inline Matrix3x3 operator-(const Matrix3x3& M) const
    {
        Matrix3x3 m;
        m.m11 = m11 - M.m11;
        m.m12 = m12 - M.m12;
        m.m13 = m13 - M.m13;
        m.m21 = m21 - M.m21;
        m.m22 = m22 - M.m22;
        m.m23 = m23 - M.m23;
        m.m31 = m31 - M.m31;
        m.m32 = m32 - M.m32;
        m.m33 = m33 - M.m33;

        return m;
    }

    inline Matrix3x3 operator*(const Matrix3x3& _m) const
    {
        Matrix3x3 m;
        m.m11 = m11 * _m.m11 + m12 * _m.m21 + m13 * _m.m31;
        m.m12 = m11 * _m.m12 + m12 * _m.m22 + m13 * _m.m32;
        m.m13 = m11 * _m.m13 + m12 * _m.m23 + m13 * _m.m33;

        m.m21 = m21 * _m.m11 + m22 * _m.m21 + m23 * _m.m31;
        m.m22 = m21 * _m.m12 + m22 * _m.m22 + m23 * _m.m32;
        m.m23 = m21 * _m.m13 + m22 * _m.m23 + m23 * _m.m33;

        m.m31 = m31 * _m.m11 + m32 * _m.m21 + m33 * _m.m31;
        m.m32 = m31 * _m.m12 + m32 * _m.m22 + m33 * _m.m32;
        m.m33 = m31 * _m.m13 + m32 * _m.m23 + m33 * _m.m33;

        return m;
    }

    inline Point3d operator*(const Point2d& _p) const
    {
        Point3d p;
        p.x = m11 * _p.x + m12 * _p.y + m13;
        p.y = m21 * _p.x + m22 * _p.y + m23;
        p.z = m31 * _p.x + m32 * _p.y + m33;
        return p;
    }

    inline Point2d operator^(const Point2d& _p) const
    {
        Point3d p = *this * _p;
        Point2d pix = Point2d(p.x / p.z, p.y / p.z);
        return pix;
    }

    inline Point3d operator*(const Point3d& _p) const
    {
        Point3d p;
        p.x = m11 * _p.x + m12 * _p.y + m13 * _p.z;
        p.y = m21 * _p.x + m22 * _p.y + m23 * _p.z;
        p.z = m31 * _p.x + m32 * _p.y + m33 * _p.z;
        return p;
    }

    inline Matrix3x3 transpose() const
    {
        Matrix3x3 m;
        m.m11 = m11;
        m.m12 = m21;
        m.m13 = m31;
        m.m21 = m12;
        m.m22 = m22;
        m.m23 = m32;
        m.m31 = m13;
        m.m32 = m23;
        m.m33 = m33;

        return m;
    }

    inline double deteminant() const
    {
        return m11 * m22 * m33 - m11 * m23 * m32 - m12 * m21 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m13 * m22 * m31;
    }

    inline bool isSingular() const
    {
        return (deteminant() == 0.0f);
    }

    inline double det() const
    {
        return m11 * (m33 * m22 - m32 * m23) - m21 * (m33 * m12 - m32 * m13) + m31 * (m23 * m12 - m22 * m13);
    }

    inline Matrix3x3 inverse() const
    {
        Matrix3x3 m;
        inverse(m);
        return m;
    }

    inline void QR(Matrix3x3& Q, Matrix3x3& R)
    {
        /*
        Point3d a1 = Point3d(m13,m23,m33);
        Point3d a2 = Point3d(m12,m22,m32);
        Point3d a3 = Point3d(m11,m21,m31);
        */

        /*
        Point3d a1 = Point3d(m11,m21,m31);
        Point3d a2 = Point3d(m12,m22,m32);
        Point3d a3 = Point3d(m13,m23,m33);
        */

        /*
        Point3d a1 = Point3d(m31,m32,m33);
        Point3d a2 = Point3d(m21,m22,m23);
        Point3d a3 = Point3d(m11,m12,m13);
        */

        Point3d a1 = Point3d(m11, m12, m13);
        Point3d a2 = Point3d(m21, m22, m23);
        Point3d a3 = Point3d(m31, m32, m33);

        Point3d u1 = a1;
        Point3d e1 = u1.normalize();
        Point3d u2 = a2 - proj(e1, a2);
        Point3d e2 = u2.normalize();
        Point3d u3 = a3 - proj(e1, a3) - proj(e2, a3);
        Point3d e3 = u3.normalize();

        Q.m11 = e1.x;
        Q.m12 = e1.y;
        Q.m13 = e1.z;
        Q.m21 = e2.x;
        Q.m22 = e2.y;
        Q.m23 = e2.z;
        Q.m31 = e3.x;
        Q.m32 = e3.y;
        Q.m33 = e3.z;

        R.m11 = dot(e1, a1);
        R.m12 = dot(e1, a2);
        R.m13 = dot(e1, a3);
        R.m21 = 0.0f;
        R.m22 = dot(e2, a2);
        R.m23 = dot(e2, a3);
        R.m31 = 0.0f;
        R.m32 = 0.0f;
        R.m33 = dot(e3, a3);
    }

    inline Matrix3x3 transpodeAndEnd1End1()
    {
        Matrix3x3 S = transpose();
        Matrix3x3 SS;
        SS.m11 = S.m33;
        SS.m12 = S.m32;
        SS.m13 = S.m31;
        SS.m21 = S.m23;
        SS.m22 = S.m22;
        SS.m23 = S.m21;
        SS.m31 = S.m13;
        SS.m32 = S.m12;
        SS.m33 = S.m11;
        return SS;
    }

    inline Point3d mldivide(const Point3d& b)
    {
        if(!isSingular())
        {
            return inverse() * b;
        }
        throw std::runtime_error("Matrix is singular.");
    }

    inline void RQ(Matrix3x3& R, Matrix3x3& Q)
    {
        /*
        Matrix3x3 S = transpodeAndEnd1End1();


        printf("S\n");
        S.doprintf();

        S.QR(Q, R);

        printf("R\n");
        R.doprintf();
        printf("Q\n");
        Q.doprintf();


        Q = Q.transpodeAndEnd1End1();
        R = R.transpodeAndEnd1End1();

        if (Q.deteminant()<0.0f)
        {
                R.m11 = -R.m11;
                R.m21 = -R.m21;
                R.m31 = -R.m31;

                Q.m11 = -Q.m11;
                Q.m12 = -Q.m12;
                Q.m13 = -Q.m13;
        };
        */

        Point3d a1 = Point3d(m31, m32, m33);
        Point3d a2 = Point3d(m21, m22, m23);
        Point3d a3 = Point3d(m11, m12, m13);

        Point3d u1 = a1;
        Point3d e1 = u1.normalize();
        Point3d u2 = a2 - proj(e1, a2);
        Point3d e2 = u2.normalize();
        Point3d u3 = a3 - proj(e1, a3) - proj(e2, a3);
        Point3d e3 = u3.normalize();

        Q.m11 = e3.x;
        Q.m12 = e3.y;
        Q.m13 = e3.z;
        Q.m21 = e2.x;
        Q.m22 = e2.y;
        Q.m23 = e2.z;
        Q.m31 = e1.x;
        Q.m32 = e1.y;
        Q.m33 = e1.z;

        R.m11 = dot(e1, a1);
        R.m12 = dot(e1, a2);
        R.m13 = dot(e1, a3);
        R.m21 = 0.0f;
        R.m22 = dot(e2, a2);
        R.m23 = dot(e2, a3);
        R.m31 = 0.0f;
        R.m32 = 0.0f;
        R.m33 = dot(e3, a3);

        Matrix3x3 RR = R;
        R.m11 = RR.m33;
        R.m12 = RR.m23;
        R.m13 = RR.m13;
        R.m21 = RR.m32;
        R.m22 = RR.m22;
        R.m23 = RR.m12;
        R.m31 = RR.m31;
        R.m32 = RR.m21;
        R.m33 = RR.m11;
    }

    inline bool inverse(Matrix3x3& m) const
    {
        double dt = det();

        if((fabs(dt) < 0.00000001f) || std::isnan(dt))
        {
            return false;
        }

        m.m11 = (m33 * m22 - m32 * m23) / dt;
        m.m12 = -(m33 * m12 - m32 * m13) / dt;
        m.m13 = (m23 * m12 - m22 * m13) / dt;
        m.m21 = -(m33 * m21 - m31 * m23) / dt;
        m.m22 = (m33 * m11 - m31 * m13) / dt;
        m.m23 = -(m23 * m11 - m21 * m13) / dt;
        m.m31 = (m32 * m21 - m31 * m22) / dt;
        m.m32 = -(m32 * m11 - m31 * m12) / dt;
        m.m33 = (m22 * m11 - m21 * m12) / dt;

        return true;

        /*
    | m11 m12 m13 |-1             |   m33m22-m32m23  -(m33m12-m32m13)   m23m12-m22m13  |
    | m21 m22 m23 |    =  1/DET * | -(m33m21-m31m23)   m33m11-m31m13  -(m23m11-m21m13) |
    | m31 m32 m33 |               |   m32m21-m31m22  -(m32m11-m31m12)   m22m11-m21m12  |

    with DET  =  m11(m33m22-m32m23)-m21(m33m12-m32m13)+m31(m23m12-m22m13)
        */

        /*
        double mem[9];
        mem[0] = (double)m11; mem[1] = (double)m12; mem[2] = (double)m13;
        mem[3] = (double)m21; mem[4] = (double)m22; mem[5] = (double)m23;
        mem[6] = (double)m31; mem[7] = (double)m32; mem[8] = (double)m33;

        Matrix<double> K(mem, 3, 3);
        Matrix<double> KK = K;
        Matrix<double> iK = KK.Inversed();


        Matrix3x3 m;
        m.m11 = (double)iK.M(1,1);
        m.m12 = (double)iK.M(1,2);
        m.m13 = (double)iK.M(1,3);
        m.m21 = (double)iK.M(2,1);
        m.m22 = (double)iK.M(2,2);
        m.m23 = (double)iK.M(2,3);
        m.m31 = (double)iK.M(3,1);
        m.m32 = (double)iK.M(3,2);
        m.m33 = (double)iK.M(3,3);

        K.Detach();

        return m;
        */
    }
};

inline Matrix3x3 I3x3()
{
    Matrix3x3 m;
    m.m11 = 1.0;
    m.m12 = 0.0;
    m.m13 = 0.0;

    m.m21 = 0.0;
    m.m22 = 1.0;
    m.m23 = 0.0;

    m.m31 = 0.0;
    m.m32 = 0.0;
    m.m33 = 1.0;

    return m;
}

inline Matrix3x3 diag3x3(double d1, double d2, double d3)
{
    Matrix3x3 m;
    m.m11 = d1;
    m.m12 = 0.0;
    m.m13 = 0.0;

    m.m21 = 0.0;
    m.m22 = d2;
    m.m23 = 0.0;

    m.m31 = 0.0;
    m.m32 = 0.0;
    m.m33 = d3;

    return m;
}

inline Matrix3x3 angRad2rot(float radx, float rady, float radz)
{
    float angx = radx;
    float angy = rady;
    float angz = radz;

    float sa = sin(angy);
    float ca = cos(angy);
    float sb = sin(angx);
    float cb = cos(angx);
    float sc = sin(angz);
    float cc = cos(angz);

    Matrix3x3 ra;
    ra.m11 = ca;
    ra.m12 = sa;
    ra.m13 = 0.0f;
    ra.m21 = -sa;
    ra.m22 = ca;
    ra.m23 = 0.0f;
    ra.m31 = 0.0f;
    ra.m32 = 0.0f;
    ra.m33 = 1.0f;

    Matrix3x3 rb;
    rb.m11 = cb;
    rb.m12 = 0.0f;
    rb.m13 = sb;
    rb.m21 = 0.0f;
    rb.m22 = 1.0f;
    rb.m23 = 0.0f;
    rb.m31 = -sb;
    rb.m32 = 0.0f;
    rb.m33 = cb;

    Matrix3x3 rc;
    rc.m11 = 1.0f;
    rc.m12 = 0.0f;
    rc.m13 = 0.0f;
    rc.m21 = 0.0f;
    rc.m22 = cc;
    rc.m23 = sc;
    rc.m31 = 0.0f;
    rc.m32 = -sc;
    rc.m33 = cc;

    return rc * rb * ra;
}

inline Matrix3x3 angDeg2rot(float degx, float degy, float degz)
{
    const float degToRad = boost::math::constants::pi<float>() / 180.0f;
    float radx = degToRad * degx;
    float rady = degToRad * degy;
    float radz = degToRad * degz;

    return angRad2rot(radx, rady, radz);
}


inline Matrix3x3 outerMultiply(const Point3d& a, const Point3d& b)
{
    Matrix3x3 m;
    m.m11 = a.x * b.x;
    m.m12 = a.x * b.y;
    m.m13 = a.x * b.z;
    m.m21 = a.y * b.x;
    m.m22 = a.y * b.y;
    m.m23 = a.y * b.z;
    m.m31 = a.z * b.x;
    m.m32 = a.z * b.y;
    m.m33 = a.z * b.z;

    return m;
}
