#pragma once

#include "mv_point2d.hpp"

#include <cmath>



class matrix2x2
{
public:
    double m11, m12, m21, m22;

    inline matrix2x2()
    {
        m11 = 0.0;
        m12 = 0.0;
        m21 = 0.0;
        m22 = 0.0;
    }

    inline matrix2x2& operator=(const matrix2x2& m) = default;

    inline matrix2x2 inverse()
    {
        double d = 1.0f / m11 * m22 - m12 * m21;

        matrix2x2 m;
        m.m11 = m22 * d;
        m.m12 = -m12 * d;
        m.m21 = -m21 * d;
        m.m22 = m11 * d;

        return m;
    }

    inline bool eig(double& e1, double& e2, point2d& v1, point2d& v2)
    {
        double b24ac = m22 * m22 - 2.0f * m11 * m22 + m11 * m11 + 4.0f * m12 * m21;
        if((b24ac < 0.00000001f) || (static_cast<int>(std::isnan(b24ac)) > 0))
        {
            return false;
        }

        b24ac = sqrt(b24ac);

        // compute eigen vectors and eigen values
        v1 = point2d(-(1.0f / 2.0f * m22 - 1.0f / 2.0f * m11 - 1.0f / 2.0f * b24ac) / m21, 1.0f);
        v2 = point2d(-(1.0f / 2.0f * m22 - 1.0f / 2.0f * m11 + 1.0f / 2.0f * b24ac) / m21, 1.0f);
        e1 = 1.0f / 2.0f * m22 + 1.0f / 2.0f * m11 + 1.0f / 2.0f * b24ac;
        e2 = 1.0f / 2.0f * m22 + 1.0f / 2.0f * m11 - 1.0f / 2.0f * b24ac;

        if(e1 > e2)
        {

            double tmpe = e1;
            e1 = e2;
            e2 = tmpe;

            point2d tmpv = v1;
            v1 = v2;
            v2 = tmpv;
        }

        return true;
    }

    inline double deteminant()
    {
        return m11 * m22 - m12 * m21;
    }

    inline bool isSingular()
    {
        return (deteminant() == 0.0);
    }

    void doprintf();
};
