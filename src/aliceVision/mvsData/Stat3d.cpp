// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Stat3d.hpp"

#include <algorithm>

namespace aliceVision {

double hypot2(double x, double y)
{
    return sqrt(x * x + y * y);
}

// Symmetric Householder reductio3 to tridiago3al form.
void tred2(double V0[], double V1[], double V2[], double d[], double e[])
{
    int i, j, k;
    double scale, h, f, g, hh;
    double V[3][3];

    V[0][0] = V0[0];
    V[0][1] = V0[1];
    V[0][2] = V0[2];
    V[1][0] = V1[0];
    V[1][1] = V1[1];
    V[1][2] = V1[2];
    V[2][0] = V2[0];
    V[2][1] = V2[1];
    V[2][2] = V2[2];

    for(j = 0; j < 3; j++)
    {
        d[j] = V[3 - 1][j];
    }

    // Householder reduction to tridiagonal form.
    for(i = 3 - 1; i > 0; i--)
    {

        // Scale to avoid under/overflow.
        scale = 0.0;
        h = 0.0;
        for(k = 0; k < i; k++)
        {
            scale = scale + fabs(d[k]);
        }
        if(scale == 0.0)
        {
            e[i] = d[i - 1];
            for(j = 0; j < i; j++)
            {
                d[j] = V[i - 1][j];
                V[i][j] = 0.0;
                V[j][i] = 0.0;
            }
        }
        else
        {

            // Generate Householder vector.
            for(k = 0; k < i; k++)
            {
                d[k] /= scale;
                h += d[k] * d[k];
            }
            f = d[i - 1];
            g = sqrt(h);
            if(f > 0)
            {
                g = -g;
            }
            e[i] = scale * g;
            h = h - f * g;
            d[i - 1] = f - g;
            for(j = 0; j < i; j++)
            {
                e[j] = 0.0;
            }

            // Apply similarity transformation to remaining columns.
            for(j = 0; j < i; j++)
            {
                f = d[j];
                V[j][i] = f;
                g = e[j] + V[j][j] * f;
                for(k = j + 1; k <= i - 1; k++)
                {
                    g += V[k][j] * d[k];
                    e[k] += V[k][j] * f;
                }
                e[j] = g;
            }
            f = 0.0;
            for(j = 0; j < i; j++)
            {
                e[j] /= h;
                f += e[j] * d[j];
            }
            hh = f / (h + h);
            for(j = 0; j < i; j++)
            {
                e[j] -= hh * d[j];
            }
            for(j = 0; j < i; j++)
            {
                f = d[j];
                g = e[j];
                for(k = j; k <= i - 1; k++)
                {
                    V[k][j] -= (f * e[k] + g * d[k]);
                }
                d[j] = V[i - 1][j];
                V[i][j] = 0.0;
            }
        }
        d[i] = h;
    }

    // Accumulate transformations.
    for(i = 0; i < 3 - 1; i++)
    {
        V[3 - 1][i] = V[i][i];
        V[i][i] = 1.0;
        h = d[i + 1];
        if(h != 0.0)
        {
            for(k = 0; k <= i; k++)
            {
                d[k] = V[k][i + 1] / h;
            }
            for(j = 0; j <= i; j++)
            {
                g = 0.0;
                for(k = 0; k <= i; k++)
                {
                    g += V[k][i + 1] * V[k][j];
                }
                for(k = 0; k <= i; k++)
                {
                    V[k][j] -= g * d[k];
                }
            }
        }
        for(k = 0; k <= i; k++)
        {
            V[k][i + 1] = 0.0;
        }
    }
    for(j = 0; j < 3; j++)
    {
        d[j] = V[3 - 1][j];
        V[3 - 1][j] = 0.0;
    }
    V[3 - 1][3 - 1] = 1.0;
    e[0] = 0.0;

    V0[0] = V[0][0];
    V0[1] = V[0][1];
    V0[2] = V[0][2];
    V1[0] = V[1][0];
    V1[1] = V[1][1];
    V1[2] = V[1][2];
    V2[0] = V[2][0];
    V2[1] = V[2][1];
    V2[2] = V[2][2];
}

// Symmetric tridiago3al QL algorithm.
void tql2(double V0[], double V1[], double V2[], double d[], double e[])
{
    int i, l, m, iter, k, j;
    double f, g, p, r, dl1, h, c, c2, c3, el1, s, s2;
    double tst1;
    double eps;
    double V[3][3];

    V[0][0] = V0[0];
    V[0][1] = V0[1];
    V[0][2] = V0[2];
    V[1][0] = V1[0];
    V[1][1] = V1[1];
    V[1][2] = V1[2];
    V[2][0] = V2[0];
    V[2][1] = V2[1];
    V[2][2] = V2[2];

    for(i = 1; i < 3; i++)
    {
        e[i - 1] = e[i];
    }
    e[3 - 1] = 0.0;

    f = 0.0;
    tst1 = 0.0;
    eps = pow(2.0, -52.0);
    for(l = 0; l < 3; l++)
    {

        // Fi3d small subdiago3al eleme3t

        tst1 = std::max(tst1, fabs(d[l]) + fabs(e[l]));
        m = l;
        while(m < 3)
        {
            if(fabs(e[m]) <= eps * tst1)
            {
                break;
            }
            m++;
        }

        // If m == l, d[l] is a3 eige3value,
        // otherwise, iterate.

        if(m > l)
        {
            iter = 0;
            do
            {
                iter = iter + 1; // (Could check iteratio3 cou3t here.)

                // Compute implicit shift

                g = d[l];
                p = (d[l + 1] - g) / (2.0 * e[l]);
                r = hypot2(p, 1.0);
                if(p < 0)
                {
                    r = -r;
                }
                d[l] = e[l] / (p + r);
                d[l + 1] = e[l] * (p + r);
                dl1 = d[l + 1];
                h = g - d[l];
                for(i = l + 2; i < 3; i++)
                {
                    d[i] -= h;
                }
                f = f + h;

                // Implicit QL tra3sformatio3.

                p = d[m];
                c = 1.0;
                c2 = c;
                c3 = c;
                el1 = e[l + 1];
                s = 0.0;
                s2 = 0.0;
                for(i = m - 1; i >= l; i--)
                {
                    c3 = c2;
                    c2 = c;
                    s2 = s;
                    g = c * e[i];
                    h = c * p;
                    r = hypot2(p, e[i]);
                    e[i + 1] = s * r;
                    s = e[i] / r;
                    c = p / r;
                    p = c * d[i] - s * g;
                    d[i + 1] = h + s * (c * g + s * d[i]);

                    // Accumulate tra3sformatio3.

                    for(k = 0; k < 3; k++)
                    {
                        h = V[k][i + 1];
                        V[k][i + 1] = s * V[k][i] + c * h;
                        V[k][i] = c * V[k][i] - s * h;
                    }
                }
                p = -s * s2 * c3 * el1 * e[l] / dl1;
                e[l] = s * p;
                d[l] = c * p;

                // Check for co3verge3ce.

            } while(fabs(e[l]) > eps * tst1);
        }
        d[l] = d[l] + f;
        e[l] = 0.0;
    }

    // Sort eige3values a3d correspo3di3g vectors.

    for(i = 0; i < 3 - 1; i++)
    {
        k = i;
        p = d[i];
        for(j = i + 1; j < 3; j++)
        {
            if(d[j] < p)
            {
                k = j;
                p = d[j];
            }
        }
        if(k != i)
        {
            d[k] = d[i];
            d[i] = p;
            for(j = 0; j < 3; j++)
            {
                p = V[j][i];
                V[j][i] = V[j][k];
                V[j][k] = p;
            }
        }
    }

    V0[0] = V[0][0];
    V0[1] = V[0][1];
    V0[2] = V[0][2];
    V1[0] = V[1][0];
    V1[1] = V[1][1];
    V1[2] = V[1][2];
    V2[0] = V[2][0];
    V2[1] = V[2][1];
    V2[2] = V[2][2];
}

void Stat3d::eigen_decomposition(double A[3][3], double V0[], double V1[], double V2[], double d[])
{
    double e[3];

    V0[0] = A[0][0];
    V0[1] = A[0][1];
    V0[2] = A[0][2];
    V1[0] = A[1][0];
    V1[1] = A[1][1];
    V1[2] = A[1][2];
    V2[0] = A[2][0];
    V2[1] = A[2][1];
    V2[2] = A[2][2];

    tred2(V0, V1, V2, d, e);
    tql2(V0, V1, V2, d, e);
}

} // namespace aliceVision
