// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_structures.hpp"

#include <stdexcept>

point2d operator*(const matrix2x3& M, const point3d& _p)
{
    point2d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23;
    return p;
}

matrix2x3 DPIP(const matrix3x4& M, const point3d& P)
{
    float M3P = M.m31 * P.x + M.m32 * P.y + M.m33 * P.z + M.m34;
    float M3P2 = M3P * M3P;

    matrix2x3 o;
    o.m11 = ((M.m11 * M.m32 - M.m31 * M.m12) * P.y + (M.m11 * M.m33 - M.m31 * M.m13) * P.z +
             (M.m11 * M.m34 - M.m31 * M.m14)) /
            M3P2;
    o.m12 = ((M.m12 * M.m31 - M.m32 * M.m11) * P.x + (M.m12 * M.m33 - M.m32 * M.m13) * P.z +
             (M.m12 * M.m34 - M.m32 * M.m14)) /
            M3P2;
    o.m13 = ((M.m13 * M.m31 - M.m33 * M.m11) * P.x + (M.m13 * M.m32 - M.m33 * M.m12) * P.y +
             (M.m13 * M.m34 - M.m33 * M.m14)) /
            M3P2;

    o.m21 = ((M.m21 * M.m32 - M.m31 * M.m22) * P.y + (M.m21 * M.m33 - M.m31 * M.m23) * P.z +
             (M.m21 * M.m34 - M.m31 * M.m24)) /
            M3P2;
    o.m22 = ((M.m22 * M.m31 - M.m32 * M.m21) * P.x + (M.m22 * M.m33 - M.m32 * M.m23) * P.z +
             (M.m22 * M.m34 - M.m32 * M.m24)) /
            M3P2;
    o.m23 = ((M.m23 * M.m31 - M.m33 * M.m21) * P.x + (M.m23 * M.m32 - M.m33 * M.m22) * P.y +
             (M.m23 * M.m34 - M.m33 * M.m24)) /
            M3P2;

    return o;
}

point3d operator*(const matrix3x3& M, const point2d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33;
    return p;

    /* //result = a*b
    result.v.x = a.v.x*b.m11 + a.v.y*b.m21 + a.v.z*b.m31;
    result.v.y = a.v.x*b.m12 + a.v.y*b.m22 + a.v.z*b.m32;
    result.v.z = a.v.x*b.m13 + a.v.y*b.m23 + a.v.z*b.m33;//
    */
    /*
    __declspec(align(16)) float a[4]; a[0] = _p.x; a[1] = _p.y; a[2] = 1.0; a[3] = 1.0;
    __declspec(align(16)) float b[12];
    b[0]  = M.m11;
    b[1]  = M.m12;
    b[2]  = M.m13;
    b[3]  = 0.0;
    b[4]  = M.m21;
    b[5]  = M.m22;
    b[6]  = M.m23;
    b[7]  = 0.0;
    b[8]  = M.m31;
    b[9]  = M.m32;
    b[10] = M.m33;
    b[11] = 0.0;

    __declspec(align(16)) float p[4];

    __asm {
            mov edi,a
            mov esi,b
            movss xmm0,[edi+0] //x
            shufps xmm0,xmm0,0 //a.v.x to all floats
            movss xmm2,[edi+4] //y
            mulps xmm0,[esi+0] //row[0]*x
            shufps xmm2,xmm2,0 //a.v.y to all floats
            movss xmm1,[edi+8] //z
            mulps xmm2,[esi+16]//row[1]*y
            shufps xmm1,xmm1,0 //a.v.z to all floats
            addps xmm0,xmm2
            mulps xmm1,[esi+32] //row[2]*z
            mov edi,p
            addps xmm0,xmm1
            movaps [edi],xmm0
    };

    point3d vc;
    vc.x = p[0];
    vc.y = p[1];
    vc.z = p[2];

    return vc;
    */
}

point2d operator^(const matrix3x3& M, const point2d& _p)
{
    point3d p = M * _p;
    point2d pix = point2d(p.x / p.z, p.y / p.z);
    return pix;
}

matrix3x4 operator|(const matrix3x3& M, const point3d& p)
{
    matrix3x4 m;
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

matrix3x3 outerMultiply(const point3d& a, const point3d& b)
{
    matrix3x3 m;
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

point3d operator*(const matrix4x4& M, const point3d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z + M.m14;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z + M.m24;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z + M.m34;
    float w = M.m41 * _p.x + M.m42 * _p.y + M.m43 * _p.z + M.m44;

    p = p / w;

    return p;
}

point3d operator*(const matrix3x3& M, const point3d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z;
    return p;
}

point3d operator*(const matrix3x4& M, const point3d& _p)
{
    return point3d(M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z + M.m14,
                   M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z + M.m24,
                   M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z + M.m34);
}

point3d mldivide(const matrix3x3& M, const point3d& b)
{
    if(!M.isSingular())
    {
        return M.inverse() * b;
    }

    M.doprintf();
    throw std::runtime_error("Matrix is singular.");
}

point2d operator*(const matrix2x2& M, const point2d& _p)
{
    point2d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y;
    p.y = M.m21 * _p.x + M.m22 * _p.y;
    return p;
}

point2d mldivide(matrix2x2& M, point2d& b)
{
    if(!M.isSingular())
    {
        return M.inverse() * b;
    }

    M.doprintf();
    throw std::runtime_error("Matrix is singular.");
}

int qSortCompareFloatAsc(const void* ia, const void* ib)
{
    float a = *(float*)ia;
    float b = *(float*)ib;
    if(a > b)
    {
        return 1;
    }

    return -1;
}

int qSortCompareIntAsc(const void* ia, const void* ib)
{
    int a = *(int*)ia;
    int b = *(int*)ib;
    if(a > b)
    {
        return 1;
    }

    return -1;
}

int qSortCompareIntDesc(const void* ia, const void* ib)
{
    int a = *(int*)ia;
    int b = *(int*)ib;
    if(a < b)
    {
        return 1;
    }

    return -1;
}

int compareSortedId(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value > b.value)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdDesc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value > b.value)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdAsc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value < b.value)
    {
        return -1;
    }

    return 1;
}
int qSortCompareVoxelByZDesc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.z > b.z)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByXAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByYAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.y < b.y)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByZAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.z < b.z)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXDesc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.x > b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXAsc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByYDesc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.y > b.y)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByYAsc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.y < b.y)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdByIdAsc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.id < b.id)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByXAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByYAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByZAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

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

void stat3d::eigen_decomposition(double A[3][3], double V0[], double V1[], double V2[], double d[])
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

mv_bites_array::mv_bites_array(int _nbits)
{
    sizeofbits = sizeof(unsigned int) * 8;
    nbits = _nbits;
    allocated = nbits / sizeofbits + 1;
    bits = new unsigned int[allocated];
    clear();
}

mv_bites_array::~mv_bites_array()
{
    delete[] bits;
}

void mv_bites_array::clear()
{
    for(int i = 0; i < allocated; i++)
    {
        bits[i] = 0;
    }
}

bool mv_bites_array::getbit(int bitid)
{
    int i = bitid / sizeofbits;
    int j = bitid % sizeofbits;
    return (bits[i] & (1 << j)) != 0; /* 0 or 1   */
}

void mv_bites_array::setbit(int bitid, bool value)
{
    int i = bitid / sizeofbits;
    int j = bitid % sizeofbits;
    // printf("%i %i %i %i\n",bitid,i,j,sizeofbits);
    if(value)
        bits[i] |= (1 << j); /* set bit  */
    else
        bits[i] &= ~(1 << j); /* clear bit*/
}

void mv_bites_array::ORBits(mv_bites_array* e)
{
    for(int i = 0; i < std::min(allocated, e->allocated); i++)
    {
        bits[i] |= e->bits[i];
    }
}

void mv_bites_array::ANDBits(mv_bites_array* e)
{
    for(int i = 0; i < std::min(allocated, e->allocated); i++)
    {
        bits[i] &= e->bits[i];
    }
}

int mv_bites_array::gentNSumBitsOfANDBits(mv_bites_array* e)
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        out += (int)(getbit(i) && e->getbit(i));
    }
    return out;
}

int mv_bites_array::getNSumBits()
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        out += (int)getbit(i);
    }
    return out;
}

void mv_bites_array::printfBits()
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        bool b = getbit(i);
        printf("%i", (int)b);
        out += (int)b;
    }
    printf("\n n 1 bits : %i\n", out);
}

int getANDBits(mv_bites_array* a1, mv_bites_array* a2)
{
    mv_bites_array* a3 = new mv_bites_array(a1->nbits);
    a3 = a1;
    a3->ANDBits(a2);
    int out = a3->getNSumBits();
    delete a3;
    return out;
}

rgb rgb_random()
{
    rgb c;

    c.r = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
    c.g = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
    c.b = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);

    return c;
}

int indexOfSortedVoxelArrByX(int val, staticVector<voxel>* values, int startId, int stopId)
{
    int lef = startId;
    int rig = stopId;
    int mid = lef + (rig - lef) / 2;
    while((rig - lef) > 1)
    {
        if((val >= (*values)[lef].x) && (val < (*values)[mid].x))
        {
            lef = lef;
            rig = mid;
            mid = lef + (rig - lef) / 2;
        }
        if((val >= (*values)[mid].x) && (val <= (*values)[rig].x))
        {
            lef = mid;
            rig = rig;
            mid = lef + (rig - lef) / 2;
        }
        if((val < (*values)[lef].x) || (val > (*values)[rig].x))
        {
            lef = 0;
            rig = 0;
            mid = 0;
        }
    }

    int id = -1;
    if(val == (*values)[lef].x)
    {
        id = lef;
    }
    if(val == (*values)[rig].x)
    {
        id = rig;
    }

    return id;
}

