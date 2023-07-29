// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/device/operators.cuh>

#include <math_constants.h>

// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

namespace aliceVision {
namespace depthMap {

__device__ inline uchar4 float4_to_uchar4(const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

__device__ inline float4 uchar4_to_float4(const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

__device__ inline float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ inline float dot(const float2& a, const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

__device__ inline float size(const float3& a)
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

__device__ inline float size(const float2& a)
{
    return sqrtf(a.x * a.x + a.y * a.y);
}

__device__ inline float dist(const float3& a, const float3& b)
{
    return size(a - b);
}

__device__ inline float dist(const float2& a, const float2& b)
{
    return size(a - b);
}

__device__ inline float3 cross(const float3& a, const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

__device__ inline void normalize(float3& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;
    // a.z /= d;

    const float dInv = __fdividef(1.0f, sqrtf(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
    a.z *= dInv;
}

__device__ inline void normalize(float2& a)
{
    // without optimization
    // float d = sqrtf(dot(a, a));
    // a.x /= d;
    // a.y /= d;

    const float dInv = __fdividef(1.0f, sqrtf(dot(a, a)));
    a.x *= dInv;
    a.y *= dInv;
}

__device__ inline float3 M3x3mulV3( const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z,
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__device__ inline float3 M3x3mulV2( const float* M3x3, const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6],
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

__device__ inline float3 M3x4mulV3(const float* M3x4, const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

__device__ inline float2 V2M3x3mulV2(float* M3x3, float2& V)
{
    const float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}


__device__ inline float2 project3DPoint(const float* M3x4, const float3& V)
{
    // without optimization
    // const float3 p = M3x4mulV3(M3x4, V);
    // return make_float2(p.x / p.z, p.y / p.z);

    float3 p = M3x4mulV3(M3x4, V);
    const float pzInv =  __fdividef(1.0f, p.z);
    return make_float2(p.x * pzInv, p.y * pzInv);
}

__device__ inline void M3x3mulM3x3(float* O3x3, const float* A3x3, const float* B3x3)
{
    O3x3[0] = A3x3[0] * B3x3[0] + A3x3[3] * B3x3[1] + A3x3[6] * B3x3[2];
    O3x3[3] = A3x3[0] * B3x3[3] + A3x3[3] * B3x3[4] + A3x3[6] * B3x3[5];
    O3x3[6] = A3x3[0] * B3x3[6] + A3x3[3] * B3x3[7] + A3x3[6] * B3x3[8];

    O3x3[1] = A3x3[1] * B3x3[0] + A3x3[4] * B3x3[1] + A3x3[7] * B3x3[2];
    O3x3[4] = A3x3[1] * B3x3[3] + A3x3[4] * B3x3[4] + A3x3[7] * B3x3[5];
    O3x3[7] = A3x3[1] * B3x3[6] + A3x3[4] * B3x3[7] + A3x3[7] * B3x3[8];

    O3x3[2] = A3x3[2] * B3x3[0] + A3x3[5] * B3x3[1] + A3x3[8] * B3x3[2];
    O3x3[5] = A3x3[2] * B3x3[3] + A3x3[5] * B3x3[4] + A3x3[8] * B3x3[5];
    O3x3[8] = A3x3[2] * B3x3[6] + A3x3[5] * B3x3[7] + A3x3[8] * B3x3[8];
}

__device__ inline void M3x3minusM3x3(float* O3x3, float* A3x3, float* B3x3)
{
    O3x3[0] = A3x3[0] - B3x3[0];
    O3x3[1] = A3x3[1] - B3x3[1];
    O3x3[2] = A3x3[2] - B3x3[2];
    O3x3[3] = A3x3[3] - B3x3[3];
    O3x3[4] = A3x3[4] - B3x3[4];
    O3x3[5] = A3x3[5] - B3x3[5];
    O3x3[6] = A3x3[6] - B3x3[6];
    O3x3[7] = A3x3[7] - B3x3[7];
    O3x3[8] = A3x3[8] - B3x3[8];
}

__device__ inline void M3x3transpose(float* O3x3, const float* A3x3)
{
    O3x3[0] = A3x3[0];
    O3x3[1] = A3x3[3];
    O3x3[2] = A3x3[6];
    O3x3[3] = A3x3[1];
    O3x3[4] = A3x3[4];
    O3x3[5] = A3x3[7];
    O3x3[6] = A3x3[2];
    O3x3[7] = A3x3[5];
    O3x3[8] = A3x3[8];
}

__device__ inline void outerMultiply(float* O3x3, const float3& a, const float3& b)
{
    O3x3[0] = a.x * b.x;
    O3x3[3] = a.x * b.y;
    O3x3[6] = a.x * b.z;
    O3x3[1] = a.y * b.x;
    O3x3[4] = a.y * b.y;
    O3x3[7] = a.y * b.z;
    O3x3[2] = a.z * b.x;
    O3x3[5] = a.z * b.y;
    O3x3[8] = a.z * b.z;
}

__device__ inline float3 linePlaneIntersect(const float3& linePoint,
                                            const float3& lineVect,
                                            const float3& planePoint,
                                            const float3& planeNormal)
{
    const float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    return linePoint + lineVect * k;
}

__device__ inline float3 closestPointOnPlaneToPoint(const float3& point, const float3& planePoint, const float3& planeNormalNormalized)
{
    return point - planeNormalNormalized * dot(planeNormalNormalized, point - planePoint);
}

__device__ inline float3 closestPointToLine3D(const float3& point, const float3& linePoint, const float3& lineVectNormalized)
{
    return linePoint + lineVectNormalized * dot(lineVectNormalized, point - linePoint);
}

__device__ inline float pointLineDistance3D(const float3& point, const float3& linePoint, const float3& lineVectNormalized)
{
    return size(cross(lineVectNormalized, linePoint - point));
}

// v1,v2 dot not have to be normalized
__device__ inline float angleBetwV1andV2(const float3& iV1, const float3& iV2)
{
    float3 V1 = iV1;
    normalize(V1);

    float3 V2 = iV2;
    normalize(V2);

    return fabsf(acosf(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (CUDART_PI_F / 180.0f));
}

__device__ inline float angleBetwABandAC(const float3& A, const float3& B, const float3& C)
{
    float3 V1 = B - A;
    float3 V2 = C - A;

    normalize(V1);
    normalize(V2);

    const double x = double(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z);
    double a = acos(x);
    a = isinf(a) ? 0.0 : a;
    return float(fabs(a) / (CUDART_PI / 180.0));
}

/**
 * @brief Calculate the line segment PaPb that is the shortest route between two lines p1-p2 and p3-p4.
 *        Calculate also the values of mua and mub where:
 *          -> pa = p1 + mua (p2 - p1)
 *          -> pb = p3 + mub (p4 - p3)
 *
 * @note This a simple conversion to MATLAB of the C code posted by Paul Bourke at:
 *       http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 *       The author of this all too imperfect translation is Cristian Dima (csd@cmu.edu).
 *
 * @see https://web.archive.org/web/20060422045048/http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/
 */
__device__ inline float3 lineLineIntersect(float* k,
                                           float* l,
                                           float3* lli1,
                                           float3* lli2,
                                           const float3& p1,
                                           const float3& p2,
                                           const float3& p3,
                                           const float3& p4)
{
    float d1343, d4321, d1321, d4343, d2121, denom, numer, p13[3], p43[3], p21[3], pa[3], pb[3], muab[2];

    p13[0] = p1.x - p3.x;
    p13[1] = p1.y - p3.y;
    p13[2] = p1.z - p3.z;

    p43[0] = p4.x - p3.x;
    p43[1] = p4.y - p3.y;
    p43[2] = p4.z - p3.z;

    /*
    if ((abs(p43[0])  < eps) & ...
        (abs(p43[1])  < eps) & ...
        (abs(p43[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    p21[0] = p2.x - p1.x;
    p21[1] = p2.y - p1.y;
    p21[2] = p2.z - p1.z;

    /*
    if ((abs(p21[0])  < eps) & ...
        (abs(p21[1])  < eps) & ...
        (abs(p21[2])  < eps))
      error('Could not compute LineLineIntersect!');
    end
    */

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;

    /*
    if (abs(denom) < eps)
      error('Could not compute LineLineIntersect!');
    end
     */

    numer = d1343 * d4321 - d1321 * d4343;

    muab[0] = numer / denom;
    muab[1] = (d1343 + d4321 * muab[0]) / d4343;

    pa[0] = p1.x + muab[0] * p21[0];
    pa[1] = p1.y + muab[0] * p21[1];
    pa[2] = p1.z + muab[0] * p21[2];

    pb[0] = p3.x + muab[1] * p43[0];
    pb[1] = p3.y + muab[1] * p43[1];
    pb[2] = p3.z + muab[1] * p43[2];

    float3 S;
    S.x = (pa[0] + pb[0]) / 2.0;
    S.y = (pa[1] + pb[1]) / 2.0;
    S.z = (pa[2] + pb[2]) / 2.0;

    *k = muab[0];
    *l = muab[1];

    lli1->x = pa[0];
    lli1->y = pa[1];
    lli1->z = pa[2];

    lli2->x = pb[0];
    lli2->y = pb[1];
    lli2->z = pb[2];

    return S;
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (x - mid) / width}}
 * @see https://www.desmos.com/calculator/1qvampwbyx
 */
__device__ inline float sigmoid(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((xval - sigMid) / sigwidth))));
}

/**
 * @brief Sigmoid function filtering
 * @note f(x) = min + (max-min) * \frac{1}{1 + e^{10 * (mid - x) / width}}
 */
__device__ inline float sigmoid2(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((sigMid - xval) / sigwidth))));
}

} // namespace depthMap
} // namespace aliceVision
