// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// mn MATRIX ADDRESSING: mxy = x*n+y (x-row,y-col), (m-number of rows, n-number of columns)

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

inline static __device__ float3 M3x3mulV3(float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

inline static __device__ float3 M3x3mulV2(float* M3x3, const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6], M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

inline static __device__ float3 M3x4mulV3(float* M3x4, const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

inline static __device__ float2 V2M3x3mulV2(float* M3x3, float2& V)
{
    float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

inline static __device__ float2 project3DPoint(float* M3x4, const float3& V)
{
    float3 p = M3x4mulV3(M3x4, V);
    return make_float2(p.x / p.z, p.y / p.z);
}

__device__ void M3x3mulM3x3(float* O3x3, float* A3x3, float* B3x3);

__device__ void M3x3minusM3x3(float* O3x3, float* A3x3, float* B3x3);

__device__ void M3x3transpose(float* O3x3, float* A3x3);

inline static __device__ uchar4 float4_to_uchar4(const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

inline static __device__ float4 uchar4_to_float4(const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

inline static __device__ float4 operator*(const float4& a, const float& d)
{
    return make_float4(a.x * d, a.y * d, a.z * d, a.w * d);
}

inline static __device__ float4 operator+(const float4& a, const float4& d)
{
    return make_float4(a.x + d.x, a.y + d.y, a.z + d.z, a.w + d.w);
}

inline static __device__ float4 operator*(const float& d, const float4& a)
{
    return make_float4(a.x * d, a.y * d, a.z * d, a.w * d);
}

inline static __device__ float3 operator*(const float3& a, const float& d)
{
    return make_float3(a.x * d, a.y * d, a.z * d);
}

inline static __device__ float3 operator/(const float3& a, const float& d)
{
    return make_float3(a.x / d, a.y / d, a.z / d);
}

inline static __device__ float3 operator+(const float3& a, const float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline static __device__ float3 operator-(const float3& a, const float3& b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline static __device__ int2 operator+(const int2& a, const int2& b)
{
    return make_int2(a.x + b.x, a.y + b.y);
}

inline static __device__ float2 operator*(const float2& a, const float& d)
{
    return make_float2(a.x * d, a.y * d);
}

inline static __device__ float2 operator/(const float2& a, const float& d)
{
    return make_float2(a.x / d, a.y / d);
}

inline static __device__ float2 operator+(const float2& a, const float2& b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

inline static __device__ float2 operator-(const float2& a, const float2& b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}

inline static __device__ float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline static __device__ float dot(const float2& a, const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline static __device__ float size(const float3& a)
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline static __device__ float size(const float2& a)
{
    return sqrtf(a.x * a.x + a.y * a.y);
}

inline static __device__ float dist(const float3& a, const float3& b)
{
    float3 ab = a - b;
    return size(ab);
}

inline static __device__ float dist(const float2& a, const float2& b)
{
    float2 ab;
    ab.x = a.x - b.x;
    ab.y = a.y - b.y;
    return size(ab);
}

inline static __device__ float3 cross(const float3& a, const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline static __device__ void normalize(float3& a)
{
    float d = sqrtf(dot(a, a));
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

inline static __device__ void normalize(float2& a)
{
    float d = sqrtf(dot(a, a));
    a.x /= d;
    a.y /= d;
}

__device__ void outerMultiply(float* O3x3, const float3& a, const float3& b);

__device__ float3 linePlaneIntersect(const float3& linePoint, const float3& lineVect, const float3& planePoint,
                                     const float3& planeNormal);

__device__ float orientedPointPlaneDistanceNormalizedNormal(const float3& point, const float3& planePoint,
                                                            const float3& planeNormalNormalized);

__device__ float3 closestPointOnPlaneToPoint(const float3& point, const float3& planePoint,
                                             const float3& planeNormalNormalized);

__device__ float3 closestPointToLine3D(const float3& point, const float3& linePoint, const float3& lineVectNormalized);

__device__ float pointLineDistance3D(const float3& point, const float3& linePoint, const float3& lineVectNormalized);

// v1,v2 dot not have to be normalized
__device__ float angleBetwV1andV2(const float3& iV1, const float3& iV2);

__device__ float angleBetwABandAC(const float3& A, const float3& B, const float3& C);

__device__ float3 lineLineIntersect(float* k, float* l, float3* lli1, float3* lli2, float3& p1, float3& p2, float3& p3,
                                    float3& p4);

/**
 * f(x)=min + (max-min) * \frac{1}{1 + e^{10 * (x - mid) / width}}
 */
inline static __device__ float sigmoid(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((xval - sigMid) / sigwidth))));
}

inline static __device__ float sigmoid2(float zeroVal, float endVal, float sigwidth, float sigMid, float xval)
{
    return zeroVal + (endVal - zeroVal) * (1.0f / (1.0f + expf(10.0f * ((sigMid - xval) / sigwidth))));
}

} // namespace depthMap
} // namespace aliceVision
