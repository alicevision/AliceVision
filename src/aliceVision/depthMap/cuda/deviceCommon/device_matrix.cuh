// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <math_constants.h>
#include <aliceVision/depthMap/cuda/deviceCommon/device_operators.cuh>

namespace aliceVision {
namespace depthMap {

__device__ static inline uchar4 float4_to_uchar4(const float4& a)
{
    return make_uchar4((unsigned char)a.x, (unsigned char)a.y, (unsigned char)a.z, (unsigned char)a.w);
}

__device__ static inline float4 uchar4_to_float4(const uchar4& a)
{
    return make_float4((float)a.x, (float)a.y, (float)a.z, (float)a.w);
}

__device__ static inline float dot(const float3& a, const float3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ static inline float dot(const float2& a, const float2& b)
{
    return a.x * b.x + a.y * b.y;
}

__device__ static inline float size(const float3& a)
{
    return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}

__device__ static inline float size(const float2& a)
{
    return sqrtf(a.x * a.x + a.y * a.y);
}

__device__ static inline float dist(const float3& a, const float3& b)
{
    float3 ab = a - b;
    return size(ab);
}

__device__ static inline float dist(const float2& a, const float2& b)
{
    float2 ab;
    ab.x = a.x - b.x;
    ab.y = a.y - b.y;
    return size(ab);
}

__device__ static inline float3 cross(const float3& a, const float3& b)
{
    return make_float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

__device__ static inline void normalize(float3& a)
{
    float d = sqrtf(dot(a, a));
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

__device__ static inline void normalize(float2& a)
{
    float d = sqrtf(dot(a, a));
    a.x /= d;
    a.y /= d;
}

__device__ static inline float3 M3x3mulV3( const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__device__ static inline float3 M3x3mulV2( const float* M3x3, const float2& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6], M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7],
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8]);
}

__device__ static inline float3 M3x4mulV3(const float* M3x4, const float3& V)
{
    return make_float3(M3x4[0] * V.x + M3x4[3] * V.y + M3x4[6] * V.z + M3x4[9],
                       M3x4[1] * V.x + M3x4[4] * V.y + M3x4[7] * V.z + M3x4[10],
                       M3x4[2] * V.x + M3x4[5] * V.y + M3x4[8] * V.z + M3x4[11]);
}

__device__ static inline float2 V2M3x3mulV2(float* M3x3, float2& V)
{
    float d = M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8];
    return make_float2((M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6]) / d, (M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7]) / d);
}

} // namespace depthMap
} // namespace aliceVision
