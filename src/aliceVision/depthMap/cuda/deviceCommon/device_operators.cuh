// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

__device__ static inline
float4 operator*(const float4& a, const float& d)
{
    return make_float4(a.x * d, a.y * d, a.z * d, a.w * d);
}

__device__ static inline
float4 operator+(const float4& a, const float4& d)
{
    return make_float4(a.x + d.x, a.y + d.y, a.z + d.z, a.w + d.w);
}

__device__ static inline
float4 operator*(const float& d, const float4& a)
{
    return make_float4(a.x * d, a.y * d, a.z * d, a.w * d);
}

__device__ static inline
float3 operator*(const float3& a, const float& d)
{
    return make_float3(a.x * d, a.y * d, a.z * d);
}

__device__ static inline
float3 operator/(const float3& a, const float& d)
{
    return make_float3(a.x / d, a.y / d, a.z / d);
}

__device__ static inline
float3 operator+(const float3& a, const float3& b)
{
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ static inline
float3 operator-(const float3& a, const float3& b)
{
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ static inline
int2 operator+(const int2& a, const int2& b)
{
    return make_int2(a.x + b.x, a.y + b.y);
}

__device__ static inline
float2 operator*(const float2& a, const float& d)
{
    return make_float2(a.x * d, a.y * d);
}

__device__ static inline
float2 operator/(const float2& a, const float& d)
{
    return make_float2(a.x / d, a.y / d);
}

__device__ static inline
float2 operator+(const float2& a, const float2& b)
{
    return make_float2(a.x + b.x, a.y + b.y);
}

__device__ static inline
float2 operator-(const float2& a, const float2& b)
{
    return make_float2(a.x - b.x, a.y - b.y);
}

} // namespace depthMap
} // namespace aliceVision

