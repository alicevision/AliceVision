#pragma once

// Generally useful utility functions for cuda, written from scratch to replace the cutil library for small projects

#include <cuda_runtime.h>
#include <math.h>
#include <cstdio>
#include <cstdlib>


// Enable run time assertion checking in kernel code
#define cudaAssert(condition) if (!(condition)) { printf("ASSERT: %s %s\n", #condition, __FILE__); }

#define cudaSafeCall(err)  _internal_cudaSafeCall(err,__FILE__,__LINE__)


// Adapted from the G3D innovation engine's debugAssert.h
#    if defined(_MSC_VER) 
#       define rawBreak()  __debugbreak();
#    elif defined(__i386__)
// gcc on intel
#       define rawBreak() __asm__ __volatile__ ( "int $3" ); 
#    else
// some other gcc
#      define rawBreak() ::abort()
#   endif

inline void _internal_cudaSafeCall(cudaError err, const char *file, const int line){
    if (cudaSuccess != err) {
        printf("%s(%i) : cudaSafeCall() error: %s\n", file, line, cudaGetErrorString(err));
        rawBreak();
    }
}

// Cuda defines this for device only, provide missing host version.

#if !defined(__CUDACC__)
__inline__ __host__ float rsqrtf(float x) {
    return 1.0f / sqrtf(x);
}
#endif


#define CUDA_UTIL_FUNC __inline__ __host__ __device__

CUDA_UTIL_FUNC float2 make_float2(float x) {
    return make_float2(x, x);
}

CUDA_UTIL_FUNC float3 make_float3(float x) {
    return make_float3(x, x, x);
}

CUDA_UTIL_FUNC float4 make_float4(float x) {
    return make_float4(x, x, x, x);
}

/////////////// Scalar-wise vector add ////////////////////
CUDA_UTIL_FUNC float2 operator+(float2 v0, float2 v1) {
    return make_float2(v0.x + v1.x, v0.y + v1.y);
}

CUDA_UTIL_FUNC float3 operator+(float3 v0, float3 v1) {
    return make_float3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}

CUDA_UTIL_FUNC float4 operator+(float4 v0, float4 v1) {
    return make_float4(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w);
}

/////////////// Scalar-wise vector subtract ////////////////////
CUDA_UTIL_FUNC float2 operator-(float2 v0, float2 v1) {
    return make_float2(v0.x - v1.x, v0.y - v1.y);
}

CUDA_UTIL_FUNC float3 operator-(float3 v0, float3 v1) {
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}

CUDA_UTIL_FUNC float4 operator-(float4 v0, float4 v1) {
    return make_float4(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z, v0.w - v1.w);
}

/////////////// Scalar-wise vector multiply ////////////////////
CUDA_UTIL_FUNC float2 operator*(float2 v0, float2 v1) {
    return make_float2(v0.x*v1.x, v0.y*v1.y);
}

CUDA_UTIL_FUNC float3 operator*(float3 v0, float3 v1) {
    return make_float3(v0.x*v1.x, v0.y*v1.y, v0.z*v1.z);
}

CUDA_UTIL_FUNC float4 operator*(float4 v0, float4 v1) {
    return make_float4(v0.x*v1.x, v0.y*v1.y, v0.z*v1.z, v0.w*v1.w);
}

/////////////// Scalar-wise vector divide ////////////////////
CUDA_UTIL_FUNC float2 operator/(float2 v0, float2 v1) {
    return make_float2(v0.x / v1.x, v0.y / v1.y);
}

CUDA_UTIL_FUNC float3 operator/(float3 v0, float3 v1) {
    return make_float3(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z);
}

CUDA_UTIL_FUNC float4 operator/(float4 v0, float4 v1) {
    return make_float4(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z, v0.w / v1.w);
}


/////////////// += ////////////////////
CUDA_UTIL_FUNC void operator+=(float2& v0, float2 v1) {
    v0.x += v1.x;
    v0.y += v1.y;
}

CUDA_UTIL_FUNC void operator+=(float3& v0, float3 v1) {
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
}

CUDA_UTIL_FUNC void operator+=(float4& v0, float4 v1) {
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
    v0.w += v1.w;
}


CUDA_UTIL_FUNC void operator+=(float2& v0, float x) {
    v0.x += x;
    v0.y += x;
}

CUDA_UTIL_FUNC void operator+=(float3& v0, float x) {
    v0.x += x;
    v0.y += x;
    v0.z += x;
}

CUDA_UTIL_FUNC void operator+=(float4& v0, float x) {
    v0.x += x;
    v0.y += x;
    v0.z += x;
    v0.w += x;
}


/////////////// -= ////////////////////
CUDA_UTIL_FUNC void operator-=(float2& v0, float2 v1) {
    v0.x -= v1.x;
    v0.y -= v1.y;
}

CUDA_UTIL_FUNC void operator-=(float3& v0, float3 v1) {
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
}

CUDA_UTIL_FUNC void operator-=(float4& v0, float4 v1) {
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
    v0.w -= v1.w;
}


CUDA_UTIL_FUNC void operator-=(float2& v0, float x) {
    v0.x -= x;
    v0.y -= x;
}

CUDA_UTIL_FUNC void operator-=(float3& v0, float x) {
    v0.x -= x;
    v0.y -= x;
    v0.z -= x;
}

CUDA_UTIL_FUNC void operator-=(float4& v0, float x) {
    v0.x -= x;
    v0.y -= x;
    v0.z -= x;
    v0.w -= x;
}



/////////////// Multiply by a scalar ////////////////////
CUDA_UTIL_FUNC float2 operator*(float x, float2 v) {
    return make_float2(v.x*x, v.y*x);
}

CUDA_UTIL_FUNC float3 operator*(float x, float3 v) {
    return make_float3(v.x*x, v.y*x, v.z*x);
}

CUDA_UTIL_FUNC float4 operator*(float x, float4 v) {
    return make_float4(v.x*x, v.y*x, v.z*x, v.w*x);
}

CUDA_UTIL_FUNC float2 operator*(float2 v, float x) {
    return make_float2(v.x*x, v.y*x);
}

CUDA_UTIL_FUNC float3 operator*(float3 v, float x) {
    return make_float3(v.x*x, v.y*x, v.z*x);
}

CUDA_UTIL_FUNC float4 operator*(float4 v, float x) {
    return make_float4(v.x*x, v.y*x, v.z*x, v.w*x);
}

/////////////// Divide with a scalar ////////////////////
CUDA_UTIL_FUNC float2 operator/(float x, float2 v) {
    return make_float2(x / v.x, x / v.y);
}

CUDA_UTIL_FUNC float3 operator/(float x, float3 v) {
    return make_float3(x / v.x, x / v.y, x / v.z);
}

CUDA_UTIL_FUNC float4 operator/(float x, float4 v) {
    return make_float4(x / v.x, x / v.y, x / v.z, x / v.w);
}

CUDA_UTIL_FUNC float2 operator/(float2 v, float x) {
    return make_float2(v.x / x, v.y / x);
}

CUDA_UTIL_FUNC float3 operator/(float3 v, float x) {
    return make_float3(v.x / x, v.y / x, v.z / x);
}

CUDA_UTIL_FUNC float4 operator/(float4 v, float x) {
    return make_float4(v.x/x, v.y/x, v.z/x, v.w/x);
}



CUDA_UTIL_FUNC float dot(float2 v0, float2 v1) {
    return v0.x*v1.x + v0.y*v1.y;
}

CUDA_UTIL_FUNC float dot(float3 v0, float3 v1) {
    return v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;
}

CUDA_UTIL_FUNC float dot(float4 v0, float4 v1) {
    return v0.x*v1.x + v0.y*v1.y + v0.z*v1.z + v0.w*v1.w;
}

CUDA_UTIL_FUNC float length(float2 v) {
    return sqrtf(dot(v, v));
}

CUDA_UTIL_FUNC float length(float3 v) {
    return sqrtf(dot(v, v));
}

CUDA_UTIL_FUNC float length(float4 v) {
    return sqrtf(dot(v, v));
}


CUDA_UTIL_FUNC float2 normalize(float2 v) {
    return v*rsqrtf(dot(v, v));
}

CUDA_UTIL_FUNC float3 normalize(float3 v) {
    return v*rsqrtf(dot(v, v));
}

CUDA_UTIL_FUNC float4 normalize(float4 v) {
    return v*rsqrtf(dot(v, v));
}


////////////// int stuff


/////////////// Scalar-wise vector add ////////////////////
CUDA_UTIL_FUNC int2 operator+(int2 v0, int2 v1) {
    return make_int2(v0.x + v1.x, v0.y + v1.y);
}

CUDA_UTIL_FUNC int3 operator+(int3 v0, int3 v1) {
    return make_int3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}

CUDA_UTIL_FUNC int4 operator+(int4 v0, int4 v1) {
    return make_int4(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w);
}

#undef CUDA_UTIL_FUNC



#if defined(__CUDA_ARCH__)
#define __CONDITIONAL_UNROLL__ #pragma unroll
#else
#define __CONDITIONAL_UNROLL__
#endif
