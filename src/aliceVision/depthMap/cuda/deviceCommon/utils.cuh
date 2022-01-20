// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>
#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.h>

namespace aliceVision {
namespace depthMap {

/**
* @brief
* @param[int] ptr
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
__device__ static inline
T* get2DBufferAt(T* ptr, int pitch, int x, int y)
{
    return &(BufPtr<T>(ptr,pitch).at(x,y));
}

/**
* @brief
* @param[int] ptr
* @param[int] spitch raw length of a 2D array in bytes
* @param[int] pitch raw length of a line in bytes
* @param[int] x
* @param[int] y
* @return
*/
template <typename T>
__device__ static inline
T* get3DBufferAt(T* ptr, int spitch, int pitch, int x, int y, int z)
{
    return ((T*)(((char*)ptr) + z * spitch + y * pitch)) + x;
}

template <typename T>
__device__ static inline
const T* get3DBufferAt(const T* ptr, int spitch, int pitch, int x, int y, int z)
{
    return ((const T*)(((const char*)ptr) + z * spitch + y * pitch)) + x;
}

template <typename T>
__device__ static inline
T* get3DBufferAt(T* ptr, int spitch, int pitch, const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

template <typename T>
__device__ static inline
const T* get3DBufferAt(const T* ptr, int spitch, int pitch, const int3& v)
{
    return get3DBufferAt(ptr, spitch, pitch, v.x, v.y, v.z);
}

__device__ static inline
float multi_fminf(float a, float b, float c)
{
  return fminf(fminf(a, b), c);
}

__device__ static inline
float multi_fminf(float a, float b, float c, float d)
{
  return fminf(fminf(fminf(a, b), c), d);
}


#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

__device__ static inline
float4 tex2D_float4(cudaTextureObject_t rc_tex, float x, float y)
{
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
    // cudaReadNormalizedFloat
    float4 a = tex2D<float4>(rc_tex, x, y);
    return make_float4(a.x * 255.0f, a.y * 255.0f, a.z * 255.0f, a.w * 255.0f);
#else
    // cudaReadElementType
    uchar4 a = tex2D<uchar4>(rc_tex, x, y);
    return make_float4(a.x, a.y, a.z, a.w);
#endif
}

#else

__device__ static inline
float4 tex2D_float4(cudaTextureObject_t rc_tex, float x, float y)
{
    return tex2D<float4>(rc_tex, x, y);
}

#endif

} // namespace depthMap
} // namespace aliceVision

