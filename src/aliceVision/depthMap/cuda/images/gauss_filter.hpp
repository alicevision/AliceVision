// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.h>

#include <set>

namespace aliceVision {
namespace depthMap {

#define MAX_CONSTANT_GAUSS_SCALES   10
#define MAX_CONSTANT_GAUSS_MEM_SIZE 128

/*********************************************************************************
* global / constant data structures
*********************************************************************************/
extern std::set<int>                 d_gaussianArrayInitialized;
extern __device__ __constant__ int   d_gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES];
extern __device__ __constant__ float d_gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE];

__device__ inline float getGauss(int scale, int idx)
{
    return d_gaussianArray[d_gaussianArrayOffset[scale] + idx];
}

extern void ps_create_gaussian_arr( int deviceId, int scales );

extern void ps_downscale_gauss( Pyramid& pyramid,
                                int scale,
                                int w, int h, int radius,
                                cudaStream_t stream );


#ifdef ALICEVISION_TMP_WITH_BILATERALFILTER

//Euclidean Distance (x, y, d) = exp((|x - y| / d)^2 / 2)
template<class Type>
__device__ inline float euclideanLen(Type a, Type b, float d);

template<>
__device__ inline float euclideanLen<float>(float a, float b, float d)
{

    float mod = (b - a) * (b - a);

    return __expf(-mod / (2.f * d * d));
}

template<>
__device__ inline float euclideanLen<float4>(float4 a, float4 b, float d)
{

    float mod = (b.x - a.x) * (b.x - a.x) +
        (b.y - a.y) * (b.y - a.y) +
        (b.z - a.z) * (b.z - a.z);

    return __expf(-mod / (2.f * d * d));
}

template<class Type>
__device__ inline Type init_type(float v);

template<>
__device__ inline float init_type<float>(float v)
{
    return v;
}
template<>
__device__ inline float2 init_type<float2>(float v)
{
    return make_float2(v, v);
}
template<>
__device__ inline float3 init_type<float3>(float v)
{
    return make_float3(v, v, v);
}
template<>
__device__ inline float4 init_type<float4>(float v)
{
    return make_float4(v, v, v, v);
}


/**
 * Bilateral filter is an edge-preserving nonlinear smoothing filter. There
 * are three parameters distribute to the filter: gaussian delta, euclidean
 * delta and iterations.
 *
 * When the euclidean delta increases, most of the fine texture will be
 * filtered away, yet all contours are as crisp as in the original image.
 * If the euclidean delta approximates to infinity, the filter becomes a normal
 * gaussian filter. Fine texture will blur more with larger gaussian delta.
 * Multiple iterations have the effect of flattening the colors in an
 * image considerably, but without blurring edges, which produces a cartoon
 * effect.
 *
 * To learn more details about this filter, please view C. Tomasi's "Bilateral
 * Filtering for Gray and Color Images".
 */
template<class Type>
__global__ void bilateralFilter_kernel(
    cudaTextureObject_t rgbaTex,
    Type* texLab, int texLab_p,
    int width, int height,
    float euclideanDelta,
    int radius, int scale)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if ((x >= width) || (y >= height))
        return;

    Type centerPix = tex2D<Type>(rgbaTex, x, y);

    float s = 0.5f;
    Type t = init_type<Type>(0.0f); // generic make_float4();
    float sum = 0.0f;
    for (int i = -radius; i <= radius; i++)
    {
        for (int j = -radius; j <= radius; j++)
        {
            Type curPix = tex2D<Type>(rgbaTex, (float)(x * scale + j) + s, (float)(y * scale + i) + s);
            float factor = getGauss(scale - 1, i + radius) // domain factor
                            * getGauss(scale - 1, j + radius) // domain factor
                            * euclideanLen(curPix, centerPix, euclideanDelta); //range factor
            t = t + curPix * factor;
            sum += factor;
        }
    }
    t /= sum;

    BufPtr<Type>(texLab, texLab_p).at(x, y) = t;
}

template<class Type>
__host__ void ps_bilateralFilter(
    cudaTextureObject_t rgbaTex,
    CudaDeviceMemoryPitched<Type, 2>& img,
    float euclideanDelta,
    int radius)
{
    int scale = 1;
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(img.getSize()[0], block.x), divUp(img.getSize()[1], block.y), 1);

    bilateralFilter_kernel
        <<<grid, block>>>
        (rgbaTex,
            img.getBuffer(), img.getPitch(),
            img.getSize()[0], img.getSize()[1],
            euclideanDelta,
            radius, scale
            );
}
#endif

__host__ void ps_medianFilter3(
    cudaTextureObject_t tex,
    CudaDeviceMemoryPitched<float, 2>& img);


} // namespace depthMap
} // namespace aliceVision

