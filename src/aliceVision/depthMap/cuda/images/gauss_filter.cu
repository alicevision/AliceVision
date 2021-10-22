// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cuda_runtime.h>

#include "gauss_filter.hpp"
#include <aliceVision/depthMap/cuda/deviceCommon/device_operators.cuh>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.cuh>
#include <aliceVision/depthMap/cuda/commonStructures.hpp>


namespace aliceVision {
namespace depthMap {

/*********************************************************************************
* global / constant data structures
*********************************************************************************/
std::set<int>                 d_gaussianArrayInitialized;
__device__ __constant__ int   d_gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES];
__device__ __constant__ float d_gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE];

/*********************************************************************************
 * kernel forward declarations
 *********************************************************************************/
__global__ void downscale_gauss_smooth_lab_kernel(
    cudaTextureObject_t rc_tex,
    CudaRGBA* texLab, int texLab_p,
    int width, int height, int scale, int radius);

/*********************************************************************************
 * exported host function
 *********************************************************************************/
__host__ void ps_create_gaussian_arr( int deviceId, int scales ) // float delta, int radius)
{
    if( scales >= MAX_CONSTANT_GAUSS_SCALES )
    {
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    cudaError_t err;

    if( d_gaussianArrayInitialized.find( deviceId ) != d_gaussianArrayInitialized.end() ) return;

    d_gaussianArrayInitialized.insert( deviceId );

    int*   h_gaussianArrayOffset;
    float* h_gaussianArray;
    err = cudaMallocHost( &h_gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES * sizeof(int) );
    THROW_ON_CUDA_ERROR(err, "Failed to allocate " << MAX_CONSTANT_GAUSS_SCALES * sizeof(int) << " of CUDA host memory."); 

    err = cudaMallocHost( &h_gaussianArray,       MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float) );
    THROW_ON_CUDA_ERROR(err, "Failed to allocate " << MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float) << " of CUDA host memory.");

    int sum_sizes = 0;
    for( int scale=0; scale<MAX_CONSTANT_GAUSS_SCALES; scale++ )
    {
        h_gaussianArrayOffset[scale] = sum_sizes;
        const int   radius = scale + 1;
        const int   size   = 2 * radius + 1;
        sum_sizes += size;
    }

    if( sum_sizes >= MAX_CONSTANT_GAUSS_MEM_SIZE )
    {
        throw std::runtime_error( "Programming error: too little memory allocated for " 
            + std::to_string(MAX_CONSTANT_GAUSS_SCALES) + " Gaussian kernels. Enlarge and recompile." );
    }

    for( int scale=0; scale<MAX_CONSTANT_GAUSS_SCALES; scale++ )
    {
        const int   radius = scale + 1;
        const float delta  = 1.0f;
        const int   size   = 2 * radius + 1;

        for( int idx=0; idx<size; idx++ )
        {
            int x = idx - radius;
            h_gaussianArray[h_gaussianArrayOffset[scale]+idx] = expf(-(x * x) / (2 * delta * delta));
        }

        // generate gaussian array
    }


    // create cuda array
    err = cudaMemcpyToSymbol( d_gaussianArrayOffset,
                              h_gaussianArrayOffset,
                              MAX_CONSTANT_GAUSS_SCALES * sizeof(int), 0, cudaMemcpyHostToDevice);

    THROW_ON_CUDA_ERROR(err, "Failed to move Gaussian filter to symbol.");

    err = cudaMemcpyToSymbol( d_gaussianArray,
                              h_gaussianArray,
                              sum_sizes * sizeof(float), 0, cudaMemcpyHostToDevice);

    THROW_ON_CUDA_ERROR(err, "Failed to move Gaussian filter to symbol." );

    cudaFreeHost( h_gaussianArrayOffset );
    cudaFreeHost( h_gaussianArray );
}

__host__ void ps_downscale_gauss( Pyramid& pyramid,
                                  int scale,
                                  int w, int h, int radius,
                                  cudaStream_t stream )
{
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(w / (scale + 1), block.x), divUp(h / (scale + 1), block.y), 1);

    downscale_gauss_smooth_lab_kernel
        <<<grid, block, 0, stream>>>
        ( pyramid[0].tex,
          pyramid[scale].arr->getBuffer(),
          pyramid[scale].arr->getPitch(),
          w / (scale + 1), h / (scale + 1), scale + 1,
          radius
          );
}
/*********************************************************************************
 * kernel definitions
 *********************************************************************************/

/* This is a bad Gaussian filter implementation - the Gaussian filter is separable. */
__global__ void downscale_gauss_smooth_lab_kernel(
    cudaTextureObject_t rc_tex,
    CudaRGBA* texLab, int texLab_p,
    int width, int height, int scale, int radius)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float s = (float)scale * 0.5f;
        float4 t = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sum = 0.0f;
        for(int i = -radius; i <= radius; i++)
        {
            for(int j = -radius; j <= radius; j++)
            {
                float4 curPix = tex2D_float4(rc_tex, (float)(x * scale + j) + s,
                                               (float)(y * scale + i) + s);
                float factor = getGauss( scale-1, i + radius )
                             * getGauss( scale-1, j + radius ); // domain factor
                t = t + curPix * factor;
                sum += factor;
            }
        }
        t.x = t.x / sum;
        t.y = t.y / sum;
        t.z = t.z / sum;
        t.w = t.w / sum;

        CudaRGBA& out = BufPtr<CudaRGBA>(texLab, texLab_p).at(x,y);
        out.x = t.x;
        out.y = t.y;
        out.z = t.z;
        out.w = t.w;
    }
}

__device__ void cuda_swap_float(float& a, float& b)
{
    float temp = a;
    a = b;
    b = temp;
}

/**
* @warning: use an hardcoded buffer size, so max radius value is 3.
*/
__global__ void medianFilter3_kernel(
    cudaTextureObject_t tex,
    float* texLab, int texLab_p,
    int width, int height,
    int scale)
{
    const int radius = 3;
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if ((x >= width - radius) || (y >= height - radius) ||
        (x < radius) || (y < radius))
        return;

    const int filterWidth = radius * 2 + 1;
    const int filterNbPixels = filterWidth * filterWidth;

    float buf[filterNbPixels]; // filterNbPixels

    // Assign masked values to buf
    for (int yi = 0; yi < filterWidth; ++yi)
    {
        for (int xi = 0; xi < filterWidth; ++xi)
        {
            float pix = tex2D<float>(tex, x + xi - radius, y + yi - radius);
            buf[yi * filterWidth + xi] = pix;
        }
    }

    // Calculate until we get the median value
    for (int k = 0; k < filterNbPixels; ++k) // (filterNbPixels + 1) / 2
        for (int l = 0; l < filterNbPixels; ++l)
            if (buf[k] < buf[l])
                cuda_swap_float(buf[k], buf[l]);

    BufPtr<float>(texLab, texLab_p).at(x, y) = buf[radius * filterWidth + radius];
}


__host__ void ps_medianFilter3(
    cudaTextureObject_t tex,
    CudaDeviceMemoryPitched<float, 2>& img)
{
    int scale = 1;
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(img.getSize()[0], block.x), divUp(img.getSize()[1], block.y), 1);

    medianFilter3_kernel
        <<<grid, block>>>
        (tex,
            img.getBuffer(), img.getPitch(),
            img.getSize()[0], img.getSize()[1],
            scale
            );
}


} // namespace depthMap
} // namespace aliceVision

