// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cuda_runtime.h>

#include <aliceVision/depthMap/cuda/images/gauss_filter.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_operators.h>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/planeSweeping/device_utils.h>
#include <aliceVision/system/Logger.hpp>

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
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius);

/*********************************************************************************
 * exported host function
 *********************************************************************************/
__host__ void ps_create_gaussian_arr( int deviceId, int scales ) // float delta, int radius)
{
    if( scales >= MAX_CONSTANT_GAUSS_SCALES )
    {
        ALICEVISION_LOG_ERROR( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    cudaError_t err;

    if( d_gaussianArrayInitialized.find( deviceId ) != d_gaussianArrayInitialized.end() ) return;

    d_gaussianArrayInitialized.insert( deviceId );

    int*   h_gaussianArrayOffset;
    float* h_gaussianArray;
    err = cudaMallocHost( &h_gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES * sizeof(int) );
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_ERROR( "Failed to allocate " << MAX_CONSTANT_GAUSS_SCALES * sizeof(int) << " of CUDA host memory." );
        throw std::runtime_error( "Failed to allocate CUDA host memory." );
    }
    err = cudaMallocHost( &h_gaussianArray,       MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float) );
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_ERROR( "Failed to allocate " << MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float) << " of CUDA host memory." );
        throw std::runtime_error( "Failed to allocate CUDA host memory." );
    }

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
        ALICEVISION_LOG_ERROR( "Programming error: too little memory allocated for " << MAX_CONSTANT_GAUSS_SCALES << " Gaussian kernels. Enlarge and recompile." );
        throw std::runtime_error( "Programming error: too little memory allocated for Gaussian kernels. Enlarge and recompile." );
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
                              MAX_CONSTANT_GAUSS_SCALES * sizeof(int), 0, cudaMemcpyDeviceToDevice);
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_ERROR( "Failed to move Gaussian filter to symbol, " << cudaGetErrorString(err) );
        throw std::runtime_error( "Failed to move Gaussian filter to symbol." );
    }

    err = cudaMemcpyToSymbol( d_gaussianArray,
                              h_gaussianArray,
                              sum_sizes * sizeof(float), 0, cudaMemcpyDeviceToDevice);
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_ERROR( "Failed to move Gaussian filter to symbol, " << cudaGetErrorString(err) );
        throw std::runtime_error( "Failed to move Gaussian filter to symbol." );

    }

    cudaFreeHost( h_gaussianArrayOffset );
    cudaFreeHost( h_gaussianArray );
}

__host__ void ps_downscale_gauss( Pyramid& ps_texs_arr,
                                  int camId, int scale,
                                  int w, int h, int radius )
{
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(w / (scale + 1), block.x), divUp(h / (scale + 1), block.y), 1);

    downscale_gauss_smooth_lab_kernel
        <<<grid, block>>>
        ( ps_texs_arr[camId][0].tex,
          ps_texs_arr[camId][scale].arr->getBuffer(),
          ps_texs_arr[camId][scale].arr->getPitch(),
          w / (scale + 1), h / (scale + 1), scale + 1,
          radius //, 15.5f
          );
}
/*********************************************************************************
 * kernel definitions
 *********************************************************************************/

__device__ inline float getGauss( int scale, int idx )
{
    return d_gaussianArray[d_gaussianArrayOffset[scale] + idx];
}

__global__ void downscale_gauss_smooth_lab_kernel(
    cudaTextureObject_t rc_tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float4 t = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sum = 0.0f;
        for(int i = -radius; i <= radius; i++)
        {
            for(int j = -radius; j <= radius; j++)
            {
                float4 curPix = 255.0f * tex2D<float4>(rc_tex, (float)(x * scale + j) + (float)scale / 2.0f,
                                               (float)(y * scale + i) + (float)scale / 2.0f);
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

        uchar4 tu4 = make_uchar4((unsigned char)t.x, (unsigned char)t.y, (unsigned char)t.z, (unsigned char)t.w);

        BufPtr<uchar4>(texLab, texLab_p).at(x,y) = tu4;
    }
}

} // namespace depthMap
} // namespace aliceVision

