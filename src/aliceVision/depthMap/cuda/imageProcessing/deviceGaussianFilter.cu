// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceGaussianFilter.hpp"

#include <aliceVision/depthMap/cuda/host/divUp.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
#include <aliceVision/depthMap/cuda/device/operators.cuh>

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

/*********************************************************************************
* global / constant data structures
*********************************************************************************/
std::set<int>                 d_gaussianArrayInitialized;
__device__ __constant__ int   d_gaussianArrayOffset[MAX_CONSTANT_GAUSS_SCALES];
__device__ __constant__ float d_gaussianArray[MAX_CONSTANT_GAUSS_MEM_SIZE];

/*********************************************************************************
 * device functions definitions
 *********************************************************************************/

__device__ void cuda_swap_float(float& a, float& b)
{
    float temp = a;
    a = b;
    b = temp;
}

/*********************************************************************************
 * kernel definitions
 *********************************************************************************/

/*
 * @note This kernel implementation is not optimized because the Gaussian filter is separable.
 */
__global__ void downscaleWithGaussianBlur_kernel(cudaTextureObject_t in_img_tex,
                                                 CudaRGBA* out_downscaledImg_d, int out_downscaledImg_p,
                                                 unsigned int downscaledImgWidth,
                                                 unsigned int downscaledImgHeight,
                                                 int downscale, 
                                                 int gaussRadius)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < downscaledImgWidth) && (y < downscaledImgHeight))
    {
        const float s = float(downscale) * 0.5f;

        float4 accPix = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sumFactor = 0.0f;

        for(int i = -gaussRadius; i <= gaussRadius; i++)
        {
            for(int j = -gaussRadius; j <= gaussRadius; j++)
            {
                const float4 curPix = tex2D_float4(in_img_tex, float(x * downscale + j) + s, float(y * downscale + i) + s);
                const float factor = getGauss(downscale - 1, i + gaussRadius) *
                                     getGauss(downscale - 1, j + gaussRadius); // domain factor

                accPix = accPix + curPix * factor;
                sumFactor += factor;
            }
        }

        CudaRGBA& out = BufPtr<CudaRGBA>(out_downscaledImg_d, out_downscaledImg_p).at(size_t(x), size_t(y));
        out.x = accPix.x / sumFactor;
        out.y = accPix.y / sumFactor;
        out.z = accPix.z / sumFactor;
        out.w = accPix.w / sumFactor;
    }
}

__global__ void gaussianBlurVolumeZ_kernel(float* out_volume_d, int out_volume_s, int out_volume_p, 
                                           const float* in_volume_d, int in_volume_s, int in_volume_p, 
                                           int volDimX, int volDimY, int volDimZ, int gaussRadius)
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z;

    const int gaussScale = gaussRadius - 1;

    if(vx >= volDimX || vy >= volDimY)
        return;

    float sum = 0.0f;
    float sumFactor = 0.0f;

    for(int rz = -gaussRadius; rz <= gaussRadius; rz++)
    {
        const int iz = vz + rz;
        if((iz < volDimZ) && (iz > 0))
        {
            const float value = float(*get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, iz));
            const float factor = getGauss(gaussScale, rz + gaussRadius);
            sum += value * factor;
            sumFactor += factor;
        }
    }

    *get3DBufferAt(out_volume_d, out_volume_s, out_volume_p, vx, vy, vz) = float(sum / sumFactor);
}

__global__ void gaussianBlurVolumeXYZ_kernel(float* out_volume_d, int out_volume_s, int out_volume_p,
                                             const float* in_volume_d, int in_volume_s, int in_volume_p,
                                             int volDimX, int volDimY, int volDimZ, int gaussRadius)
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z;

    const int gaussScale = gaussRadius - 1;

    if(vx >= volDimX || vy >= volDimY)
        return;

    const int xMinRadius = max(-gaussRadius, -vx);
    const int yMinRadius = max(-gaussRadius, -vy);
    const int zMinRadius = max(-gaussRadius, -vz);

    const int xMaxRadius = min(gaussRadius, volDimX - vx - 1);
    const int yMaxRadius = min(gaussRadius, volDimY - vy - 1);
    const int zMaxRadius = min(gaussRadius, volDimZ - vz - 1);

    float sum = 0.0f;
    float sumFactor = 0.0f;

    for(int rx = xMinRadius; rx <= xMaxRadius; rx++)
    {
        const int ix = vx + rx;

        for(int ry = yMinRadius; ry <= yMaxRadius; ry++)
        {
            const int iy = vy + ry;

            for(int rz = zMinRadius; rz <= zMaxRadius; rz++)
            {
                const int iz = vz + rz;
   
                const float value = float(*get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, ix, iy, iz));
                const float factor = getGauss(gaussScale, rx + gaussRadius) * getGauss(gaussScale, ry + gaussRadius) * getGauss(gaussScale, rz + gaussRadius);
                sum += value * factor;
                sumFactor += factor;
            }
        }
    }

    *get3DBufferAt(out_volume_d, out_volume_s, out_volume_p, vx, vy, vz) = float(sum / sumFactor);
}

/**
 * @warning: use an hardcoded buffer size, so max radius value is 3.
 */
__global__ void medianFilter3_kernel(cudaTextureObject_t tex, float* texLab_d, int texLab_p, int width, int height, int scale)
{
    const int radius = 3;
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x >= width - radius) || (y >= height - radius) || (x < radius) || (y < radius))
        return;

    const int filterWidth = radius * 2 + 1;
    const int filterNbPixels = filterWidth * filterWidth;

    float buf[filterNbPixels]; // filterNbPixels

    // Assign masked values to buf
    for(int yi = 0; yi < filterWidth; ++yi)
    {
        for(int xi = 0; xi < filterWidth; ++xi)
        {
            float pix = tex2D<float>(tex, x + xi - radius, y + yi - radius);
            buf[yi * filterWidth + xi] = pix;
        }
    }

    // Calculate until we get the median value
    for(int k = 0; k < filterNbPixels; ++k) // (filterNbPixels + 1) / 2
        for(int l = 0; l < filterNbPixels; ++l)
            if(buf[k] < buf[l])
                cuda_swap_float(buf[k], buf[l]);

    BufPtr<float>(texLab_d, texLab_p).at(x, y) = buf[radius * filterWidth + radius];
}

/*********************************************************************************
 * exported host function
 *********************************************************************************/
__host__ void cuda_createConstantGaussianArray(int cudaDeviceId, int scales) // float delta, int radius)
{
    if(scales >= MAX_CONSTANT_GAUSS_SCALES)
    {
        throw std::runtime_error( "Programming error: too few scales pre-computed for Gaussian kernels. Enlarge and recompile." );
    }

    cudaError_t err;

    if(d_gaussianArrayInitialized.find(cudaDeviceId) != d_gaussianArrayInitialized.end())
        return;

    d_gaussianArrayInitialized.insert(cudaDeviceId);

    int*   h_gaussianArrayOffset;
    float* h_gaussianArray;

    err = cudaMallocHost(&h_gaussianArrayOffset, MAX_CONSTANT_GAUSS_SCALES * sizeof(int));
    THROW_ON_CUDA_ERROR(err, "Failed to allocate " << MAX_CONSTANT_GAUSS_SCALES * sizeof(int) << " of CUDA host memory."); 

    err = cudaMallocHost(&h_gaussianArray, MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float));
    THROW_ON_CUDA_ERROR(err, "Failed to allocate " << MAX_CONSTANT_GAUSS_MEM_SIZE * sizeof(float) << " of CUDA host memory.");

    int sumSizes = 0;

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        h_gaussianArrayOffset[scale] = sumSizes;
        const int radius = scale + 1;
        const int size = 2 * radius + 1;
        sumSizes += size;
    }

    if(sumSizes >= MAX_CONSTANT_GAUSS_MEM_SIZE)
    {
        throw std::runtime_error( "Programming error: too little memory allocated for " 
            + std::to_string(MAX_CONSTANT_GAUSS_SCALES) + " Gaussian kernels. Enlarge and recompile." );
    }

    for(int scale = 0; scale < MAX_CONSTANT_GAUSS_SCALES; ++scale)
    {
        const int radius = scale + 1;
        const float delta  = 1.0f;
        const int size   = 2 * radius + 1;

        for(int idx = 0; idx < size; idx++)
        {
            int x = idx - radius;
            h_gaussianArray[h_gaussianArrayOffset[scale]+idx] = expf(-(x * x) / (2 * delta * delta));
        }
    }

    // create cuda array
    err = cudaMemcpyToSymbol( d_gaussianArrayOffset,
                              h_gaussianArrayOffset,
                              MAX_CONSTANT_GAUSS_SCALES * sizeof(int), 0, cudaMemcpyHostToDevice);

    THROW_ON_CUDA_ERROR(err, "Failed to move Gaussian filter to symbol.");

    err = cudaMemcpyToSymbol(d_gaussianArray,
                             h_gaussianArray,
                             sumSizes * sizeof(float), 0, cudaMemcpyHostToDevice);

    THROW_ON_CUDA_ERROR(err, "Failed to move Gaussian filter to symbol." );

    cudaFreeHost(h_gaussianArrayOffset);
    cudaFreeHost(h_gaussianArray);
}

__host__ void cuda_downscaleWithGaussianBlur(CudaDeviceMemoryPitched<CudaRGBA, 2>& out_downscaledImg_dmp,
                                             cudaTextureObject_t in_img_tex,
                                             int downscale, 
                                             int gaussRadius,
                                             cudaStream_t stream)
{
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(out_downscaledImg_dmp.getSize().x(), block.x), divUp(out_downscaledImg_dmp.getSize().y(), block.y), 1);

    downscaleWithGaussianBlur_kernel<<<grid, block, 0, stream>>>(
          in_img_tex,
          out_downscaledImg_dmp.getBuffer(),
          out_downscaledImg_dmp.getPitch(),
          (unsigned int)(out_downscaledImg_dmp.getSize().x()),
          (unsigned int)(out_downscaledImg_dmp.getSize().y()),
          downscale,
          gaussRadius);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_gaussianBlurVolumeZ(CudaDeviceMemoryPitched<float, 3>& inout_volume_dmp, int gaussRadius, cudaStream_t stream)
{
    const CudaSize<3>& volDim = inout_volume_dmp.getSize();
    CudaDeviceMemoryPitched<float, 3> volSmoothZ_dmp(volDim);

    const dim3 block(32, 1, 1);
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

    gaussianBlurVolumeZ_kernel<<<grid, block, 0, stream>>>(
        volSmoothZ_dmp.getBuffer(), 
        volSmoothZ_dmp.getBytesPaddedUpToDim(1), 
        volSmoothZ_dmp.getBytesPaddedUpToDim(0), 
        inout_volume_dmp.getBuffer(), 
        inout_volume_dmp.getBytesPaddedUpToDim(1), 
        inout_volume_dmp.getBytesPaddedUpToDim(0), 
        int(volDim.x()), 
        int(volDim.y()), 
        int(volDim.z()), 
        gaussRadius);

    inout_volume_dmp.copyFrom(volSmoothZ_dmp);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_gaussianBlurVolumeXYZ(CudaDeviceMemoryPitched<float, 3>& inout_volume_dmp, int gaussRadius, cudaStream_t stream)
{
    const CudaSize<3>& volDim = inout_volume_dmp.getSize();
    CudaDeviceMemoryPitched<float, 3> volSmoothXYZ_dmp(volDim);

    const dim3 block(32, 1, 1);
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

    gaussianBlurVolumeXYZ_kernel<<<grid, block, 0, stream>>>(
        volSmoothXYZ_dmp.getBuffer(), 
        volSmoothXYZ_dmp.getBytesPaddedUpToDim(1), 
        volSmoothXYZ_dmp.getBytesPaddedUpToDim(0), 
        inout_volume_dmp.getBuffer(), 
        inout_volume_dmp.getBytesPaddedUpToDim(1), 
        inout_volume_dmp.getBytesPaddedUpToDim(0), 
        int(volDim.x()), 
        int(volDim.y()), 
        int(volDim.z()), 
        gaussRadius);

    inout_volume_dmp.copyFrom(volSmoothXYZ_dmp);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_medianFilter3(cudaTextureObject_t tex, CudaDeviceMemoryPitched<float, 2>& img)
{
    int scale = 1;
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(img.getSize()[0], block.x), divUp(img.getSize()[1], block.y), 1);

    medianFilter3_kernel<<<grid, block>>>(
            tex,
            img.getBuffer(), img.getPitch(),
            img.getSize()[0], img.getSize()[1],
            scale);

    CHECK_CUDA_ERROR();
}


} // namespace depthMap
} // namespace aliceVision

