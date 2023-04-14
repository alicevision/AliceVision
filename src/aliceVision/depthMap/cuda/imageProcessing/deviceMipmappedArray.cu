// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceMipmappedArray.hpp"

#include <aliceVision/depthMap/cuda/host/divUp.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
#include <aliceVision/depthMap/cuda/device/operators.cuh>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>

#include <cuda_runtime.h>

namespace aliceVision {
namespace depthMap {

template<int TRadius>
__global__ void createMipmappedArrayLevel_kernel(cudaSurfaceObject_t out_currentLevel_surf,
                                                 cudaTextureObject_t in_previousLevel_tex,
                                                 unsigned int width,
                                                 unsigned int height)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    const float px = 1.f / float(width);
    const float py = 1.f / float(height);

    float4 sumColor = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
    float sumFactor = 0.0f;

#pragma unroll
    for(int i = -TRadius; i <= TRadius; i++)
    {

#pragma unroll
        for(int j = -TRadius; j <= TRadius; j++)
        {
            // domain factor
            const float factor = getGauss(1, i + TRadius) * getGauss(1, j + TRadius);

            // normalized coordinates
            const float u = (x + j + 0.5f) * px;
            const float v = (y + i + 0.5f) * py;

            // current pixel color
            const float4 color = tex2D_float4(in_previousLevel_tex, u, v);

            // sum color
            sumColor = sumColor + color * factor;

            // sum factor
            sumFactor += factor;
        }
    }

    const float4 color = sumColor / sumFactor;

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color to unsigned char
    CudaRGBA out;
    out.x = CudaColorBaseType(color.x);
    out.y = CudaColorBaseType(color.y);
    out.z = CudaColorBaseType(color.z);
    out.w = CudaColorBaseType(color.w);

    // write output color
    surf2Dwrite(out, out_currentLevel_surf, int(x * sizeof(CudaRGBA)), int(y));
#else // texture use float4 or half4
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    // convert color to half
    CudaRGBA out;
    out.x = __float2half(color.x);
    out.y = __float2half(color.y);
    out.z = __float2half(color.z);
    out.w = __float2half(color.w);

    // write output color
    // note: surf2Dwrite cannot write half directly
    surf2Dwrite(*(reinterpret_cast<ushort4*>(&(out))), out_currentLevel_surf, int(x * sizeof(ushort4)), int(y));
#else // texture use float4
     // write output color
    surf2Dwrite(color, out_currentLevel_surf, int(x * sizeof(float4)), int(y));
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}

/*
__global__ void createMipmappedArrayLevel_kernel(cudaSurfaceObject_t out_currentLevel_surf,
                                                 cudaTextureObject_t in_previousLevel_tex,
                                                 unsigned int width,
                                                 unsigned int height)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    // corresponding texture normalized coordinates
    const float u = (x + 0.5f) / float(width);
    const float v = (y + 0.5f) / float(height);

    // corresponding color in previous level texture
    const float4 color = tex2D_float4(in_previousLevel_tex, u, v);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color to unsigned char
    CudaRGBA out;
    out.x = CudaColorBaseType(color.x);
    out.y = CudaColorBaseType(color.y);
    out.z = CudaColorBaseType(color.z);
    out.w = CudaColorBaseType(color.w);

    // write output color
    surf2Dwrite(out, out_currentLevel_surf, int(x * sizeof(CudaRGBA)), int(y));
#else // texture use float4 or half4
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    // convert color to half
    CudaRGBA out;
    out.x = __float2half(color.x);
    out.y = __float2half(color.y);
    out.z = __float2half(color.z);
    out.w = __float2half(color.w);

    // write output color
    // note: surf2Dwrite cannot write half directly
    surf2Dwrite(*(reinterpret_cast<ushort4*>(&(out))), out_currentLevel_surf, int(x * sizeof(ushort4)), int(y));
#else // texture use float4
     // write output color
    surf2Dwrite(color, out_currentLevel_surf, int(x * sizeof(float4)), int(y));
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}
*/

__global__ void createMipmappedArrayDebugFlatImage_kernel(CudaRGBA* out_flatImage_d, int out_flatImage_p,
                                                          cudaTextureObject_t in_mipmappedArray_tex,
                                                          unsigned int levels,
                                                          unsigned int firstLevelWidth,
                                                          unsigned int firstLevelHeight)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(y >= firstLevelHeight)
        return;

    // set default color value
    float4 color = make_float4(0.f, 0.f, 0.f, 0.f);

    if(x < firstLevelWidth)
    {
        // level 0
        // corresponding texture normalized coordinates
        const float u = (x + 0.5f) / float(firstLevelWidth);
        const float v = (y + 0.5f) / float(firstLevelHeight);

        // set color value from mipmappedArray texture
        color = tex2DLod<float4>(in_mipmappedArray_tex, u, v, 0.f);
    }
    else
    {
        // level from global y coordinate
        const unsigned int level = int(log2(1.0 / (1.0 - (y / double(firstLevelHeight))))) + 1;
        const unsigned int levelDownscale = pow(2, level);
        const unsigned int levelWidth  = firstLevelWidth  / levelDownscale;
        const unsigned int levelHeight = firstLevelHeight / levelDownscale;

        // corresponding level coordinates
        const float lx = x - firstLevelWidth;
        const float ly = y % levelHeight;

        // corresponding texture normalized coordinates
        const float u = (lx + 0.5f) / float(levelWidth);
        const float v = (ly + 0.5f) / float(levelHeight);

        if(u <= 1.f && v <= 1.f && level < levels)
        {
            // set color value from mipmappedArray texture
            color = tex2DLod<float4>(in_mipmappedArray_tex, u, v, float(level));
        }
    }

    // write output color
    CudaRGBA* out_colorPtr = get2DBufferAt(out_flatImage_d, out_flatImage_p, x, y);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    // convert color from (0, 1) to (0, 255)
    color.x *= 255.f;
    color.y *= 255.f;
    color.z *= 255.f;
    color.w *= 255.f;

    out_colorPtr->x = CudaColorBaseType(color.x);
    out_colorPtr->y = CudaColorBaseType(color.y);
    out_colorPtr->z = CudaColorBaseType(color.z);
    out_colorPtr->w = CudaColorBaseType(color.w);
#else // texture use float4 or half4
    // convert color from (0, 255) to (0, 1)
    color.x /= 255.f;
    color.y /= 255.f;
    color.z /= 255.f;
    color.w /= 255.f;
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    out_colorPtr->x = __float2half(color.x);
    out_colorPtr->y = __float2half(color.y);
    out_colorPtr->z = __float2half(color.z);
    out_colorPtr->w = __float2half(color.w);
#else // texture use float4
    *out_colorPtr = color;
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
}

__host__ void cuda_createMipmappedArrayFromImage(cudaMipmappedArray_t* out_mipmappedArrayPtr,
                                                 const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp,
                                                 const unsigned int levels)
{
    const CudaSize<2>& in_imgSize = in_img_dmp.getSize();
    const cudaExtent imgSize = make_cudaExtent(in_imgSize.x(), in_imgSize.y(), 0);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
    const cudaChannelFormatDesc desc = cudaCreateChannelDescHalf4();
#else
    const cudaChannelFormatDesc desc = cudaCreateChannelDesc<CudaRGBA>();
#endif

    // allocate CUDA mipmapped array
    CHECK_CUDA_RETURN_ERROR(cudaMallocMipmappedArray(out_mipmappedArrayPtr, &desc, imgSize, levels));

    // get mipmapped array at level 0
    cudaArray_t level0;
    CHECK_CUDA_RETURN_ERROR(cudaGetMipmappedArrayLevel(&level0, *out_mipmappedArrayPtr, 0));

    // copy input image buffer into mipmapped array at level 0
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr.ptr = (void *)in_img_dmp.getBytePtr();
    copyParams.srcPtr.pitch = in_img_dmp.getPitch();
    copyParams.srcPtr.xsize = in_img_dmp.getUnitsInDim(0);
    copyParams.srcPtr.ysize = in_img_dmp.getUnitsInDim(1);
    copyParams.dstArray = level0;
    copyParams.extent = imgSize;
    copyParams.extent.depth = 1;
    copyParams.kind = cudaMemcpyDeviceToDevice;
    CHECK_CUDA_RETURN_ERROR(cudaMemcpy3D(&copyParams));

    // initialize each mipmapped array level from level 0
    size_t width  = in_imgSize.x();
    size_t height = in_imgSize.y();

    for(size_t l = 1; l < levels; ++l)
    {
        // current level width/height
        width  /= 2;
        height /= 2;

        // previous level array (or level 0)
        cudaArray_t previousLevelArray;
        CHECK_CUDA_RETURN_ERROR(cudaGetMipmappedArrayLevel(&previousLevelArray, *out_mipmappedArrayPtr, l - 1));

        // current level array
        cudaArray_t currentLevelArray;
        CHECK_CUDA_RETURN_ERROR(cudaGetMipmappedArrayLevel(&currentLevelArray, *out_mipmappedArrayPtr, l));

        // check current level array size
        cudaExtent currentLevelArraySize;
        CHECK_CUDA_RETURN_ERROR(cudaArrayGetInfo(nullptr, &currentLevelArraySize, nullptr, currentLevelArray));

        assert(currentLevelArraySize.width  == width);
        assert(currentLevelArraySize.height == height);
        assert(currentLevelArraySize.depth  == 0);

        // generate texture object for previous level reading
        cudaTextureObject_t previousLevel_tex;
        {
            cudaResourceDesc texRes;
            memset(&texRes, 0, sizeof(cudaResourceDesc));
            texRes.resType = cudaResourceTypeArray;
            texRes.res.array.array = previousLevelArray;

            cudaTextureDesc texDescr;
            memset(&texDescr, 0, sizeof(cudaTextureDesc));
            texDescr.normalizedCoords = 1;
            texDescr.filterMode = cudaFilterModeLinear;
            texDescr.addressMode[0] = cudaAddressModeClamp;
            texDescr.addressMode[1] = cudaAddressModeClamp;
            texDescr.addressMode[2] = cudaAddressModeClamp;
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
            texDescr.readMode = cudaReadModeNormalizedFloat;
#else
            texDescr.readMode = cudaReadModeElementType;
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR


            CHECK_CUDA_RETURN_ERROR(cudaCreateTextureObject(&previousLevel_tex, &texRes, &texDescr, nullptr));
        }

        // generate surface object for current level writing
        cudaSurfaceObject_t currentLevel_surf;
        {
            cudaResourceDesc surfRes;
            memset(&surfRes, 0, sizeof(cudaResourceDesc));
            surfRes.resType = cudaResourceTypeArray;
            surfRes.res.array.array = currentLevelArray;

            CHECK_CUDA_RETURN_ERROR(cudaCreateSurfaceObject(&currentLevel_surf, &surfRes));
        }

        // downscale previous level image into the current level image
        {
            const dim3 block(16, 16, 1);
            const dim3 grid(divUp(width, block.x), divUp(height, block.y), 1);

            createMipmappedArrayLevel_kernel<2 /* radius */><<<grid, block>>>(currentLevel_surf, previousLevel_tex, (unsigned int)(width), (unsigned int)(height));
        }

        // wait for kernel completion
        // device has completed all preceding requested tasks
        CHECK_CUDA_RETURN_ERROR(cudaDeviceSynchronize());
        CHECK_CUDA_ERROR();

        // destroy temporary CUDA objects
        CHECK_CUDA_RETURN_ERROR(cudaDestroySurfaceObject(currentLevel_surf));
        CHECK_CUDA_RETURN_ERROR(cudaDestroyTextureObject(previousLevel_tex));
    }
}

__host__ void cuda_createMipmappedArrayTexture(cudaTextureObject_t* out_mipmappedArray_texPtr,
                                               const cudaMipmappedArray_t in_mipmappedArray,
                                               const unsigned int levels)
{
    cudaResourceDesc resDescr;
    memset(&resDescr, 0, sizeof(cudaResourceDesc));
    resDescr.resType = cudaResourceTypeMipmappedArray;
    resDescr.res.mipmap.mipmap = in_mipmappedArray;

    cudaTextureDesc texDescr;
    memset(&texDescr, 0, sizeof(cudaTextureDesc));
    texDescr.normalizedCoords = 1; // should always be set to 1 for mipmapped array
    texDescr.filterMode = cudaFilterModeLinear;
    texDescr.mipmapFilterMode = cudaFilterModeLinear;
    texDescr.addressMode[0] = cudaAddressModeClamp;
    texDescr.addressMode[1] = cudaAddressModeClamp;
    texDescr.addressMode[2] = cudaAddressModeClamp;
    texDescr.maxMipmapLevelClamp = float(levels - 1);
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
    texDescr.readMode = cudaReadModeNormalizedFloat;
#else
    texDescr.readMode = cudaReadModeElementType;
#endif // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR

    CHECK_CUDA_RETURN_ERROR(cudaCreateTextureObject(out_mipmappedArray_texPtr, &resDescr, &texDescr, nullptr));
}

__host__ void cuda_createMipmappedArrayDebugFlatImage(CudaDeviceMemoryPitched<CudaRGBA, 2>& out_flatImage_dmp,
                                                      const cudaTextureObject_t in_mipmappedArray_tex,
                                                      const unsigned int levels,
                                                      const int firstLevelWidth,
                                                      const int firstLevelHeight,
                                                      cudaStream_t stream)
{
    const CudaSize<2>& out_flatImageSize = out_flatImage_dmp.getSize();

    assert(out_flatImageSize.x() == size_t(firstLevelWidth * 1.5f));
    assert(out_flatImageSize.y() == size_t(firstLevelHeight));

    const dim3 block(16, 16, 1);
    const dim3 grid(divUp(out_flatImageSize.x(), block.x), divUp(out_flatImageSize.y(), block.y), 1);

    createMipmappedArrayDebugFlatImage_kernel<<<grid, block, 0, stream>>>(
        out_flatImage_dmp.getBuffer(),
        out_flatImage_dmp.getBytesPaddedUpToDim(0),
        in_mipmappedArray_tex,
        levels,
        (unsigned int)(firstLevelWidth),
        (unsigned int)(firstLevelHeight));

    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision

