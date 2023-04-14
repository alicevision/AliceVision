// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceDepthSimilarityMap.hpp"
#include "deviceDepthSimilarityMapKernels.cuh"

#include <aliceVision/depthMap/cuda/host/divUp.hpp>

#include <utility>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_depthSimMapCopyDepthOnly(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp,
                                            const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                            float defaultSim, 
                                            cudaStream_t stream)
{
    // get output map dimensions
    const CudaSize<2>& depthSimMapDim = out_depthSimMap_dmp.getSize();

    // kernel launch parameters
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(depthSimMapDim.x(), blockSize), divUp(depthSimMapDim.y(), blockSize), 1);

    // kernel execution
    depthSimMapCopyDepthOnly_kernel<<<grid, block, 0, stream>>>(
        out_depthSimMap_dmp.getBuffer(),
        out_depthSimMap_dmp.getPitch(),
        in_depthSimMap_dmp.getBuffer(),
        in_depthSimMap_dmp.getPitch(),
        (unsigned int)(depthSimMapDim.x()),
        (unsigned int)(depthSimMapDim.y()),
        defaultSim);

    // check cuda last error
    CHECK_CUDA_ERROR();
}

__host__ void cuda_normalMapUpscale(CudaDeviceMemoryPitched<float3, 2>& out_upscaledMap_dmp,
                                    const CudaDeviceMemoryPitched<float3, 2>& in_map_dmp,
                                    const ROI& roi,
                                    cudaStream_t stream)
{
    // compute upscale ratio
    const CudaSize<2>& out_mapDim = out_upscaledMap_dmp.getSize();
    const CudaSize<2>& in_mapDim = in_map_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());

    // kernel launch parameters
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    // kernel execution
    mapUpscale_kernel<float3><<<grid, block, 0, stream>>>(
        out_upscaledMap_dmp.getBuffer(),
        out_upscaledMap_dmp.getPitch(),
        in_map_dmp.getBuffer(),
        in_map_dmp.getPitch(),
        ratio,
        roi);

    // check cuda last error
    CHECK_CUDA_ERROR();
}

__host__ void cuda_depthThicknessSmoothThickness(CudaDeviceMemoryPitched<float2, 2>& inout_depthThicknessMap_dmp,
                                               const SgmParams& sgmParams,
                                               const RefineParams& refineParams,
                                               const ROI& roi,
                                               cudaStream_t stream)
{
    const int sgmScaleStep = sgmParams.scale * sgmParams.stepXY;
    const int refineScaleStep = refineParams.scale * refineParams.stepXY;

    // min/max number of Refine samples in SGM thickness area
    const float minNbRefineSamples = 2.f;
    const float maxNbRefineSamples = max(sgmScaleStep / float(refineScaleStep), minNbRefineSamples);

    // min/max SGM thickness inflate factor
    const float minThicknessInflate = refineParams.halfNbDepths / maxNbRefineSamples;
    const float maxThicknessInflate = refineParams.halfNbDepths / minNbRefineSamples;

    // kernel launch parameters
    const int blockSize = 8;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    // kernel execution
    depthThicknessMapSmoothThickness_kernel<<<grid, block, 0, stream>>>(
        inout_depthThicknessMap_dmp.getBuffer(),
        inout_depthThicknessMap_dmp.getPitch(),
        minThicknessInflate,
        maxThicknessInflate,
        roi);

    // check cuda last error
    CHECK_CUDA_ERROR();
}

__host__ void cuda_computeSgmUpscaledDepthPixSizeMap(CudaDeviceMemoryPitched<float2, 2>& out_upscaledDepthPixSizeMap_dmp,
                                                     const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthThicknessMap_dmp,
                                                     const int rcDeviceCameraParamsId,
                                                     const DeviceMipmapImage& rcDeviceMipmapImage,
                                                     const RefineParams& refineParams,
                                                     const ROI& roi,
                                                     cudaStream_t stream)
{
    // compute upscale ratio
    const CudaSize<2>& out_mapDim = out_upscaledDepthPixSizeMap_dmp.getSize();
    const CudaSize<2>& in_mapDim = in_sgmDepthThicknessMap_dmp.getSize();
    const float ratio = float(in_mapDim.x()) / float(out_mapDim.x());

    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const CudaSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(refineParams.scale);

    // kernel launch parameters
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    // kernel execution
    if(refineParams.interpolateMiddleDepth)
    {
        computeSgmUpscaledDepthPixSizeMap_bilinear_kernel<<<grid, block, 0, stream>>>(
            out_upscaledDepthPixSizeMap_dmp.getBuffer(),
            out_upscaledDepthPixSizeMap_dmp.getPitch(),
            in_sgmDepthThicknessMap_dmp.getBuffer(),
            in_sgmDepthThicknessMap_dmp.getPitch(),
            rcDeviceCameraParamsId,
            rcDeviceMipmapImage.getTextureObject(),
            (unsigned int)(rcLevelDim.x()),
            (unsigned int)(rcLevelDim.y()),
            rcMipmapLevel,
            refineParams.stepXY,
            refineParams.halfNbDepths,
            ratio,
            roi);
    }
    else
    {
        computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel<<<grid, block, 0, stream>>>(
            out_upscaledDepthPixSizeMap_dmp.getBuffer(),
            out_upscaledDepthPixSizeMap_dmp.getPitch(),
            in_sgmDepthThicknessMap_dmp.getBuffer(),
            in_sgmDepthThicknessMap_dmp.getPitch(),
            rcDeviceCameraParamsId,
            rcDeviceMipmapImage.getTextureObject(),
            (unsigned int)(rcLevelDim.x()),
            (unsigned int)(rcLevelDim.y()),
            rcMipmapLevel,
            refineParams.stepXY,
            refineParams.halfNbDepths,
            ratio,
            roi);
    }

    // check cuda last error
    CHECK_CUDA_ERROR();
}

__host__ void cuda_depthSimMapComputeNormal(CudaDeviceMemoryPitched<float3, 2>& out_normalMap_dmp,
                                            const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                            const int rcDeviceCameraParamsId,
                                            const int stepXY,
                                            const ROI& roi,
                                            cudaStream_t stream)
{
    // kernel launch parameters
    const dim3 block(8, 8, 1);
    const dim3 grid(divUp(roi.width(), block.x), divUp(roi.height(), block.y), 1);

    // kernel execution
    depthSimMapComputeNormal_kernel<3 /* wsh */><<<grid, block, 0, stream>>>(
        out_normalMap_dmp.getBuffer(),
        out_normalMap_dmp.getPitch(),
        in_depthSimMap_dmp.getBuffer(),
        in_depthSimMap_dmp.getPitch(),
        rcDeviceCameraParamsId,
        stepXY,
        roi);

    // check cuda last error
    CHECK_CUDA_ERROR();
}

__host__ void cuda_depthSimMapOptimizeGradientDescent(CudaDeviceMemoryPitched<float2, 2>& out_optimizeDepthSimMap_dmp,
                                                      CudaDeviceMemoryPitched<float, 2>& inout_imgVariance_dmp,
                                                      CudaDeviceMemoryPitched<float, 2>& inout_tmpOptDepthMap_dmp,
                                                      const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                                      const CudaDeviceMemoryPitched<float2, 2>& in_refineDepthSimMap_dmp,
                                                      const int rcDeviceCameraParamsId,
                                                      const DeviceMipmapImage& rcDeviceMipmapImage,
                                                      const RefineParams& refineParams,
                                                      const ROI& roi,
                                                      cudaStream_t stream)
{
    // get R mipmap image level and dimensions
    const float rcMipmapLevel = rcDeviceMipmapImage.getLevel(refineParams.scale);
    const CudaSize<2> rcLevelDim = rcDeviceMipmapImage.getDimensions(refineParams.scale);

    // initialize depth/sim map optimized with SGM depth/pixSize map
    out_optimizeDepthSimMap_dmp.copyFrom(in_sgmDepthPixSizeMap_dmp, stream);

    {
        // kernel launch parameters
        const dim3 lblock(32, 2, 1);
        const dim3 lgrid(divUp(roi.width(), lblock.x), divUp(roi.height(), lblock.y), 1);

        // kernel execution
        optimize_varLofLABtoW_kernel<<<lgrid, lblock, 0, stream>>>(
            inout_imgVariance_dmp.getBuffer(), 
            inout_imgVariance_dmp.getPitch(),
            rcDeviceMipmapImage.getTextureObject(),
            (unsigned int)(rcLevelDim.x()),
            (unsigned int)(rcLevelDim.y()),
            rcMipmapLevel,
            refineParams.stepXY,
            roi);
    }

    CudaTexture<float, false, false> imgVarianceTex(inout_imgVariance_dmp); // neighbor interpolation, without normalized coordinates
    CudaTexture<float, false, false> depthTex(inout_tmpOptDepthMap_dmp);    // neighbor interpolation, without normalized coordinates

    // kernel launch parameters
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    for(int iter = 0; iter < refineParams.optimizationNbIterations; ++iter) // default nb iterations is 100
    {
        // copy depths values from out_depthSimMapOptimized_dmp to inout_tmpOptDepthMap_dmp
        optimize_getOptDeptMapFromOptDepthSimMap_kernel<<<grid, block, 0, stream>>>(
            inout_tmpOptDepthMap_dmp.getBuffer(), 
            inout_tmpOptDepthMap_dmp.getPitch(), 
            out_optimizeDepthSimMap_dmp.getBuffer(), // initialized with SGM depth/pixSize map
            out_optimizeDepthSimMap_dmp.getPitch(),
            roi);

        // adjust depth/sim by using previously computed depths
        optimize_depthSimMap_kernel<<<grid, block, 0, stream>>>(
            out_optimizeDepthSimMap_dmp.getBuffer(),
            out_optimizeDepthSimMap_dmp.getPitch(),
            in_sgmDepthPixSizeMap_dmp.getBuffer(),
            in_sgmDepthPixSizeMap_dmp.getPitch(),
            in_refineDepthSimMap_dmp.getBuffer(),
            in_refineDepthSimMap_dmp.getPitch(),
            rcDeviceCameraParamsId,
            imgVarianceTex.textureObj,
            depthTex.textureObj,
            iter, 
            roi);
    }

    // check cuda last error
    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
