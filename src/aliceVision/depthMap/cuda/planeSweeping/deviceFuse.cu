// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceFuse.hpp"
#include "deviceFuseKernels.cuh"

#include <aliceVision/depthMap/cuda/hostUtils.hpp>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_fuseDepthSimMapsGaussianKernelVoting(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapRefinedFused_dmp,
                                                        const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapPartSgmUpscale_dmp,
                                                        const std::vector<CudaDeviceMemoryPitched<float2, 2>>& in_depthSimMapPartPerRcTc_dmp,
                                                        const RefineParams& refineParams,
                                                        const ROI& roi, 
                                                        cudaStream_t stream)
{
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));
    const float twoTimesSigmaPowerTwo = 2.0f * refineParams.sigma * refineParams.sigma;

    // setup block and grid
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    const CudaSize<2> roiSize(roi.width(), roi.height());

    assert(roiSize == in_depthSimMapPartPerCam_dmp.front().getSize());

    CudaDeviceMemoryPitched<float2, 2> bestGsvSampleMapPart_dmp(roiSize);
    CudaDeviceMemoryPitched<float, 2> gsvSampleMapPart_dmp(roiSize);

    // sliding gaussian window
    for(int sample = -refineParams.nSamplesHalf; sample <= refineParams.nSamplesHalf; ++sample) // default sample range from -150 to 150
    {
        // compute the gaussian window sample 
        for(int tci = 0; tci < in_depthSimMapPartPerRcTc_dmp.size(); ++tci) // number of T cameras
        {
            // sum gaussian window sample score of each RcTc depth/sim map
            fuse_computeGaussianKernelVotingSampleMap_kernel<<<grid, block, 0, stream>>>(
                gsvSampleMapPart_dmp.getBuffer(), 
                gsvSampleMapPart_dmp.getPitch(), 
                in_depthSimMapPartPerRcTc_dmp[tci].getBuffer(), // tc depth/sim map 
                in_depthSimMapPartPerRcTc_dmp[tci].getPitch(),
                in_depthSimMapPartSgmUpscale_dmp.getBuffer(), // sgm depth/pixSize map for middle depth
                in_depthSimMapPartSgmUpscale_dmp.getPitch(),
                tci, // first tc cam id, (re)-initialization
                float(sample),
                samplesPerPixSize, 
                twoTimesSigmaPowerTwo,
                roi);
        }

        // save the sample if it's the best
        fuse_updateBestGaussianKernelVotingSampleMap_kernel<<<grid, block, 0, stream>>>(
            bestGsvSampleMapPart_dmp.getBuffer(), 
            bestGsvSampleMapPart_dmp.getPitch(), 
            gsvSampleMapPart_dmp.getBuffer(),
            gsvSampleMapPart_dmp.getPitch(), 
            sample + refineParams.nSamplesHalf, // first sample, first initialization 
            float(sample),                 
            roi);
    }

    // write the output depth/sim for the best sample
    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block, 0, stream>>>(
        out_depthSimMapRefinedFused_dmp.getBuffer(), 
        out_depthSimMapRefinedFused_dmp.getPitch(), 
        bestGsvSampleMapPart_dmp.getBuffer(),
        bestGsvSampleMapPart_dmp.getPitch(), 
        in_depthSimMapPartSgmUpscale_dmp.getBuffer(), // sgm depth/pixSize map for middle depth
        in_depthSimMapPartSgmUpscale_dmp.getPitch(), 
        samplesPerPixSize,
        roi);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_optimizeDepthSimMapGradientDescent(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp,
                                                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,
                                                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapRefinedFused_dmp,
                                                      const DeviceCamera& rcDeviceCamera, 
                                                      const RefineParams& refineParams,
                                                      const ROI& roi,
                                                      cudaStream_t stream)
{
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));

    // initialize depth/sim map optimized with SGM depth/sim map
    copy(out_depthSimMapOptimized_dmp, in_depthSimMapSgmUpscale_dmp);
    
    const CudaSize<2> roiSize(roi.width(), roi.height());
    CudaDeviceMemoryPitched<float, 2> optDepthMapPart_dmp(roiSize);
    CudaDeviceMemoryPitched<float, 2> imgVariancePart_dmp(roiSize);

    {
        // setup block and grid
        const dim3 lblock(32, 2, 1);
        const dim3 lgrid(divUp(roi.width(), lblock.x), divUp(roi.height(), lblock.y), 1);

        compute_varLofLABtoW_kernel<<<lgrid, lblock, 0, stream>>>(
            rcDeviceCamera.getTextureObject(), 
            imgVariancePart_dmp.getBuffer(), 
            imgVariancePart_dmp.getPitch(),
            roi);
    }

    CudaTexture<float> imgVarianceTex(imgVariancePart_dmp);

    // setup block and grid
    const int blockSize = 16;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    for(int iter = 0; iter < refineParams.nIters; ++iter) // default nb iterations is 100
    {
        // copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOptDepthSimMap_kernel<<<grid, block, 0, stream>>>(
            optDepthMapPart_dmp.getBuffer(), 
            optDepthMapPart_dmp.getPitch(), 
            out_depthSimMapOptimized_dmp.getBuffer(), // initialized with SGM depth/sim map
            out_depthSimMapOptimized_dmp.getPitch(),
            roi);

        CudaTexture<float> depthTex(optDepthMapPart_dmp);

        // adjust depth/sim by using previously computed depths
        fuse_optimizeDepthSimMap_kernel<<<grid, block, 0, stream>>>(
            rcDeviceCamera.getTextureObject(), 
            rcDeviceCamera.getDeviceCamId(), 
            imgVarianceTex.textureObj,
            depthTex.textureObj, 
            out_depthSimMapOptimized_dmp.getBuffer(), 
            out_depthSimMapOptimized_dmp.getPitch(),
            in_depthSimMapSgmUpscale_dmp.getBuffer(), 
            in_depthSimMapSgmUpscale_dmp.getPitch(),
            in_depthSimMapRefinedFused_dmp.getBuffer(), 
            in_depthSimMapRefinedFused_dmp.getPitch(),
            iter, 
            samplesPerPixSize, 
            roi);
    }

    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
