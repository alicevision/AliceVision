// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceRefine.hpp"
#include "deviceRefineKernels.cuh"

#include <aliceVision/depthMap/cuda/hostUtils.hpp>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_refineDepthMap(const DeviceCamera& rcDeviceCamera, 
                                  const DeviceCamera& tcDeviceCamera,
                                  float* inout_depthMap_hmh,
                                  float* out_simMap_hmh,
                                  const RefineParams& refineParams, 
                                  int xFrom, int wPart)
{
    // setup block and grid
    const dim3 block(16, 16, 1);
    const dim3 grid(divUp(wPart, block.x), divUp(rcDeviceCamera.getHeight(), block.y), 1);

    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(wPart, rcDeviceCamera.getHeight()));
    copy(rcDepthMap_dmp, inout_depthMap_hmh, wPart, rcDeviceCamera.getHeight());

    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(wPart, rcDeviceCamera.getHeight()));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(wPart, rcDeviceCamera.getHeight()));

    const int halfNSteps = ((refineParams.nDepthsToRefine - 1) / 2) + 1; // Default ntcsteps = 31

    for(int i = 0; i < halfNSteps; ++i)
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(), 
            wPart, 
            rcDeviceCamera.getHeight(), 
            refineParams.wsh, 
            refineParams.gammaC, 
            refineParams.gammaP,
            float(i), 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight());
    }

    for(int i = 1; i < halfNSteps; ++i)
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(), 
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(), 
            wPart, 
            rcDeviceCamera.getHeight(),
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            float(-i),
            refineParams.useTcOrRcPixSize, 
            xFrom, 
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight());
    }

    /*
    // Filter intermediate refined images does not improve
    for (int i = 0; i < 5; ++i)
    {
        // Filter refined depth map
        CudaTexture<float> depthTex(bestDptMap_dmp);
        float euclideanDelta = 1.0;
        int radius = 3;
        ps_bilateralFilter<float>(
            depthTex.textureObj,
            bestDptMap_dmp,
            euclideanDelta,
            radius);
        ps_medianFilter<float>(
            depthTex.textureObj,
            bestDptMap_dmp,
            radius);
    }
    */

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap_dmp(CudaSize<2>(wPart, rcDeviceCamera.getHeight()));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(wPart, rcDeviceCamera.getHeight()));

    {
        // Set best sim map into lastThreeSimsMap_dmp.y
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(), 
            wPart, rcDeviceCamera.getHeight(), 1);
        /*
        // Compute NCC for depth-1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rc_cam.param_dev.i, 
            tc_cam.param_dev.i,
            rc_tex, tc_tex,
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            wPart, rcHeight,
            refineParams.wsh,
            refineParams.gammaC,
            refineParams.gammaP,
            0.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcDeviceCamera.getWidth(),
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(),
            tcDeviceCamera.getHeight());

        // Set sim for depth-1 into lastThreeSimsMap_dmp.y
        refine_setLastThreeSimsMap_kernel <<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            wPart, rcHeight, 1);
        */
    }

    {
        // Compute NCC for depth-1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(), 
            wPart, 
            rcDeviceCamera.getHeight(), 
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            -1.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight());

        // Set sim for depth-1 into lastThreeSimsMap_dmp.x
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(), 
            wPart, rcDeviceCamera.getHeight(), 0);
    }

    {
        // Compute NCC for depth+1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(), 
            wPart, 
            rcDeviceCamera.getHeight(), 
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            +1.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight());

        // Set sim for depth+1 into lastThreeSimsMap_dmp.z
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(), 
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            wPart, rcDeviceCamera.getHeight(), 2);
    }

    // Interpolation from the lastThreeSimsMap_dmp
    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        rcDeviceCamera.getDeviceCamId(),
        tcDeviceCamera.getDeviceCamId(),
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(), 
        wPart, 
        rcDeviceCamera.getHeight(),  
        refineParams.useTcOrRcPixSize, 
        xFrom);

    copy(out_simMap_hmh, wPart, rcDeviceCamera.getHeight(), bestSimMap_dmp);
    copy(inout_depthMap_hmh, wPart, rcDeviceCamera.getHeight(), bestDptMap_dmp);
}

} // namespace depthMap
} // namespace aliceVision
