// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceRefine.hpp"
#include "deviceRefineKernels.cuh"

#include <aliceVision/depthMap/cuda/hostUtils.hpp>

#include <utility>

namespace aliceVision {
namespace depthMap {

__host__ extern void cuda_refineDepthMap(CudaDeviceMemoryPitched<float2, 2>& inout_rcTcDepthSimMap_dmp,
                                         const DeviceCamera& rcDeviceCamera, 
                                         const DeviceCamera& tcDeviceCamera,
                                         const RefineParams& refineParams, 
                                         const ROI& roi,
                                         cudaStream_t stream)
{
    // setup block and grid
    const dim3 block(16, 16, 1);
    const dim3 grid(divUp(roi.width(), block.x), divUp(roi.height(), block.y), 1);

    const CudaSize<2>& depthSimMapSize = inout_rcTcDepthSimMap_dmp.getSize();
    CudaDeviceMemoryPitched<float2, 2> bestDepthSimMap_dmp(depthSimMapSize);

    const int halfNSteps = ((refineParams.nDepthsToRefine - 1) / 2) + 1; // default nDepthsToRefine = 31
    const int firstStep = 1 - halfNSteps;

    // find best depth/sim map in depth offset from -(halfNSteps - 1) to (halfNSteps - 1), default from -15 to 15
    for(int step = firstStep; step < halfNSteps; ++step)
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block, 0, stream>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            inout_rcTcDepthSimMap_dmp.getBuffer(), 
            inout_rcTcDepthSimMap_dmp.getPitch(),
            bestDepthSimMap_dmp.getBuffer(), 
            bestDepthSimMap_dmp.getPitch(), 
            refineParams.wsh, 
            refineParams.gammaC, 
            refineParams.gammaP,
            refineParams.useTcOrRcPixSize, 
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight(),
            step, 
            firstStep,
            roi);
    }

    /* note: filter intermediate refined depth/sim map using bilateral filter or median filter does not improve quality */

    // save the best sim and its direct neighbors sim for interpolation
    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap_dmp(depthSimMapSize);

    // set best sim map into lastThreeSimsMap_dmp 
    refine_setLastThreeSimsMap_kernel<<<grid, block, 0, stream>>>(
        lastThreeSimsMap_dmp.getBuffer(), 
        lastThreeSimsMap_dmp.getPitch(),
        bestDepthSimMap_dmp.getBuffer(), 
        bestDepthSimMap_dmp.getPitch(), 
        1, // index 0: (best depth -1), 1: (best depth), 2: (best depth + 1)
        roi); 
  
    {
        // compute similarity of (best depth - 1)
        // note: update best similarity, best depth is unchanged
        refine_compYKNCCSimMapPatch_kernel<<<grid, block, 0, stream>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            bestDepthSimMap_dmp.getBuffer(), 
            bestDepthSimMap_dmp.getPitch(), 
            refineParams.wsh, 
            refineParams.gammaC, 
            refineParams.gammaP,
            refineParams.useTcOrRcPixSize, 
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight(),
            -1.0f, // best depth - 1
            roi);

        // set similarity of (best depth - 1) into lastThreeSimsMap_dmp
        refine_setLastThreeSimsMap_kernel<<<grid, block, 0, stream>>>(
          lastThreeSimsMap_dmp.getBuffer(), 
          lastThreeSimsMap_dmp.getPitch(),
          bestDepthSimMap_dmp.getBuffer(), 
          bestDepthSimMap_dmp.getPitch(), 
          0, // index 0: (best depth -1), 1: (best depth), 2: (best depth + 1)
          roi); 
    }

    {
        // compute similarity of (best depth + 1)
        // note: update best similarity, best depth is unchanged
        refine_compYKNCCSimMapPatch_kernel<<<grid, block, 0, stream>>>(
            rcDeviceCamera.getDeviceCamId(),
            tcDeviceCamera.getDeviceCamId(),
            rcDeviceCamera.getTextureObject(), 
            tcDeviceCamera.getTextureObject(),
            bestDepthSimMap_dmp.getBuffer(), 
            bestDepthSimMap_dmp.getPitch(), 
            refineParams.wsh, 
            refineParams.gammaC, 
            refineParams.gammaP,
            refineParams.useTcOrRcPixSize, 
            rcDeviceCamera.getWidth(), 
            rcDeviceCamera.getHeight(),
            tcDeviceCamera.getWidth(), 
            tcDeviceCamera.getHeight(),
            +1.0f, // best depth + 1
            roi);

        // set sim of (best depth + 1) into lastThreeSimsMap_dmp
        refine_setLastThreeSimsMap_kernel<<<grid, block, 0, stream>>>(
          lastThreeSimsMap_dmp.getBuffer(), 
          lastThreeSimsMap_dmp.getPitch(),
          bestDepthSimMap_dmp.getBuffer(), 
          bestDepthSimMap_dmp.getPitch(), 
          2, // index 0: (best depth -1), 1: (best depth), 2: (best depth + 1)
          roi); 
    }

    // interpolation from the lastThreeSimsMap_dmp
    refine_interpolateDepthFromThreeSimsMap_kernel<<<grid, block, 0, stream>>>(
      rcDeviceCamera.getDeviceCamId(),
      tcDeviceCamera.getDeviceCamId(),
      lastThreeSimsMap_dmp.getBuffer(), 
      lastThreeSimsMap_dmp.getPitch(), 
      bestDepthSimMap_dmp.getBuffer(), 
      bestDepthSimMap_dmp.getPitch(), 
      refineParams.useTcOrRcPixSize,
      roi);

    inout_rcTcDepthSimMap_dmp.copyFrom(bestDepthSimMap_dmp);
    
    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
