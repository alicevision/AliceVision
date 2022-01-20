// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceFuse.hpp"
#include "deviceFuseKernels.cuh"

#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_fuseDepthSimMapsGaussianKernelVoting(int width, int height,
                                                        CudaHostMemoryHeap<float2, 2>* out_depthSimMap_hmh,
                                                        std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh,
                                                        int ndepthSimMaps, const RefineParams& refineParams)
{
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));
    const float twoTimesSigmaPowerTwo = 2.0f * refineParams.sigma * refineParams.sigma;

    // setup block and grid
    const int block_size = 16;
    const dim3 block(block_size, block_size, 1);
    const dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    CudaDeviceMemoryPitched<float2, 2> bestDepthSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> bestGsvSampleMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> gsvSampleMap_dmp(CudaSize<2>(width, height));
    std::vector<CudaDeviceMemoryPitched<float2, 2>*> depthSimMaps_dmp(ndepthSimMaps);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        depthSimMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*depthSimMaps_dmp[i]), (*depthSimMaps_hmh[i]));
    }

    for(int s = -refineParams.nSamplesHalf; s <= refineParams.nSamplesHalf; s++) // (-150, 150)
    {
        for(int c = 1; c < ndepthSimMaps; c++) // number of T cameras
        {
            fuse_computeGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
                gsvSampleMap_dmp.getBuffer(), gsvSampleMap_dmp.getPitch(), depthSimMaps_dmp[c]->getBuffer(),
                depthSimMaps_dmp[c]->getPitch(), depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->getPitch(),
                width, height, (float)s, c - 1, samplesPerPixSize, twoTimesSigmaPowerTwo);
        }
        fuse_updateBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
            bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.getPitch(), gsvSampleMap_dmp.getBuffer(),
            gsvSampleMap_dmp.getPitch(), width, height, (float)s, s + refineParams.nSamplesHalf);
    }

    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
        bestDepthSimMap_dmp.getBuffer(), bestDepthSimMap_dmp.getPitch(), bestGsvSampleMap_dmp.getBuffer(),
        bestGsvSampleMap_dmp.getPitch(), depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->getPitch(), width,
        height, samplesPerPixSize);

    copy((*out_depthSimMap_hmh), bestDepthSimMap_dmp);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        delete depthSimMaps_dmp[i];
    }
}

__host__ void cuda_optimizeDepthSimMapGradientDescent(const DeviceCamera& rcDeviceCamera,
                                                      CudaHostMemoryHeap<float2, 2>& out_optimizedDepthSimMap_hmh,
                                                      const CudaHostMemoryHeap<float2, 2>& sgmDepthPixSizeMap_hmh,
                                                      const CudaHostMemoryHeap<float2, 2>& refinedDepthSimMap_hmh,
                                                      const CudaSize<2>& depthSimMapPartDim, 
                                                      const RefineParams& refineParams,
                                                      int yFrom)
{
    const int partWidth = depthSimMapPartDim.x();
    const int partHeight = depthSimMapPartDim.y();
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));

    // setup block and grid
    const int block_size = 16;
    const dim3 block(block_size, block_size, 1);
    const dim3 grid(divUp(partWidth, block_size), divUp(partHeight, block_size), 1);

    const CudaDeviceMemoryPitched<float2, 2> sgmDepthPixSizeMap_dmp(sgmDepthPixSizeMap_hmh);
    const CudaDeviceMemoryPitched<float2, 2> refinedDepthSimMap_dmp(refinedDepthSimMap_hmh);

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(depthSimMapPartDim);
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(depthSimMapPartDim);
    copy(optDepthSimMap_dmp, sgmDepthPixSizeMap_dmp);

    CudaDeviceMemoryPitched<float, 2> imgVariance_dmp(depthSimMapPartDim);
    {
        const dim3 lblock(32, 2, 1);
        const dim3 lgrid(divUp(partWidth, lblock.x), divUp(partHeight, lblock.y), 1);

        compute_varLofLABtoW_kernel<<<lgrid, lblock>>>(rcDeviceCamera.getTextureObject(), imgVariance_dmp.getBuffer(),
                                                       imgVariance_dmp.getPitch(), partWidth, partHeight, yFrom);
    }
    CudaTexture<float> imgVarianceTex(imgVariance_dmp);

    for(int iter = 0; iter < refineParams.nIters; ++iter) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOptDepthSimMap_kernel<<<grid, block>>>(
            optDepthMap_dmp.getBuffer(), optDepthMap_dmp.getPitch(), optDepthSimMap_dmp.getBuffer(),
            optDepthSimMap_dmp.getPitch(), partWidth, partHeight);

        CudaTexture<float> depthTex(optDepthMap_dmp);

        // Adjust depth/sim by using previously computed depths
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(
            rcDeviceCamera.getTextureObject(), rcDeviceCamera.getDeviceCamId(), imgVarianceTex.textureObj,
            depthTex.textureObj, optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
            sgmDepthPixSizeMap_dmp.getBuffer(), sgmDepthPixSizeMap_dmp.getPitch(), refinedDepthSimMap_dmp.getBuffer(),
            refinedDepthSimMap_dmp.getPitch(), partWidth, partHeight, iter, samplesPerPixSize, yFrom);
    }

    copy(out_optimizedDepthSimMap_hmh, optDepthSimMap_dmp);
}

} // namespace depthMap
} // namespace aliceVision
