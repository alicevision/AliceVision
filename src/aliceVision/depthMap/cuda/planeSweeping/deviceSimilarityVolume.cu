// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceSimilarityVolume.hpp"
#include "deviceSimilarityVolumeKernels.cuh"

#include <aliceVision/depthMap/cuda/hostUtils.hpp>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSim, 3>& volume_dmp, TSim value, cudaStream_t stream)
{
  const CudaSize<3>& volDim = volume_dmp.getSize();
  const dim3 block(32, 4, 1);
  const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

  volume_init_kernel<<<grid, block, 0, stream>>>(
      volume_dmp.getBuffer(),
      volume_dmp.getBytesPaddedUpToDim(1),
      volume_dmp.getBytesPaddedUpToDim(0), 
      volDim.x(), 
      volDim.y());
}

__host__ void cuda_volumeComputeSimilarity(CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                                           CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                           const CudaDeviceMemory<float>& depths_d,
                                           const DeviceCamera& rcDeviceCamera, 
                                           const DeviceCamera& tcDeviceCamera,
                                           const OneTC& cell, 
                                           const SgmParams& sgmParams, 
                                           cudaStream_t stream)
{
    const CudaSize<3>& volDim = volBestSim_dmp.getSize();

    const int startDepthIndex = cell.getDepthToStart();
    const int nbDepthsToSearch = cell.getDepthsToSearch();
    
    const dim3 block(32, 1, 1); // minimal default settings
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), nbDepthsToSearch);
    
    volume_slice_kernel<<<grid, block, 0, stream>>>(
        rcDeviceCamera.getTextureObject(),
        tcDeviceCamera.getTextureObject(),
        rcDeviceCamera.getDeviceCamId(),
        tcDeviceCamera.getDeviceCamId(),
        depths_d.getBuffer(),
        startDepthIndex,
        nbDepthsToSearch,
        rcDeviceCamera.getWidth(), 
        rcDeviceCamera.getHeight(), 
        tcDeviceCamera.getWidth(), 
        tcDeviceCamera.getHeight(), 
        sgmParams.wsh,
        float(sgmParams.gammaC), 
        float(sgmParams.gammaP),
        volBestSim_dmp.getBuffer(),
        volBestSim_dmp.getBytesPaddedUpToDim(1),
        volBestSim_dmp.getBytesPaddedUpToDim(0),
        volSecBestSim_dmp.getBuffer(),
        volSecBestSim_dmp.getBytesPaddedUpToDim(1),
        volSecBestSim_dmp.getBytesPaddedUpToDim(0),
        sgmParams.stepXY,
        volDim.x(), 
        volDim.y());
}

__host__ void cuda_volumeAggregatePath(CudaDeviceMemoryPitched<TSim, 3>& d_volAgr,
                                       const CudaDeviceMemoryPitched<TSim, 3>& d_volSim,
                                       const CudaSize<3>& volDim,
                                       const CudaSize<3>& axisT,
                                       cudaTextureObject_t rc_tex, 
                                       const SgmParams& sgmParams,
                                       bool invY, int filteringIndex)
{
    const size_t volDimX = volDim[axisT[0]];
    const size_t volDimY = volDim[axisT[1]];
    const size_t volDimZ = volDim[axisT[2]];

    const int3 volDim_ = make_int3(volDim[0], volDim[1], volDim[2]);
    const int3 axisT_ = make_int3(axisT[0], axisT[1], axisT[2]);
    const int ySign = (invY ? -1 : 1);

    // setup block and grid
    const int blockSize = 8;
    const dim3 blockVolXZ(blockSize, blockSize, 1);
    const dim3 gridVolXZ(divUp(volDimX, blockVolXZ.x), divUp(volDimZ, blockVolXZ.y), 1);

    const int blockSizeL = 64;
    const dim3 blockColZ(blockSizeL, 1, 1);
    const dim3 gridColZ(divUp(volDimX, blockColZ.x), 1, 1);

    const dim3 blockVolSlide(blockSizeL, 1, 1);
    const dim3 gridVolSlide(divUp(volDimX, blockVolSlide.x), volDimZ, 1);

    CudaDeviceMemoryPitched<TSimAcc, 2> d_sliceBufferA(CudaSize<2>(volDimX, volDimZ));
    CudaDeviceMemoryPitched<TSimAcc, 2> d_sliceBufferB(CudaSize<2>(volDimX, volDimZ));

    CudaDeviceMemoryPitched<TSimAcc, 2>* d_xzSliceForY = &d_sliceBufferA; // Y slice
    CudaDeviceMemoryPitched<TSimAcc, 2>* d_xzSliceForYm1 = &d_sliceBufferB; // Y-1 slice

    CudaDeviceMemoryPitched<TSimAcc, 2> d_bestSimInYm1(CudaSize<2>(volDimX, 1)); // best sim score along the Y axis for each Z value

    // Copy the first XZ plane (at Y=0) from 'd_volSim' into 'd_xzSliceForYm1'
    volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ>>>(
        d_xzSliceForYm1->getBuffer(),
        d_xzSliceForYm1->getPitch(),
        d_volSim.getBuffer(),
        d_volSim.getBytesPaddedUpToDim(1),
        d_volSim.getBytesPaddedUpToDim(0),
        volDim_, axisT_, 0); // Y=0

    // Set the first Z plane from 'd_volAgr' to 255
    volume_initVolumeYSlice_kernel<TSim><<<gridVolXZ, blockVolXZ>>>(
        d_volAgr.getBuffer(),
        d_volAgr.getBytesPaddedUpToDim(1),
        d_volAgr.getBytesPaddedUpToDim(0),
        volDim_, axisT_, 0, 255);

    for(int iy = 1; iy < volDimY; ++iy)
    {
        const int y = invY ? volDimY - 1 - iy : iy;

        // For each column: compute the best score
        // Foreach x:
        //   d_zBestSimInYm1[x] = min(d_xzSliceForY[1:height])
        volume_computeBestZInSlice_kernel<<<gridColZ, blockColZ>>>(
            d_xzSliceForYm1->getBuffer(), d_xzSliceForYm1->getPitch(),
            d_bestSimInYm1.getBuffer(),
            volDimX, volDimZ);

        // Copy the 'z' plane from 'd_volSimT' into 'd_xzSliceForY'
        volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ>>>(
            d_xzSliceForY->getBuffer(),
            d_xzSliceForY->getPitch(),
            d_volSim.getBuffer(),
            d_volSim.getBytesPaddedUpToDim(1),
            d_volSim.getBytesPaddedUpToDim(0),
            volDim_, axisT_, y);

        volume_agregateCostVolumeAtXinSlices_kernel<<<gridVolSlide, blockVolSlide>>>(
            rc_tex,
            d_xzSliceForY->getBuffer(), d_xzSliceForY->getPitch(),              // inout: xzSliceForY
            d_xzSliceForYm1->getBuffer(), d_xzSliceForYm1->getPitch(),          // in:    xzSliceForYm1
            d_bestSimInYm1.getBuffer(),                                         // in:    bestSimInYm1
            d_volAgr.getBuffer(), d_volAgr.getBytesPaddedUpToDim(1), d_volAgr.getBytesPaddedUpToDim(0), // out:   volAgr
            volDim_, axisT_, 
            sgmParams.stepXY, 
            y, 
            sgmParams.p1, 
            sgmParams.p2Weighting,
            ySign, filteringIndex);

        std::swap(d_xzSliceForYm1, d_xzSliceForY);
    }
    // CHECK_CUDA_ERROR();
}

__host__ void cuda_volumeRetrieveBestDepth(int rcamCacheId, 
                                           CudaDeviceMemoryPitched<float, 2>& bestDepth_dmp,
                                           CudaDeviceMemoryPitched<float, 2>& bestSim_dmp,
                                           const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                                           const CudaSize<3>& volDim,
                                           const CudaDeviceMemory<float>& depths_d, 
                                           int scaleStep, 
                                           bool interpolate)
{
    const int block_size = 8;
    const dim3 block(block_size, block_size, 1);
    const dim3 grid(divUp(volDim.x(), block_size), divUp(volDim.y(), block_size), 1);

    volume_retrieveBestZ_kernel<<<grid, block>>>(
      rcamCacheId, 
      bestDepth_dmp.getBuffer(), bestDepth_dmp.getBytesPaddedUpToDim(0), 
      bestSim_dmp.getBuffer(), bestSim_dmp.getBytesPaddedUpToDim(0), 
      volSim_dmp.getBuffer(), volSim_dmp.getBytesPaddedUpToDim(1), volSim_dmp.getBytesPaddedUpToDim(0), 
      int(volDim.x()), int(volDim.y()), int(volDim.z()), 
      depths_d.getBuffer(),
      scaleStep, 
      interpolate);
}

} // namespace depthMap
} // namespace aliceVision
