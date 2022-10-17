// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceSimilarityVolume.hpp"
#include "deviceSimilarityVolumeKernels.cuh"

#include <aliceVision/depthMap/cuda/host/hostUtils.hpp>

#include <map>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSim, 3>& inout_volume_dmp, TSim value, cudaStream_t stream)
{
    const CudaSize<3>& volDim = inout_volume_dmp.getSize();
    const dim3 block(32, 4, 1);
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

    volume_init_kernel<TSim><<<grid, block, 0, stream>>>(
        inout_volume_dmp.getBuffer(),
        inout_volume_dmp.getBytesPaddedUpToDim(1),
        inout_volume_dmp.getBytesPaddedUpToDim(0), 
        int(volDim.x()), 
        int(volDim.y()), 
        value);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp, TSimRefine value, cudaStream_t stream)
{
    const CudaSize<3>& volDim = inout_volume_dmp.getSize();
    const dim3 block(32, 4, 1);
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

    volume_init_kernel<TSimRefine><<<grid, block, 0, stream>>>(
        inout_volume_dmp.getBuffer(),
        inout_volume_dmp.getBytesPaddedUpToDim(1),
        inout_volume_dmp.getBytesPaddedUpToDim(0), 
        int(volDim.x()), 
        int(volDim.y()), 
        value);

    CHECK_CUDA_ERROR();
}

__host__ void cuda_volumeAdd(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp, 
                             const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volume_dmp, 
                             cudaStream_t stream)
{
    const CudaSize<3>& volDim = inout_volume_dmp.getSize();
    const dim3 block(32, 4, 1);
    const dim3 grid(divUp(volDim.x(), block.x), divUp(volDim.y(), block.y), volDim.z());

    volume_add_kernel<<<grid, block, 0, stream>>>(
        inout_volume_dmp.getBuffer(),
        inout_volume_dmp.getBytesPaddedUpToDim(1),
        inout_volume_dmp.getBytesPaddedUpToDim(0),
        in_volume_dmp.getBuffer(),
        in_volume_dmp.getBytesPaddedUpToDim(1),
        in_volume_dmp.getBytesPaddedUpToDim(0),
        int(volDim.x()),
        int(volDim.y()));

    CHECK_CUDA_ERROR();
}

__host__ void cuda_volumeComputeSimilarity(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp,
                                           CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp,
                                           const CudaDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                           const DeviceCamera& rcDeviceCamera, 
                                           const DeviceCamera& tcDeviceCamera,
                                           const SgmParams& sgmParams,
                                           const Range& depthRange,
                                           const ROI& roi,
                                           cudaStream_t stream)
{
    const dim3 block(32, 1, 1); // minimal default settings
    const dim3 grid(divUp(roi.width(), block.x), divUp(roi.height(), block.y), depthRange.size());

    volume_slice_kernel<<<grid, block, 0, stream>>>(
        rcDeviceCamera.getTextureObject(),
        tcDeviceCamera.getTextureObject(),
        rcDeviceCamera.getDeviceCamId(),
        tcDeviceCamera.getDeviceCamId(),
        rcDeviceCamera.getWidth(), 
        rcDeviceCamera.getHeight(), 
        tcDeviceCamera.getWidth(), 
        tcDeviceCamera.getHeight(), 
        float(sgmParams.gammaC), 
        float(sgmParams.gammaP),
        sgmParams.wsh,
        sgmParams.stepXY,
        in_depths_dmp.getBuffer(), 
        in_depths_dmp.getBytesPaddedUpToDim(0), 
        out_volBestSim_dmp.getBuffer(),
        out_volBestSim_dmp.getBytesPaddedUpToDim(1),
        out_volBestSim_dmp.getBytesPaddedUpToDim(0),
        out_volSecBestSim_dmp.getBuffer(),
        out_volSecBestSim_dmp.getBytesPaddedUpToDim(1),
        out_volSecBestSim_dmp.getBytesPaddedUpToDim(0), 
        depthRange,
        roi);

    CHECK_CUDA_ERROR();
}

extern void cuda_volumeRefineSimilarity(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volSim_dmp, 
                                        const CudaDeviceMemoryPitched<float2, 2>& in_midDepthSimMap_dmp,
                                        const CudaDeviceMemoryPitched<float3, 2>* in_normalMap_dmpPtr,
                                        const DeviceCamera& rcDeviceCamera, 
                                        const DeviceCamera& tcDeviceCamera, 
                                        const RefineParams& refineParams, 
                                        const Range& depthRange,
                                        const ROI& roi,
                                        cudaStream_t stream)
{
    const dim3 block(32, 1, 1); // minimal default settings
    const dim3 grid(divUp(roi.width(), block.x), divUp(roi.height(), block.y), depthRange.size());

    volume_refine_kernel<<<grid, block, 0, stream>>>(
        rcDeviceCamera.getTextureObject(),
        tcDeviceCamera.getTextureObject(),
        rcDeviceCamera.getDeviceCamId(),
        tcDeviceCamera.getDeviceCamId(),
        rcDeviceCamera.getWidth(), 
        rcDeviceCamera.getHeight(), 
        tcDeviceCamera.getWidth(), 
        tcDeviceCamera.getHeight(), 
        int(inout_volSim_dmp.getSize().z()), 
        refineParams.stepXY,
        refineParams.wsh, 
        float(refineParams.gammaC), 
        float(refineParams.gammaP), 
        in_midDepthSimMap_dmp.getBuffer(), 
        in_midDepthSimMap_dmp.getBytesPaddedUpToDim(0), 
        (in_normalMap_dmpPtr == nullptr) ? nullptr : in_normalMap_dmpPtr->getBuffer(),
        (in_normalMap_dmpPtr == nullptr) ? 0 : in_normalMap_dmpPtr->getBytesPaddedUpToDim(0),
        inout_volSim_dmp.getBuffer(), 
        inout_volSim_dmp.getBytesPaddedUpToDim(1),
        inout_volSim_dmp.getBytesPaddedUpToDim(0), 
        depthRange,
        roi);

    CHECK_CUDA_ERROR();
}


__host__ void cuda_volumeAggregatePath(CudaDeviceMemoryPitched<TSim, 3>& out_volAgr_dmp,
                                       CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                       CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                       CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                                       const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, 
                                       const CudaSize<3>& axisT,
                                       const DeviceCamera& rcDeviceCamera,
                                       const SgmParams& sgmParams,
                                       int lastDepthIndex,
                                       int filteringIndex, 
                                       bool invY, 
                                       const ROI& roi,
                                       cudaStream_t stream)
{
    CudaSize<3> volDim = in_volSim_dmp.getSize();
    volDim[2] = lastDepthIndex; // override volume depth, use rc depth list last index

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

    CudaDeviceMemoryPitched<TSimAcc, 2>* xzSliceForY   = &inout_volSliceAccA_dmp; // Y slice
    CudaDeviceMemoryPitched<TSimAcc, 2>* xzSliceForYm1 = &inout_volSliceAccB_dmp; // Y-1 slice
    CudaDeviceMemoryPitched<TSimAcc, 2>* bestSimInYm1  = &inout_volAxisAcc_dmp;   // best sim score along the Y axis for each Z value

    // Copy the first XZ plane (at Y=0) from 'in_volSim_dmp' into 'xzSliceForYm1'
    volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ, 0, stream>>>(
        xzSliceForYm1->getBuffer(),
        xzSliceForYm1->getPitch(),
        in_volSim_dmp.getBuffer(),
        in_volSim_dmp.getBytesPaddedUpToDim(1),
        in_volSim_dmp.getBytesPaddedUpToDim(0),
        volDim_, 
        axisT_, 
        0 /* Y = 0 */ ); 

    // Set the first Z plane from 'out_volAgr_dmp' to 255
    volume_initVolumeYSlice_kernel<TSim><<<gridVolXZ, blockVolXZ, 0, stream>>>(
        out_volAgr_dmp.getBuffer(),
        out_volAgr_dmp.getBytesPaddedUpToDim(1),
        out_volAgr_dmp.getBytesPaddedUpToDim(0),
        volDim_, 
        axisT_,
        0, 255);

    for(int iy = 1; iy < volDimY; ++iy)
    {
        const int y = invY ? volDimY - 1 - iy : iy;

        // For each column: compute the best score
        // Foreach x:
        //   bestSimInYm1[x] = min(d_xzSliceForY[1:height])
        volume_computeBestZInSlice_kernel<<<gridColZ, blockColZ, 0, stream>>>(
            xzSliceForYm1->getBuffer(), 
            xzSliceForYm1->getPitch(),
            bestSimInYm1->getBuffer(),
            volDimX, volDimZ);

        // Copy the 'z' plane from 'in_volSim_dmp' into 'xzSliceForY'
        volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ, 0, stream>>>(
            xzSliceForY->getBuffer(),
            xzSliceForY->getPitch(),
            in_volSim_dmp.getBuffer(),
            in_volSim_dmp.getBytesPaddedUpToDim(1),
            in_volSim_dmp.getBytesPaddedUpToDim(0),
            volDim_, axisT_, y);

        volume_agregateCostVolumeAtXinSlices_kernel<<<gridVolSlide, blockVolSlide, 0, stream>>>(
            rcDeviceCamera.getTextureObject(), 
            xzSliceForY->getBuffer(),   // inout: xzSliceForY
            xzSliceForY->getPitch(),     
            xzSliceForYm1->getBuffer(), // in:    xzSliceForYm1
            xzSliceForYm1->getPitch(), 
            bestSimInYm1->getBuffer(),  // in:    bestSimInYm1                        
            out_volAgr_dmp.getBuffer(), 
            out_volAgr_dmp.getBytesPaddedUpToDim(1), 
            out_volAgr_dmp.getBytesPaddedUpToDim(0), 
            volDim_, axisT_, 
            sgmParams.stepXY, 
            y, 
            sgmParams.p1, 
            sgmParams.p2Weighting,
            ySign, 
            filteringIndex,
            roi);

        std::swap(xzSliceForYm1, xzSliceForY);
    }
    
    CHECK_CUDA_ERROR();
}

__host__ void cuda_volumeOptimize(CudaDeviceMemoryPitched<TSim, 3>& out_volSimFiltered_dmp,
                                  CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                  CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                  CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                                  const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, 
                                  const DeviceCamera& rcDeviceCamera,
                                  const SgmParams& sgmParams, 
                                  int lastDepthIndex,
                                  const ROI& roi,
                                  cudaStream_t stream)
{
    // update aggregation volume
    int npaths = 0;
    const auto updateAggrVolume = [&](const CudaSize<3>& axisT, bool invX)
    {
        cuda_volumeAggregatePath(out_volSimFiltered_dmp, 
                                 inout_volSliceAccA_dmp, 
                                 inout_volSliceAccB_dmp,
                                 inout_volAxisAcc_dmp,
                                 in_volSim_dmp, 
                                 axisT, 
                                 rcDeviceCamera, 
                                 sgmParams, 
                                 lastDepthIndex,
                                 npaths,
                                 invX, 
                                 roi,
                                 stream);
        npaths++;
    };

    // filtering is done on the last axis
    const std::map<char, CudaSize<3>> mapAxes = {
        {'X', {1, 0, 2}}, // XYZ -> YXZ
        {'Y', {0, 1, 2}}, // XYZ
    };

    for(char axis : sgmParams.filteringAxes)
    {
        const CudaSize<3>& axisT = mapAxes.at(axis);
        updateAggrVolume(axisT, false); // without transpose
        updateAggrVolume(axisT, true);  // with transpose of the last axis
    }
}

__host__ void cuda_volumeRetrieveBestDepth(CudaDeviceMemoryPitched<float2, 2>& out_bestDepthSimMap_dmp,
                                           const CudaDeviceMemoryPitched<float, 2>& in_depths_dmp, 
                                           const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, 
                                           const DeviceCamera& rcDeviceCamera,
                                           const SgmParams& sgmParams, 
                                           const Range& depthRange,
                                           const ROI& roi, 
                                           cudaStream_t stream)
{
    const int scaleStep = sgmParams.scale * sgmParams.stepXY;
    const int blockSize = 8;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);
    
    volume_retrieveBestZ_kernel<<<grid, block, 0, stream>>>(
      out_bestDepthSimMap_dmp.getBuffer(),
      out_bestDepthSimMap_dmp.getBytesPaddedUpToDim(0),  
      in_depths_dmp.getBuffer(), 
      in_depths_dmp.getBytesPaddedUpToDim(0), 
      in_volSim_dmp.getBuffer(), 
      in_volSim_dmp.getBytesPaddedUpToDim(1), 
      in_volSim_dmp.getBytesPaddedUpToDim(0), 
      in_volSim_dmp.getSize().z(),
      rcDeviceCamera.getDeviceCamId(), 
      scaleStep, 
      depthRange,
      roi);

    CHECK_CUDA_ERROR();
}

extern void cuda_volumeRefineBestDepth(CudaDeviceMemoryPitched<float2, 2>& out_bestDepthSimMap_dmp,
                                       const CudaDeviceMemoryPitched<float2, 2>& in_midDepthSimMap_dmp,
                                       const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp, 
                                       const DeviceCamera& rcDeviceCamera, 
                                       const RefineParams& refineParams, 
                                       const ROI& roi, 
                                       cudaStream_t stream)
{
    const int scaleStep = refineParams.scale * refineParams.stepXY;
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));
    const float twoTimesSigmaPowerTwo = float(2.0 * refineParams.sigma * refineParams.sigma);

    const int blockSize = 8;
    const dim3 block(blockSize, blockSize, 1);
    const dim3 grid(divUp(roi.width(), blockSize), divUp(roi.height(), blockSize), 1);

    volume_refineBestZ_kernel<<<grid, block, 0, stream>>>(
      out_bestDepthSimMap_dmp.getBuffer(),
      out_bestDepthSimMap_dmp.getBytesPaddedUpToDim(0), 
      in_midDepthSimMap_dmp.getBuffer(), 
      in_midDepthSimMap_dmp.getBytesPaddedUpToDim(0),
      in_volSim_dmp.getBuffer(),
      in_volSim_dmp.getBytesPaddedUpToDim(1), 
      in_volSim_dmp.getBytesPaddedUpToDim(0), 
      int(in_volSim_dmp.getSize().z()), 
      rcDeviceCamera.getDeviceCamId(),
      scaleStep,
      samplesPerPixSize,
      twoTimesSigmaPowerTwo,
      refineParams.nSamplesHalf, 
      roi);

    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
