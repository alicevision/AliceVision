// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SgmParams.hpp>

#include <aliceVision/depthMap/cuda/memory.hpp>
#include <aliceVision/depthMap/cuda/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/OneTC.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

extern void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSim, 3>& volume_dmp, TSim value, cudaStream_t stream);

extern void cuda_volumeComputeSimilarity(CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp, 
                                         CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                         const CudaDeviceMemory<float>& depths_d,
                                         const DeviceCamera& rcDeviceCamera, 
                                         const DeviceCamera& tcDeviceCamera, 
                                         const SgmParams& sgmParams, 
                                         const ROI& roi,
                                         cudaStream_t stream);
 
extern void cuda_volumeAggregatePath(CudaDeviceMemoryPitched<TSim, 3>& d_volAgr,
                                     const CudaDeviceMemoryPitched<TSim, 3>& d_volSim, 
                                     const CudaSize<3>& volDim,
                                     const CudaSize<3>& axisT, 
                                     cudaTextureObject_t rc_tex, 
                                     const SgmParams& sgmParams, 
                                     bool invY,
                                     int filteringIndex);

extern void cuda_volumeRetrieveBestDepth(CudaDeviceMemoryPitched<float, 2>& bestDepth_dmp,
                                         CudaDeviceMemoryPitched<float, 2>& bestSim_dmp,
                                         const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                                         const CudaDeviceMemory<float>& depths_d, 
                                         const DeviceCamera& rcDeviceCamera,
                                         const SgmParams& sgmParams, 
                                         const ROI& roi, 
                                         cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision
