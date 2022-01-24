// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SgmParams.hpp>

#include <aliceVision/depthMap/cuda/memory.hpp>
#include <aliceVision/depthMap/cuda/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/ROI.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

  
/**
 * @brief Initialize all the given similarity volume in device memory to the given value.
 * @param[out] volume_dmp the similarity volume in device memory
 * @param[in] value the value to initalize with
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSim, 3>& volume_dmp, TSim value, cudaStream_t stream);


/**
 * @brief Compute the best / second best similarity volume for the given RC / TC.
 * @param[out] volBestSim_dmp the best similarity volume in device memory
 * @param[out] volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] depths_d the R camera depth list in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] tcDeviceCamera the T device camera
 * @param[in] SgmParams the Semi Global Matching parameters
 * @param[in] roi the 3d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeComputeSimilarity(CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp, 
                                         CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                         const CudaDeviceMemory<float>& depths_d,
                                         const DeviceCamera& rcDeviceCamera, 
                                         const DeviceCamera& tcDeviceCamera, 
                                         const SgmParams& sgmParams, 
                                         const ROI& roi,
                                         cudaStream_t stream);

/**
 * @brief Filter / Optimize the given similarity volume
 * @param[out] volSimFiltered_dmp the output similarity volume in device memory
 * @param[in] volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] SgmParams the Semi Global Matching parameters
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeOptimize(CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp,
                                const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                                const DeviceCamera& rcDeviceCamera,
                                const SgmParams& sgmParams,
                                cudaStream_t stream);

/**
 * @brief Retrieve the best depth/sim in the given similarity volume.
 * @param[out] bestDepth_dmp the output best depth map in device memory
 * @param[out] bestSim_dmp the output best sim map in device memory
 * @param[in] volSim_dmp the input similarity volume in device memory
 * @param[in] depths_d the R camera depth list in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] SgmParams the Semi Global Matching parameters
 * @param[in] roi the 3d region of interest
 * @param[in] stream the stream for gpu execution
 */
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
