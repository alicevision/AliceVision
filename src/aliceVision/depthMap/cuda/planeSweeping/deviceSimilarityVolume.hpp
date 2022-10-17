// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Initialize all the given similarity volume in device memory to the given value.
 * @param[in,out] inout_volume_dmp the similarity volume in device memory
 * @param[in] value the value to initalize with
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSim, 3>& inout_volume_dmp, TSim value, cudaStream_t stream);

/**
 * @brief Initialize all the given similarity volume in device memory to the given value.
 * @param[in,out] inout_volume_dmp the similarity volume in device memory
 * @param[in] value the value to initalize with
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeInitialize(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp, TSimRefine value, cudaStream_t stream);

/**
 * @brief Add similarity values from a given volume to another given volume.
 * @param[in,out] inout_volume_dmp the input/output similarity volume in device memory
 * @param[in] in_volume_dmp the input similarity volume in device memory
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeAdd(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp, 
                           const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volume_dmp, 
                           cudaStream_t stream);

/**
 * @brief Compute the best / second best similarity volume for the given RC / TC.
 * @param[out] out_volBestSim_dmp the best similarity volume in device memory
 * @param[out] out_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] tcDeviceCamera the T device camera
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeComputeSimilarity(CudaDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp, 
                                         CudaDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp,
                                         const CudaDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                         const DeviceCamera& rcDeviceCamera, 
                                         const DeviceCamera& tcDeviceCamera, 
                                         const SgmParams& sgmParams, 
                                         const Range& depthRange,
                                         const ROI& roi,
                                         cudaStream_t stream);

/**
 * @brief Refine the best similarity volume for the given RC / TC.
 * @param[out] inout_volSim_dmp the similarity volume in device memory
 * @param[in] in_midDepthSimMap_dmp the SGM upscaled depth/sim map (usefull to get middle depth) in device memory
 * @param[in] in_normalMap_dmpPtr (or nullptr) The related normal map
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] tcDeviceCamera the T device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeRefineSimilarity(CudaDeviceMemoryPitched<TSimRefine, 3>& inout_volSim_dmp, 
                                        const CudaDeviceMemoryPitched<float2, 2>& in_midDepthSimMap_dmp,
                                        const CudaDeviceMemoryPitched<float3, 2>* in_normalMap_dmpPtr,
                                        const DeviceCamera& rcDeviceCamera, 
                                        const DeviceCamera& tcDeviceCamera, 
                                        const RefineParams& refineParams, 
                                        const Range& depthRange,
                                        const ROI& roi,
                                        cudaStream_t stream);

/**
 * @brief Filter / Optimize the given similarity volume
 * @param[out] out_volSimFiltered_dmp the output similarity volume in device memory
 * @param[in,out] inout_volSliceAccA_dmp the volume slice first accumulation buffer in device memory
 * @param[in,out] inout_volSliceAccB_dmp the volume slice second accumulation buffer in device memory
 * @param[in,out] inout_volAxisAcc_dmp the volume axisaccumulation buffer in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] lastDepthIndex the R camera last depth index
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeOptimize(CudaDeviceMemoryPitched<TSim, 3>& out_volSimFiltered_dmp,
                                CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                CudaDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                                const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, 
                                const DeviceCamera& rcDeviceCamera,
                                const SgmParams& sgmParams, 
                                int lastDepthIndex,
                                const ROI& roi,
                                cudaStream_t stream);

/**
 * @brief Retrieve the best depth/sim in the given similarity volume.
 * @param[out] out_bestDepthSimMap_dmp the output best depth/sim map in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeRetrieveBestDepth(CudaDeviceMemoryPitched<float2, 2>& out_bestDepthSimMap_dmp,
                                         const CudaDeviceMemoryPitched<float, 2>& in_depths_dmp, 
                                         const CudaDeviceMemoryPitched<TSim, 3>& in_volSim_dmp, 
                                         const DeviceCamera& rcDeviceCamera,
                                         const SgmParams& sgmParams, 
                                         const Range& depthRange,
                                         const ROI& roi, 
                                         cudaStream_t stream);

/**
 * @brief Retrieve the best depth/sim in the given refined similarity volume.
 * @param[out] out_bestDepthSimMap_dmp the output refined and fused depth/sim map in device memory
 * @param[in] in_midDepthSimMap_dmp the SGM upscaled depth/sim map (usefull to get middle depth) in device memory
 * @param[in] in_volSim_dmp the similarity volume in device memory
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_volumeRefineBestDepth(CudaDeviceMemoryPitched<float2, 2>& out_bestDepthSimMap_dmp,
                                       const CudaDeviceMemoryPitched<float2, 2>& in_midDepthSimMap_dmp,
                                       const CudaDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp, 
                                       const DeviceCamera& rcDeviceCamera,
                                       const RefineParams& refineParams, 
                                       const ROI& roi, 
                                       cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision
