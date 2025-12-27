// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap_sycl/SgmParams.hpp>
#include <aliceVision/depthMap_sycl/RefineParams.hpp>
#include <aliceVision/depthMap_sycl/sycl/memory.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceMipmapImage.hpp>
#include <aliceVision/depthMap_sycl/sycl/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Initialize all the given similarity volume in device memory to the given value.
 * @param[in,out] inout_volume_dmp the similarity volume in device memory
 * @param[in] value the value to initialize with
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeInitialize(SyclDeviceMemoryPitched<TSim, 3>& inout_volume_dmp, TSim value, sycl::queue queue, sycl::event prerequisite);

/**
 * @brief Initialize all the given similarity volume in device memory to the given value.
 * @param[in,out] inout_volume_dmp the similarity volume in device memory
 * @param[in] value the value to initialize with
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeInitialize(SyclDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp, TSimRefine value, sycl::queue queue, sycl::event prerequisite);

/**
 * @brief Add similarity values from a given volume to another given volume.
 * @param[in,out] inout_volume_dmp the input/output similarity volume in device memory
 * @param[in] in_volume_dmp the input similarity volume in device memory
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeAdd(SyclDeviceMemoryPitched<TSimRefine, 3>& inout_volume_dmp,
                           const SyclDeviceMemoryPitched<TSimRefine, 3>& in_volume_dmp,
                           sycl::queue queue,
                           sycl::event prerequisite);

/**
 * @brief Update second best similarity volume uninitialized values with first best volume values.
 * @param[in] in_volBestSim_dmp the best similarity volume in device memory
 * @param[out] inout_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] queue the queue for device execution
 * @param[in] prerequisites the events to schedule after
 */
sycl::event sycl_volumeUpdateUninitializedSimilarity(const SyclDeviceMemoryPitched<TSim, 3>& in_volBestSim_dmp,
                                                     SyclDeviceMemoryPitched<TSim, 3>& inout_volSecBestSim_dmp,
                                                     sycl::queue queue,
                                                     std::vector<sycl::event> prerequisites);

/**
 * @brief Compute the best / second best similarity volume for the given RC / TC.
 * @param[out] out_volBestSim_dmp the best similarity volume in device memory
 * @param[out] out_volSecBestSim_dmp the second best similarity volume in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] rc the R camera id
 * @param[in] tc the T camera id
 * @param[in] mp the MultiViewParams to fetch camera parameters from
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] tcDeviceMipmapImage the T mipmap image in device memory container
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] queue the queue for device execution
 * @param[in] prerequisites the events to schedule after
 */
sycl::event sycl_volumeComputeSimilarity(SyclDeviceMemoryPitched<TSim, 3>& out_volBestSim_dmp,
                                         SyclDeviceMemoryPitched<TSim, 3>& out_volSecBestSim_dmp,
                                         const SyclDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                         const int rc,
                                         const int tc,
                                         const mvsUtils::MultiViewParams& mp,
                                         const DeviceMipmapImage& rcDeviceMipmapImage,
                                         const DeviceMipmapImage& tcDeviceMipmapImage,
                                         const SgmParams& sgmParams,
                                         const Range& depthRange,
                                         const ROI& roi,
                                         sycl::queue queue,
                                         std::vector<sycl::event> prerequisites);

/**
 * @brief Refine the best similarity volume for the given RC / TC.
 * @param[out] inout_volSim_dmp the similarity volume in device memory
 * @param[in] in_sgmDepthPixSizeMap_dmp the SGM upscaled depth/pixSize map (useful to get middle depth) in device memory
 * @param[in] in_sgmNormalMap_dmpPtr (or nullptr) the SGM upscaled normal map in device memory
 * @param[in] rc the R camera id
 * @param[in] tc the T camera id
 * @param[in] mp the MultiViewParams to fetch camera parameters from
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] tcDeviceMipmapImage the T mipmap image in device memory container
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeRefineSimilarity(SyclDeviceMemoryPitched<TSimRefine, 3>& inout_volSim_dmp,
                                        const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                        const SyclDeviceMemoryPitched<sycl::float3, 2>* in_sgmNormalMap_dmpPtr,
                                        const int rc,
                                        const int tc,
                                        const mvsUtils::MultiViewParams& mp,
                                        const DeviceMipmapImage& rcDeviceMipmapImage,
                                        const DeviceMipmapImage& tcDeviceMipmapImage,
                                        const RefineParams& refineParams,
                                        const Range& depthRange,
                                        const ROI& roi,
                                        sycl::queue queue,
                                        sycl::event prerequisite);

/**
 * @brief Filter / Optimize the given similarity volume
 * @param[out] out_volSimFiltered_dmp the output similarity volume in device memory
 * @param[in,out] inout_volSliceAccA_dmp the volume slice first accumulation buffer in device memory
 * @param[in,out] inout_volSliceAccB_dmp the volume slice second accumulation buffer in device memory
 * @param[in,out] inout_volAxisAcc_dmp the volume axisaccumulation buffer in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rcDeviceMipmapImage the R mipmap image in device memory container
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] lastDepthIndex the R camera last depth index
 * @param[in] roi the 2d region of interest
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeOptimize(SyclDeviceMemoryPitched<TSim, 3>& out_volSimFiltered_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccA_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volSliceAccB_dmp,
                                SyclDeviceMemoryPitched<TSimAcc, 2>& inout_volAxisAcc_dmp,
                                const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                const DeviceMipmapImage& rcDeviceMipmapImage,
                                const SgmParams& sgmParams,
                                const int lastDepthIndex,
                                const ROI& roi,
                                sycl::queue queue,
                                sycl::event prerequisite);

/**
 * @brief Retrieve the best depth/sim in the given similarity volume.
 * @param[out] out_sgmDepthThicknessMap_dmp the output depth/thickness map in device memory
 * @param[out] out_sgmDepthSimMap_dmp the output best depth/sim map in device memory
 * @param[in] in_depths_dmp the R camera depth list in device memory
 * @param[in] in_volSim_dmp the input similarity volume in device memory
 * @param[in] rc the R camera id
 * @param[in] mp the MultiViewParams to fetch camera parameters from
 * @param[in] sgmParams the Semi Global Matching parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] queue the queue for device execution
 * @param[in] prerequisite the event to schedule after
 */
sycl::event sycl_volumeRetrieveBestDepth(SyclDeviceMemoryPitched<sycl::float2, 2>& out_sgmDepthThicknessMap_dmp,
                                         SyclDeviceMemoryPitched<sycl::float2, 2>& out_sgmDepthSimMap_dmp,
                                         const SyclDeviceMemoryPitched<float, 2>& in_depths_dmp,
                                         const SyclDeviceMemoryPitched<TSim, 3>& in_volSim_dmp,
                                         const int rc,
                                         const mvsUtils::MultiViewParams& mp,
                                         const SgmParams& sgmParams,
                                         const Range& depthRange,
                                         const ROI& roi,
                                         sycl::queue queue,
                                         sycl::event prerequisite);

/**
 * @brief Retrieve the best depth/sim in the given refined similarity volume.
 * @param[out] out_refineDepthSimMap_dmp the output refined and fused depth/sim map in device memory
 * @param[in] in_sgmDepthPixSizeMap_dmp the SGM upscaled depth/pixSize map (useful to get middle depth) in device memory
 * @param[in] in_volSim_dmp the similarity volume in device memory
 * @param[in] refineParams the Refine parameters
 * @param[in] depthRange the volume depth range to compute
 * @param[in] roi the 2d region of interest
 * @param[in] queue the queue for device execution
 * @param[in] prerequisites the events to schedule after
 */
sycl::event sycl_volumeRefineBestDepth(SyclDeviceMemoryPitched<sycl::float2, 2>& out_refineDepthSimMap_dmp,
                                       const SyclDeviceMemoryPitched<sycl::float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                       const SyclDeviceMemoryPitched<TSimRefine, 3>& in_volSim_dmp,
                                       const RefineParams& refineParams,
                                       const ROI& roi,
                                       sycl::queue queue,
                                       std::vector<sycl::event> prerequisites);

}  // namespace depthMap
}  // namespace aliceVision
