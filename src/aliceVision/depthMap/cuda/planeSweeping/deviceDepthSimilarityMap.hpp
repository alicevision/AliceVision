// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Copy depth and default from input depth/sim map to another depth/sim map.
 * @param[out] out_depthSimMap_dmp the output depth/sim map
 * @param[in] in_depthSimMap_dmp the input depth/sim map to copy
 * @param[in] defaultSim the default similarity value to copy
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_depthSimMapCopyDepthOnly(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp,
                                          const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                          float defaultSim,
                                          cudaStream_t stream);

/**
 * @brief Upscale the given normal map.
 * @param[out] out_upscaledMap_dmp the output upscaled normal map
 * @param[in] in_map_dmp the normal map to upscaled
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_normalMapUpscale(CudaDeviceMemoryPitched<float3, 2>& out_upscaledMap_dmp,
                                  const CudaDeviceMemoryPitched<float3, 2>& in_map_dmp,
                                  const ROI& roi,
                                  cudaStream_t stream);

/**
 * @brief Upscale the given depth/sim map and filter masked pixels.
 * @param[out] out_upscaledDepthSimMap_dmp the output upscaled depth/sim map
 * @param[in] in_otherDepthSimMap_dmp the depth/sim map to upscaled
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_depthSimMapUpscaleAndFilter(CudaDeviceMemoryPitched<float2, 2>& out_upscaledDepthSimMap_dmp,
                                             const CudaDeviceMemoryPitched<float2, 2>& in_otherDepthSimMap_dmp,
                                             const DeviceCamera& rcDeviceCamera,
                                             const RefineParams& refineParams,
                                             const ROI& roi,
                                             cudaStream_t stream);

/**
 * @brief Compute the pixSize map from the depth map.
 * @param[in,out] inout_depthPixSizeMap_dmp the input depth map, the output depth/pixSize map
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_depthSimMapComputePixSize(CudaDeviceMemoryPitched<float2, 2>& inout_depthPixSizeMap_dmp,
                                           const DeviceCamera& rcDeviceCamera, 
                                           const RefineParams& refineParams,
                                           const ROI& roi,
                                           cudaStream_t stream);


/**
 * @brief Compute the normal map from the depth/sim map (only depth is used).
 * @param[out] out_normalMap_dmp the output normal map
 * @param[in] in_depthSimMap_dmp the input depth/sim map (only depth is used)
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] stepXY the input depth/sim map stepXY factor
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_depthSimMapComputeNormal(CudaDeviceMemoryPitched<float3, 2>& out_normalMap_dmp,
                                          const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                                          const DeviceCamera& rcDeviceCamera,
                                          const int stepXY,
                                          const ROI& roi,
                                          cudaStream_t stream);

/**
 * @brief Optimize a depth/sim map with the refineFused depth/sim map and the SGM depth/pixSize map.
 * @param[out] out_optimizeDepthSimMap_dmp the output optimized depth/sim map
 * @param[in,out] inout_imgVariance_dmp the image variance buffer
 * @param[in,out] inout_tmpOptDepthMap_dmp the temporary optimized depth map buffer
 * @param[in] in_sgmDepthPixSizeMap_dmp the input SGM upscaled depth/pixSize map
 * @param[in] in_refineDepthSimMap_dmp the input refined and fused depth/sim map
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_depthSimMapOptimizeGradientDescent(CudaDeviceMemoryPitched<float2, 2>& out_optimizeDepthSimMap_dmp,
                                                    CudaDeviceMemoryPitched<float, 2>& inout_imgVariance_dmp,
                                                    CudaDeviceMemoryPitched<float, 2>& inout_tmpOptDepthMap_dmp,
                                                    const CudaDeviceMemoryPitched<float2, 2>& in_sgmDepthPixSizeMap_dmp,
                                                    const CudaDeviceMemoryPitched<float2, 2>& in_refineDepthSimMap_dmp,
                                                    const DeviceCamera& rcDeviceCamera, 
                                                    const RefineParams& refineParams,
                                                    const ROI& roi,
                                                    cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision
