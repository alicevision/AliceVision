// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/host/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Optimize a depth/sim map with the refineFused depth/sim map and the SGM depth/sim map.
 * @param[out] out_depthSimMapOptimized_dmp the output optimized depth/sim map
 * @param[in] in_depthSimMapSgmUpscale_dmp the input upscaled SGM depth/sim map
 * @param[in] in_depthSimMapRefinedFused_dmp the inpuyt refined and fused depth/sim map
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_optimizeDepthSimMapGradientDescent(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMapOptimized_dmp,
                                                    const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapSgmUpscale_dmp,
                                                    const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMapRefinedFused_dmp,
                                                    const DeviceCamera& rcDeviceCamera, 
                                                    const RefineParams& refineParams,
                                                    const ROI& roi,
                                                    cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision
