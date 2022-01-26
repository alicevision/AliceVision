// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/ROI.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @brief Refine the given depth/sim map for the given RC / TC.
 * @note Try to find the best similarity by shifting the SGM depth result and interpolating.
 * @param[in,out] inout_rcTcDepthSimMap_dmp the output best depth/sim map and input SGM depth/sim map
 * @param[in] rcDeviceCamera the R device camera
 * @param[in] tcDeviceCamera the T device camera
 * @param[in] refineParams the Refine parameters
 * @param[in] roi the 2d region of interest
 * @param[in] stream the stream for gpu execution
 */
extern void cuda_refineDepthMap(CudaDeviceMemoryPitched<float2, 2>& inout_rcTcDepthSimMap_dmp,
                                const DeviceCamera& rcDeviceCamera, 
                                const DeviceCamera& tcDeviceCamera,
                                const RefineParams& refineParams, 
                                const ROI& roi,
                                cudaStream_t stream);

} // namespace depthMap
} // namespace aliceVision
