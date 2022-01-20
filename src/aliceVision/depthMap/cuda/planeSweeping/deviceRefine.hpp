// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/DeviceCamera.hpp>

namespace aliceVision {
namespace depthMap {

extern void cuda_refineDepthMap(const DeviceCamera& rcDeviceCamera, 
                                const DeviceCamera& tcDeviceCamera,
                                float* inout_depthMap_hmh,
                                float* out_simMap_hmh, 
                                const RefineParams& refineParams, 
                                int xFrom, int wPart);

} // namespace depthMap
} // namespace aliceVision
