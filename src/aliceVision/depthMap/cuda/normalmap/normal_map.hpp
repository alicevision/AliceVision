// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

void ps_computeNormalMap(
    CudaHostMemoryHeap<float3, 2>& normalMap_hmh,
    CudaHostMemoryHeap<float, 2>& depthMap_hmh,
    const CameraStruct& camera, int width, int height,
    int scale, int ncamsAllocated, int scales, int wsh, bool verbose,
    float gammaC, float gammaP);

} // namespace depthMap
} // namespace aliceVision

