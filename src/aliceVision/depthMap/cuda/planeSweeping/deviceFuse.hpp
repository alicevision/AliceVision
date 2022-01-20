// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/memory.hpp>

namespace aliceVision {
namespace depthMap {

extern void cuda_fuseDepthSimMapsGaussianKernelVoting(int width, int height,
                                                      CudaHostMemoryHeap<float2, 2>* out_depthSimMap_hmh,
                                                      std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh,
                                                      int ndepthSimMaps, 
                                                      const RefineParams& refineParams);

extern void cuda_optimizeDepthSimMapGradientDescent(const DeviceCamera& rcDeviceCamera, 
                                                    CudaHostMemoryHeap<float2, 2>& out_optimizedDepthSimMap_hmh,
                                                    const CudaHostMemoryHeap<float2, 2>& sgmDepthPixSizeMap_hmh,
                                                    const CudaHostMemoryHeap<float2, 2>& refinedDepthSimMap_hmh,
                                                    const CudaSize<2>& depthSimMapPartDim, 
                                                    const RefineParams& refineParams,
                                                    int yFrom);

} // namespace depthMap
} // namespace aliceVision
