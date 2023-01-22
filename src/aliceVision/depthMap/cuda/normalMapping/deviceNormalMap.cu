// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceNormalMap.hpp"
#include "deviceNormalMapKernels.cuh"

#include <aliceVision/depthMap/cuda/host/divUp.hpp>

namespace aliceVision {
namespace depthMap {

__host__ void cuda_computeNormalMap(DeviceNormalMapper* mapping,
                                    int width, 
                                    int height,
                                    int wsh, 
                                    float gammaC, 
                                    float gammaP)
{
  const DeviceCameraParams* cameraParameters_d = mapping->cameraParameters_d;

  CudaDeviceMemoryPitched<float, 2>  depthMap_dmp(CudaSize<2>( width, height ));
  depthMap_dmp.copyFrom( mapping->getDepthMapHst(), width, height );

  CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(CudaSize<2>( width, height ));

  const int blockSize = 8;
  const dim3 block(blockSize, blockSize, 1);
  const dim3 grid(divUp(width, blockSize), divUp(height, blockSize), 1);

  // compute normal map
  computeNormalMap_kernel<<<grid, block>>>(
    *cameraParameters_d,
    depthMap_dmp.getBuffer(),
    depthMap_dmp.getPitch(),
    normalMap_dmp.getBuffer(),
    normalMap_dmp.getPitch(),
    width, height, wsh,
    gammaC, gammaP);

  normalMap_dmp.copyTo( mapping->getNormalMapHst(), width, height );

  CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision

