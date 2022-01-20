// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "deviceColorConversion.hpp"

#include <aliceVision/depthMap/cuda/hostUtils.hpp>
#include <aliceVision/depthMap/cuda/device/utils.cuh>
#include <aliceVision/depthMap/cuda/device/color.cuh>

namespace aliceVision {
namespace depthMap {

__global__ void rgb2lab_kernel(CudaRGBA* irgbaOlab, int irgbaOlab_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x >= width) || (y >= height))
        return;

    CudaRGBA* rgb = get2DBufferAt(irgbaOlab, irgbaOlab_p, x, y);
    float3 flab = xyz2lab(rgb2xyz(make_float3(rgb->x / 255.f, rgb->y / 255.f, rgb->z / 255.f)));

    rgb->x = flab.x;
    rgb->y = flab.y;
    rgb->z = flab.z;
}

__host__ void cuda_rgb2lab(CudaDeviceMemoryPitched<CudaRGBA, 2>& frame_dmp, int width, int height, cudaStream_t stream)
{
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(width, block.x), divUp(height, block.y), 1);

    // in-place color conversion into CIELAB
    rgb2lab_kernel<<<grid, block, 0, stream>>>(frame_dmp.getBuffer(), frame_dmp.getPitch(), width, height);
    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
