// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "deviceColorConversion.hpp"

#include <aliceVision/depthMap/cuda/host/divUp.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
#include <aliceVision/depthMap/cuda/device/color.cuh>

namespace aliceVision {
namespace depthMap {

__global__ void rgb2lab_kernel(CudaRGBA* inout_img_d,
                               unsigned int inout_img_p,
                               unsigned int width,
                               unsigned int height)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x >= width) || (y >= height))
        return;

    // corresponding input CudaRGBA
    CudaRGBA* rgb = get2DBufferAt(inout_img_d, inout_img_p, x, y);

    // CudaRGBA (uchar4 or float4) in range (0, 255)
    // rgb2xyz needs RGB in range (0, 1)
    constexpr float d = 1 / 255.f;

    // compute output CIELAB
    // RGB(0, 255) to XYZ(0, 1) to CIELAB(0, 255)
    float3 flab = xyz2lab(rgb2xyz(make_float3(float(rgb->x) * d, float(rgb->y) * d, float(rgb->z) * d)));

    // write output CIELAB
    rgb->x = flab.x;
    rgb->y = flab.y;
    rgb->z = flab.z;
}

__host__ void cuda_rgb2lab(CudaDeviceMemoryPitched<CudaRGBA, 2>& inout_img_dmp, cudaStream_t stream)
{
    // kernel launch parameters
    const dim3 block(32, 2, 1);
    const dim3 grid(divUp(inout_img_dmp.getSize().x(), block.x), divUp(inout_img_dmp.getSize().y(), block.y), 1);

    // in-place color conversion from RGB to CIELAB
    rgb2lab_kernel<<<grid, block, 0, stream>>>(
        inout_img_dmp.getBuffer(),
        (unsigned int)inout_img_dmp.getPitch(),
        (unsigned int)inout_img_dmp.getSize().x(),
        (unsigned int)inout_img_dmp.getSize().y());

    // check cuda last error
    CHECK_CUDA_ERROR();
}

} // namespace depthMap
} // namespace aliceVision
