// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"

#include "aliceVision/depthMap/cuda/planeSweeping/device_utils.cuh"

namespace aliceVision {
namespace depthMap {

__global__ void rgb2lab_kernel(uchar4* irgbaOlab, int irgbaOlab_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        uchar4* rgb = get2DBufferAt(irgbaOlab, irgbaOlab_p, x, y);
        float3 flab = xyz2lab(rgb2xyz(uchar4_to_float3(*rgb)));

        uchar4 lab;
        lab.x = (unsigned char)(flab.x);
        lab.y = (unsigned char)(flab.y);
        lab.z = (unsigned char)(flab.z);
        lab.w = 0;

        *rgb = lab;
    }
}

/*
    Because a 2D gaussian mask is symmetry in row and column,
    here only generate a 1D mask, and use the product by row
    and column index later.

    1D gaussian distribution :
        g(x, d) -- C * exp(-x^2/d^2), C is a constant amplifier

    parameters:
    og - output gaussian array in global memory
    delta - the 2nd parameter 'd' in the above function
    radius - half of the filter size
             (total filter size = 2 * radius + 1)
*/
// use only one block
__global__ void generateGaussian_kernel(float* og, float delta, int radius)
{
    int x = threadIdx.x - radius;
    og[threadIdx.x] = __expf(-(x * x) / (2 * delta * delta));
}

} // namespace depthMap
} // namespace aliceVision
