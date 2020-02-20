// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.cuh>

namespace aliceVision {
namespace depthMap {

inline __device__ float Euclidean(const float3 x1, const float3 x2)
{
    // return sqrtf((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
    return norm3df(x1.x - x2.x, x1.y - x2.y, x1.z - x2.z);
}

inline __device__ float Euclidean3(const float4 x1, const float4 x2)
{
    // return sqrtf((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
    return norm3df(x1.x - x2.x, x1.y - x2.y, x1.z - x2.z);
}

//== colour conversion utils ======================================================================

// sRGB (0..1) to linear RGB (0..1)
inline __device__ float3 srgb2rgb(const float3 c)
{
    return make_float3(c.x <= 0.04045f ? c.x / 12.92f : __powf((c.x + 0.055f) / 1.055f, 2.4f),
                       c.y <= 0.04045f ? c.y / 12.92f : __powf((c.y + 0.055f) / 1.055f, 2.4f),
                       c.z <= 0.04045f ? c.z / 12.92f : __powf((c.z + 0.055f) / 1.055f, 2.4f));
}

// linear RGB (0..1) to XZY (0..1) using sRGB primaries
inline __device__ float3 rgb2xyz(const float3 c)
{
    return make_float3(0.4124564f * c.x + 0.3575761f * c.y + 0.1804375f * c.z,
                       0.2126729f * c.x + 0.7151522f * c.y + 0.0721750f * c.z,
                       0.0193339f * c.x + 0.1191920f * c.y + 0.9503041f * c.z);
}

inline __host__ float3 h_rgb2xyz(const float3 c)
{
    return make_float3(0.4124564f * c.x + 0.3575761f * c.y + 0.1804375f * c.z,
                       0.2126729f * c.x + 0.7151522f * c.y + 0.0721750f * c.z,
                       0.0193339f * c.x + 0.1191920f * c.y + 0.9503041f * c.z);
}

// linear RGB (0..1) to HSL (0..1)
inline __device__ float3 rgb2hsl(const float3& c)
{
    const float cmin = fminf(c.x, fminf(c.y, c.z));
    const float cmax = fmaxf(c.x, fmaxf(c.y, c.z));

    float h = 0.0f;
    if(cmin == cmax)
    { /* h = 0.0f; */
    }
    else if(cmax == c.x)
    {
        h = ((c.y - c.z) / (cmax - cmin) + 6.0f) / 6.0f;
        if(h >= 1.0f)
            h -= 1.0f;
    }
    else if(cmax == c.y)
    {
        h = ((c.z - c.x) / (cmax - cmin) + 2.0f) / 6.0f;
    }
    else /* if(cmax == c.z) */
    {
        h = ((c.x - c.y) / (cmax - cmin) + 4.0f) / 6.0f;
    }

    const float l = 0.5f * (cmin + cmax);

    float s = 0.0f;
    if(cmin == cmax)
    { /* s = 0.0f; */
    }
    else if(l <= 0.5f)
    {
        s = (cmax - cmin) / (2.0f * l);
    }
    else /* if(l > 0.5f) */
    {
        s = (cmax - cmin) / (2.0f - 2.0f * l);
    }

    return make_float3(h, s, l);
}

// XYZ (0..1) to CIELAB (0..255) assuming D65 whitepoint
inline __host__ __device__ float3 xyz2lab(const float3 c)
{
    // assuming whitepoint D65, XYZ=(0.95047, 1.00000, 1.08883)
    float3 r = make_float3(c.x / 0.95047f, c.y, c.z / 1.08883f);

    float3 f = make_float3((r.x > 216.0f / 24389.0f ? cbrtf(r.x) : (24389.0f / 27.0f * r.x + 16.0f) / 116.0f),
                           (r.y > 216.0f / 24389.0f ? cbrtf(r.y) : (24389.0f / 27.0f * r.y + 16.0f) / 116.0f),
                           (r.z > 216.0f / 24389.0f ? cbrtf(r.z) : (24389.0f / 27.0f * r.z + 16.0f) / 116.0f));

    float3 out = make_float3(116.0f * f.y - 16.0f, 500.0f * (f.x - f.y), 200.0f * (f.y - f.z));

    // convert values to fit into 0..255 (could be out-of-range)
    // TODO FACA: use float textures, the values are out-of-range for a and b.
    out.x = out.x * 2.55f;
    out.y = out.y * 2.55f;
    out.z = out.z * 2.55f;
    return out;
}

inline __device__ float rgb2gray(const uchar4 c)
{
    return 0.2989f * (float)c.x + 0.5870f * (float)c.y + 0.1140f * (float)c.z;
}

/**
 * @brief 
 * 
 * "Adaptive Support-Weight Approach for Correspondence Search", Kuk-Jin Yoon, In So Kweon
 * http://koasas.kaist.ac.kr/bitstream/10203/21048/1/000235253300014.pdf
 * 
 * @param[in] dx
 * @param[in] dy
 * @param[in] c1
 * @param[in] c2
 * @param[in] gammaC Strength of Grouping by Color similarity 5.5 / 105.5
 * @param[in] gammaP Strength of Grouping by Proximity          8 / 4
 * @return distance value
 */
inline __device__ float CostYKfromLab(const int dx, const int dy, const float4 c1, const float4 c2, const float gammaC,
                                      const float gammaP)
{
    // const float deltaC = 0; // ignore colour difference

    //// AD in RGB
    // const float deltaC =
    //    fabsf(float(c1.x) - float(c2.x)) +
    //    fabsf(float(c1.y) - float(c2.y)) +
    //    fabsf(float(c1.z) - float(c2.z));

    //// Euclidean distance in RGB
    // const float deltaC = Euclidean(
    //    uchar4_to_float3(c1),
    //    uchar4_to_float3(c2)
    //);

    //// Euclidean distance in Lab, assuming sRGB
    // const float deltaC = Euclidean(
    //    xyz2lab(rgb2xyz(srgb2rgb(uchar4_to_float3(c1)))),
    //    xyz2lab(rgb2xyz(srgb2rgb(uchar4_to_float3(c2))))
    //);

    // Euclidean distance in Lab, assuming linear RGB
    float deltaC = Euclidean3(c1, c2);
    // const float deltaC = fmaxf(fabs(c1.x-c2.x),fmaxf(fabs(c1.y-c2.y),fabs(c1.z-c2.z)));

    deltaC /= gammaC;

    // spatial distance to the center of the patch (in pixels)
    float deltaP = sqrtf(float(dx * dx + dy * dy));

    deltaP /= gammaP;

    deltaC += deltaP;

    return __expf(-deltaC); // Yoon & Kweon
    // return __expf(-(deltaC * deltaC / (2 * gammaC * gammaC))) * sqrtf(__expf(-(deltaP * deltaP / (2 * gammaP * gammaP)))); // DCB
}
/*
inline __device__ float CostYKfromLab(const float4 c1, const float4 c2, const float gammaC)
{
    // Euclidean distance in Lab, assuming linear RGB
    const float deltaC = Euclidean3(c1, c2);
    // const float deltaC = fmaxf(fabs(c1.x-c2.x),fmaxf(fabs(c1.y-c2.y),fabs(c1.z-c2.z)));

    return __expf(-(deltaC / gammaC)); // Yoon & Kweon
}
*/
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

/*
__global__ void downscale_kernel(unsigned char* tex, int tex_p, int width, int height, int scale)
{
        int x = blockIdx.x*blockDim.x + threadIdx.x;
        int y = blockIdx.y*blockDim.y + threadIdx.y;

        if ((x<width)&&(y<height)) {
                tex[y*tex_p+x] = 255.0f*tex2D(rtex, (float)x*(float)scale+0.5f, (float)y*(float)scale+0.5f);
        };
}
*/

} // namespace depthMap
} // namespace aliceVision
