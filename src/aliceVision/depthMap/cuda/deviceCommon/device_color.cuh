// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

inline __device__ float Euclidean(const float3 x1, const float3 x2)
{
    return sqrtf((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

inline __device__ float Euclidean3(const float4 x1, const float4 x2)
{
    return sqrtf((x1.x - x2.x) * (x1.x - x2.x) + (x1.y - x2.y) * (x1.y - x2.y) + (x1.z - x2.z) * (x1.z - x2.z));
}

//== data conversion utils ========================================================================

// uchar4 with 0..255 components => float3 with 0..1 components
inline __device__ __host__ float3 uchar4_to_float3(const uchar4 c)
{
    return make_float3(float(c.x) / 255.0f, float(c.y) / 255.0f, float(c.z) / 255.0f);
}

//== colour conversion utils ======================================================================

// linear RGB (0..1) to XZY (0..1) using sRGB primaries
inline __host__ __device__ float3 rgb2xyz(const float3 c)
{
    return make_float3(0.4124564f * c.x + 0.3575761f * c.y + 0.1804375f * c.z,
                       0.2126729f * c.x + 0.7151522f * c.y + 0.0721750f * c.z,
                       0.0193339f * c.x + 0.1191920f * c.y + 0.9503041f * c.z);
}

// XYZ (0..1) to CIELAB (0..100) assuming D65 whitepoint - old
// XYZ (0..1) to CIELAB (0..255) assuming D65 whitepoint - new
inline __host__ __device__ float3 xyz2lab(const float3 c)
{
    // assuming whitepoint D65, XYZ=(0.95047, 1.00000, 1.08883)
    float3 r = make_float3(c.x / 0.95047f, c.y, c.z / 1.08883f);

    float3 f = make_float3((r.x > 216.0f / 24389.0f ? cbrtf(r.x) : (24389.0f / 27.0f * r.x + 16.0f) / 116.0f),
                           (r.y > 216.0f / 24389.0f ? cbrtf(r.y) : (24389.0f / 27.0f * r.y + 16.0f) / 116.0f),
                           (r.z > 216.0f / 24389.0f ? cbrtf(r.z) : (24389.0f / 27.0f * r.z + 16.0f) / 116.0f));

    // location of xzy2lab-bug (CR 2010-03-14): 116.0f * f.x - 16.0f is wrong
    float3 out = make_float3(116.0f * f.y - 16.0f, 500.0f * (f.x - f.y), 200.0f * (f.y - f.z));
    out.x = out.x * 2.55f;
    out.y = out.y * 2.55f;
    out.z = out.z * 2.55f;
    return out;
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
inline __device__ float CostYK(const int dx, const int dy, const uchar4 c1, const uchar4 c2, const float gammaC,
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
    const float deltaC = Euclidean(xyz2lab(rgb2xyz(uchar4_to_float3(c1))), xyz2lab(rgb2xyz(uchar4_to_float3(c2))));

    // spatial distance
    const float deltaP = sqrtf(float(dx * dx + dy * dy));

    return __expf(-(deltaC / gammaC + deltaP / gammaP)); // Yoon & Kweon
    // return __expf(-(deltaC * deltaC / (2 * gammaC * gammaC))) * sqrtf(__expf(-(deltaP * deltaP / (2 * gammaP *
    // gammaP)))); // DCB
}

/**
 * @see CostYK
 */
inline __device__ float CostYKfromLab(const int dx, const int dy, const float4 c1, const float4 c2, const float gammaC,
                                      const float gammaP)
{
    // Euclidean distance in Lab, assuming linear RGB
    const float deltaC = Euclidean3(c1, c2);
    // const float deltaC = fmaxf(fabs(c1.x-c2.x),fmaxf(fabs(c1.y-c2.y),fabs(c1.z-c2.z)));

    // spatial distance to the center of the patch (in pixels)
    const float deltaP = sqrtf(float(dx * dx + dy * dy));

    return __expf(-(deltaC / gammaC + deltaP / gammaP)); // Yoon & Kweon
}

inline __device__ float CostYKfromLab(const float4 c1, const float4 c2, const float gammaC)
{
    // Euclidean distance in Lab, assuming linear RGB
    const float deltaC = Euclidean3(c1, c2);
    // const float deltaC = fmaxf(fabs(c1.x-c2.x),fmaxf(fabs(c1.y-c2.y),fabs(c1.z-c2.z)));

    return __expf(-(deltaC / gammaC)); // Yoon & Kweon
}

__global__ void rgb2lab_kernel(uchar4* irgbaOlab, int irgbaOlab_p, int width, int height);

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
__global__ void generateGaussian_kernel(float* og, float delta, int radius);

} // namespace depthMap
} // namespace aliceVision
