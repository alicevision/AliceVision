// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/deviceCommon/device_global.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_matrix.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_simStat.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

struct patch
{
	float3 p; //< 3d point
	float3 n; //< normal
	float3 x; //< 
	float3 y; //< 
    float d;  //< 
};

inline __device__ void computeRotCS(float3& xax, float3& yax, float3& n)
{
    xax.x = -n.y + n.z; // get any cross product
    xax.y = +n.x + n.z;
    xax.z = -n.x - n.y;
    if(fabs(xax.x) < 0.0000001f && fabs(xax.y) < 0.0000001f && fabs(xax.z) < 0.0000001f)
    {
        xax.x = -n.y - n.z; // get any cross product (complementar)
        xax.y = +n.x - n.z;
        xax.z = +n.x + n.y;
    };
    normalize(xax);
    yax = cross(n, xax);
}

/* used in device_code.cu device_code_refine.cu device_code_volume.cu */
inline __device__ void computeRotCSEpip(patch& ptch, const float3& p)
{
    ptch.p = p;

    // Vector from the reference camera to the 3d point
    float3 v1 = sg_s_rC - p;
    // Vector from the target camera to the 3d point
    float3 v2 = sg_s_tC - p;
    normalize(v1);
    normalize(v2);

    // y has to be ortogonal to the epipolar plane
    // n has to be on the epipolar plane
    // x has to be on the epipolar plane

    ptch.y = cross(v1, v2);
    normalize(ptch.y);

    ptch.n = (v1 + v2) / 2.0f; // IMPORTANT !!!
    normalize(ptch.n);
    // ptch.n = sg_s_rZVect; //IMPORTANT !!!

    ptch.x = cross(ptch.y, ptch.n);
    normalize(ptch.x);
}

/**
 * @brief Compute Normalized Cross-Correlation
 * 
 * @param[inout] ptch
 * @param[in] wsh half-width of the similarity homography matrix (width = wsh*2+1)
 * @param[in] width image width
 * @param[in] height image height
 * @param[in] _gammaC
 * @param[in] _gammaP
 * @param[in] epipShift
 * 
 * @return similarity value
 */
inline __device__ float compNCCby3DptsYK(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    patch& ptch, int wsh, int width, int height, const float _gammaC, const float _gammaP,
    const float epipShift)
{
    float3 p = ptch.p;
    float2 rp = project3DPoint(sg_s_rP, p);
    float2 tp = project3DPoint(sg_s_tP, p);

    float3 pUp = p + ptch.y * (ptch.d * 10.0f); // assuming that ptch.y is ortogonal to epipolar plane
    float2 tvUp = project3DPoint(sg_s_tP, pUp);
    tvUp = tvUp - tp;
    normalize(tvUp);
    float2 vEpipShift = tvUp * epipShift;
    tp = tp + vEpipShift;

    const float dd = wsh + 2.0f;
    if((rp.x < dd) || (rp.x > (float)(width  - 1) - dd) ||
       (rp.y < dd) || (rp.y > (float)(height - 1) - dd) ||
       (tp.x < dd) || (tp.x > (float)(width  - 1) - dd) ||
       (tp.y < dd) || (tp.y > (float)(height - 1) - dd))
    {
        return 1.0f;
    }

    // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j exactly
    // value od I(i,j) ... it is what we want
    float4 gcr = 255.0f * tex2D<float4>(r4tex, rp.x + 0.5f, rp.y + 0.5f);
    float4 gct = 255.0f * tex2D<float4>(t4tex, tp.x + 0.5f, tp.y + 0.5f);
    // gcr = 255.0f*tex2D<float4>(r4tex, rp.x, rp.y);
    // gct = 255.0f*tex2D<float4>(t4tex, tp.x, tp.y);

    float gammaC = _gammaC;
    // float gammaC = ((gcr.w>0)||(gct.w>0))?sigmoid(_gammaC,25.5f,20.0f,10.0f,fmaxf(gcr.w,gct.w)):_gammaC;
    // float gammaP = ((gcr.w>0)||(gct.w>0))?sigmoid(1.5,(float)(wsh+3),30.0f,20.0f,fmaxf(gcr.w,gct.w)):_gammaP;
    float gammaP = _gammaP;

    simStat sst = simStat();
    for(int yp = -wsh; yp <= wsh; yp++)
    {
        for(int xp = -wsh; xp <= wsh; xp++)
        {
            p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
            float2 rp1 = project3DPoint(sg_s_rP, p);
            float2 tp1 = project3DPoint(sg_s_tP, p) + vEpipShift;
            // float2 rp1 = rp + rvLeft*(float)xp + rvUp*(float)yp;
            // float2 tp1 = tp + tvLeft*(float)xp + tvUp*((float)yp+epipShift);

            // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j
            // exactly value od I(i,j) ... it is what we want
            float4 gcr1f = tex2D<float4>(r4tex, rp1.x + 0.5f, rp1.y + 0.5f);
            float4 gct1f = tex2D<float4>(t4tex, tp1.x + 0.5f, tp1.y + 0.5f);
            float4 gcr1 = 255.0f * gcr1f;
            float4 gct1 = 255.0f * gct1f;
            // gcr1 = 255.0f*tex2D(r4tex, rp1.x, rp1.y);
            // gct1 = 255.0f*tex2D(t4tex, tp1.x, tp1.y);

            // Weighting is based on:
            //  * color difference to the center pixel of the patch:
            //    ** low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
            //    ** high value (close to 1) means that the color is close the center pixel (ie. uniform color)
            //  * distance in image to the center pixel of the patch:
            //    ** low value (close to 0) means that the pixel is close to the center of the patch
            //    ** high value (close to 1) means that the pixel is far from the center of the patch
            float w = CostYKfromLab(xp, yp, gcr, gcr1, gammaC, gammaP) * CostYKfromLab(xp, yp, gct, gct1, gammaC, gammaP);
            assert(w >= 0.f);
            assert(w <= 1.f);
            sst.update(gcr1.x, gct1.x, w); // TODO: try with gcr1f and gtc1f
        }
    }
    sst.computeWSim();
    return sst.sim;
}

/* used in device_code.cu */
inline __device__ void getPixelFor3DPointRC(float2& out, float3& X)
{
    float3 p = M3x4mulV3(sg_s_rP, X);
    out = make_float2(p.x / p.z, p.y / p.z);

    if(p.z < 0.0f)
    {
        out.x = -1.0f;
        out.y = -1.0f;
    };
}

/* used in device_code.cu device_code_volume.cu */
inline __device__ void getPixelFor3DPointTC(float2& out, float3& X)
{
    float3 p = M3x4mulV3(sg_s_tP, X);
    out = make_float2(p.x / p.z, p.y / p.z);

    if(p.z < 0.0f)
    {
        out.x = -1.0f;
        out.y = -1.0f;
    }
}

/* used in device_code_refine.cu */
inline __device__ float frontoParellePlaneRCDepthFor3DPoint(const float3& p)
{
    return fabsf(orientedPointPlaneDistanceNormalizedNormal(p, sg_s_rC, sg_s_rZVect));
}

/* used in device_code_volume.cu */
inline __device__ float frontoParellePlaneTCDepthFor3DPoint(const float3& p)
{
    return fabsf(orientedPointPlaneDistanceNormalizedNormal(p, sg_s_tC, sg_s_tZVect));
}

/* used in device_code.cu device_code_refine.cu device_code_volume.cu */
inline __device__ float3 get3DPointForPixelAndFrontoParellePlaneRC(float2& pix, float fpPlaneDepth)
{
    float3 planep = sg_s_rC + sg_s_rZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(sg_s_riP, pix);
    normalize(v);
    return linePlaneIntersect(sg_s_rC, v, planep, sg_s_rZVect);
}

/* used in device_code.cu device_code_refine.cu device_code_volume.cu */
inline __device__ float3 get3DPointForPixelAndFrontoParellePlaneRC(int2& pixi, float fpPlaneDepth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
}

inline __device__ float3 get3DPointForPixelAndDepthFromRC(const float2& pix, float depth)
{
    float3 rpv = M3x3mulV2(sg_s_riP, pix);
    normalize(rpv);
    return sg_s_rC + rpv * depth;
}

inline __device__ float3 get3DPointForPixelAndDepthFromTC(const float2& pix, float depth)
{
    float3 tpv = M3x3mulV2(sg_s_tiP, pix);
    normalize(tpv);
    return sg_s_tC + tpv * depth;
}

inline __device__ float3 get3DPointForPixelAndDepthFromRC(const int2& pixi, float depth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndDepthFromRC(pix, depth);
}

inline __device__ float3 triangulateMatchRef(float2& refpix, float2& tarpix)
{
    float3 refvect = M3x3mulV2(sg_s_riP, refpix);
    normalize(refvect);
    float3 refpoint = refvect + sg_s_rC;

    float3 tarvect = M3x3mulV2(sg_s_tiP, tarpix);
    normalize(tarvect);
    float3 tarpoint = tarvect + sg_s_tC;

    float k, l;
    float3 lli1, lli2;

    lineLineIntersect(&k, &l, &lli1, &lli2, sg_s_rC, refpoint, sg_s_tC, tarpoint);

    return sg_s_rC + refvect * k;
}

inline __device__ float computeRcPixSize(const float3& p)
{
    float2 rp = project3DPoint(sg_s_rP, p);
    float2 rp1 = rp + make_float2(1.0f, 0.0f);

    float3 refvect = M3x3mulV2(sg_s_riP, rp1);
    normalize(refvect);
    return pointLineDistance3D(p, sg_s_rC, refvect);
}

inline __device__ float computePixSize(const float3& p)
{
    return computeRcPixSize(p);
}

inline __device__ float computeTcPixSize(const float3& p)
{
    float2 tp = project3DPoint(sg_s_tP, p);
    float2 tp1 = tp + make_float2(1.0f, 0.0f);

    float3 tarvect = M3x3mulV2(sg_s_tiP, tp1);
    normalize(tarvect);
    return pointLineDistance3D(p, sg_s_tC, tarvect);
}

inline __device__ float refineDepthSubPixel(const float3& depths, const float3& sims)
{
//    float floatDepth = depths.y;
    float outDepth = -1.0f;

    // subpixel refinement
    // subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation, and
    // Occlusion Handling Qingxiong pami08
    // quadratic polynomial interpolation is used to approximate the cost function between three discrete depth
    // candidates: d, dA, and dB.
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as f?x? ? ax2
    // B bx B c.
    
    float simM1 = sims.x;
    float simP1 = sims.z;
    float sim1 = sims.y;
    simM1 = (simM1 + 1.0f) / 2.0f;
    simP1 = (simP1 + 1.0f) / 2.0f;
    sim1 = (sim1 + 1.0f) / 2.0f;

    if((simM1 > sim1) && (simP1 > sim1))
    {
        float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim1)));

        float floatDepthM1 = depths.x;
        float floatDepthP1 = depths.z;

        //-1 : floatDepthM1
        // 0 : floatDepth
        //+1 : floatDepthP1
        // linear function fit
        // f(x)=a*x+b
        // floatDepthM1=-a+b
        // floatDepthP1= a+b
        // a = b - floatDepthM1
        // floatDepthP1=2*b-floatDepthM1
        float b = (floatDepthP1 + floatDepthM1) / 2.0f;
        float a = b - floatDepthM1;

        outDepth = a * dispStep + b;
    };

    return outDepth;
}

} // namespace depthMap
} // namespace aliceVision
