// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/deviceCommon/device_global.cu>
#include <aliceVision/depthMap/cuda/deviceCommon/device_matrix.cu>
#include <aliceVision/depthMap/cuda/deviceCommon/device_patch_es_glob.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_simStat.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/device_utils.cu>

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

__device__ void computeRotCSEpip( const CameraStructBase& rc_cam,
                                  const CameraStructBase& tc_cam,
                                  Patch& ptch )
{
    // Vector from the reference camera to the 3d point
    float3 v1 = rc_cam.C - ptch.p;
    // Vector from the target camera to the 3d point
    float3 v2 = tc_cam.C - ptch.p;
    normalize(v1);
    normalize(v2);

    // y has to be ortogonal to the epipolar plane
    // n has to be on the epipolar plane
    // x has to be on the epipolar plane

    ptch.y = cross(v1, v2);
    normalize(ptch.y);

    ptch.n = (v1 + v2) / 2.0f; // IMPORTANT !!!
    normalize(ptch.n);
    // ptch.n = sg_s_r.ZVect; //IMPORTANT !!!

    ptch.x = cross(ptch.y, ptch.n);
    normalize(ptch.x);
}

__device__ int angleBetwUnitV1andUnitV2(float3& V1, float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (CUDART_PI_F / 180.0f));
}

/*
__device__ float getRefCamPixSize(Patch &ptch)
{
        float2 rp = project3DPoint(sg_s_r.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = rp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_r.iP,pix);
                float3 lpi = linePlaneIntersect(sg_s_r.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

__device__ float getTarCamPixSize(Patch &ptch)
{
        float2 tp = project3DPoint(sg_s_t.P,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = tp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_t.iP,pix);
                float3 lpi = linePlaneIntersect(sg_s_t.C, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
}

__device__ float getPatchPixSize(Patch &ptch)
{
        return fmaxf(getTarCamPixSize(ptch),getRefCamPixSize(ptch));
}
*/

__device__ void computeHomography(const CameraStructBase& rc_cam, const CameraStructBase& tc_cam, float* _H, const float3& _p, const float3& _n)
{
    // hartley zisserman second edition p.327 (13.2)
    float3 _tl = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(rc_cam.R, rc_cam.C);
    float3 _tr = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(tc_cam.R, tc_cam.C);

    float3 p = M3x3mulV3(rc_cam.R, (_p - rc_cam.C));
    float3 n = M3x3mulV3(rc_cam.R, _n);
    normalize(n);
    float d = -dot(n, p);

    float RrT[9];
    M3x3transpose(RrT, rc_cam.R);

    float tmpRr[9];
    M3x3mulM3x3(tmpRr, tc_cam.R, RrT);
    float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    float tmp[9];
    float tmp1[9];
    outerMultiply(tmp, tr, n / d);
    M3x3minusM3x3(tmp, tmpRr, tmp);
    M3x3mulM3x3(tmp1, tc_cam.K, tmp);
    M3x3mulM3x3(tmp, tmp1, rc_cam.iK);

    for(int i = 0; i < 9; i++)
    {
        _H[i] = tmp[i];
    }
}

/*
__device__ float compNCCbyH(const CameraStructBase& rc_cam, const CameraStructBase& tc_cam, const Patch& ptch, int wsh)
{
    float2 rpix = project3DPoint(sg_s_r.P, ptch.p);
    float2 tpix = project3DPoint(sg_s_t.P, ptch.p);

    float H[9];
    computeHomography(rc_cam, tc_cam, H, ptch.p, ptch.n);

    simStat sst = simStat();
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            float2 rp;
            float2 tp;
            rp.x = rpix.x + (float)xp;
            rp.y = rpix.y + (float)yp;
            tp = V2M3x3mulV2(H, rp);

            float2 g;
            g.x = 255.0f * tex2D(rtex, rp.x + 0.5f, rp.y + 0.5f);
            g.y = 255.0f * tex2D(ttex, tp.x + 0.5f, tp.y + 0.5f);
            sst.update(g);
        }
    }
    sst.computeSim();

    return sst.sim;
}
*/

/**
 * @brief Compute Normalized Cross-Correlation
 * 
 * @param[inout] ptch
 * @param[in] wsh half-width of the similarity homography matrix (width = wsh*2+1)
 * @param[in] width image width
 * @param[in] height image height
 * @param[in] _gammaC
 * @param[in] _gammaP
 * 
 * @return similarity value
 */
__device__ float compNCCby3DptsYK( cudaTextureObject_t rc_tex,
                                   cudaTextureObject_t tc_tex,
                                   const CameraStructBase& rc_cam,
                                   const CameraStructBase& tc_cam,
                                   const Patch& ptch,
                                   int wsh,
                                   int rc_width, int rc_height,
                                   int tc_width, int tc_height,
                                   const float _gammaC, const float _gammaP)
{
    float3 p = ptch.p;
    float2 rp = project3DPoint(rc_cam.P, p);
    float2 tp = project3DPoint(tc_cam.P, p);

    const float dd = wsh + 2.0f; // TODO FACA
    if((rp.x < dd) || (rp.x > (float)(rc_width  - 1) - dd) ||
       (rp.y < dd) || (rp.y > (float)(rc_height - 1) - dd) ||
       (tp.x < dd) || (tp.x > (float)(tc_width  - 1) - dd) ||
       (tp.y < dd) || (tp.y > (float)(tc_height - 1) - dd))
    {
        return 1.0f;
    }

    // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j exactly
    // value od I(i,j) ... it is what we want
    float4 gcr = tex2D<float4>(rc_tex, rp.x + 0.5f, rp.y + 0.5f);
    float4 gct = tex2D<float4>(tc_tex, tp.x + 0.5f, tp.y + 0.5f);

    // printf("gcr: R: %f, G: %f, B: %f, A: %f", gcr.x, gcr.y, gcr.z, gcr.w);
    // printf("gct: R: %f, G: %f, B: %f, A: %f", gct.x, gct.y, gct.z, gct.w);

    if (gcr.w == 0.0f || gct.w == 0.0f)
        return 1.0f; // if no alpha, invalid pixel from input mask

    float gammaC = _gammaC;
    // float gammaC = ((gcr.w>0)||(gct.w>0))?sigmoid(_gammaC,25.5f,20.0f,10.0f,fmaxf(gcr.w,gct.w)):_gammaC;
    // float gammaP = ((gcr.w>0)||(gct.w>0))?sigmoid(1.5,(float)(wsh+3),30.0f,20.0f,fmaxf(gcr.w,gct.w)):_gammaP;
    float gammaP = _gammaP;

    simStat sst;
    for(int yp = -wsh; yp <= wsh; yp++)
    {
        for(int xp = -wsh; xp <= wsh; xp++)
        {
            p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
            float2 rp1 = project3DPoint(rc_cam.P, p);
            float2 tp1 = project3DPoint(tc_cam.P, p);

            // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j
            // exactly value od I(i,j) ... it is what we want
            float4 gcr1 = tex2D<float4>(rc_tex, rp1.x + 0.5f, rp1.y + 0.5f);
            float4 gct1 = tex2D<float4>(tc_tex, tp1.x + 0.5f, tp1.y + 0.5f);

            // TODO: Does it make a difference to accurately test it for each pixel of the patch?
            // if (gcr1.w == 0.0f || gct1.w == 0.0f)
            //     continue;

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
            sst.update(gcr1.x, gct1.x, w);
        }
    }
    return sst.computeWSim();
}



/**
* @brief Compute Normalized Cross-Correlation
*
* @param[in] rc_tex
* @param[in] tc_tex3D
* @param[in] coord
* @param[in] volStepXY
* @param[in] wsh half-width of the similarity homography matrix (width = wsh*2+1)
* @param[in] gammaC
* @param[in] gammaP
*
* @return similarity value
*/
__device__ float compNCCby3DptsYK_vol(
    cudaTextureObject_t rc_tex,
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
    cudaTextureObject_t tc_tex3D,
#else
    const float4* volTcamColors, const int volTcamColors_s, const int volTcamColors_p,
#endif
    const int coordX, const int coordY, const int coordZ,
    const int volDimX, const int volDimY,
    const int scale, const int volStepXY,
    const int wsh,
    const float gammaC, const float gammaP)
{
    // const dim3 coord = { coordX, coordY, coordZ };

    const float4 gcr = tex2D<float4>(rc_tex, coordX * volStepXY + 0.5f, coordY * volStepXY + 0.5f);

#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
    const float4 gct = tex3D<float4>(tc_tex3D, coordX + 0.5f, coordY + 0.5f, coordZ + 0.5f);
#else
    const float4 gct = *get3DBufferAt<float4>(volTcamColors, volTcamColors_s, volTcamColors_p, coordX, coordY, coordZ);
#endif

    // const int verbose = (int(coordX) % 200 == 0 && int(coordY) % 200 == 0 && int(coordZ) % 50 == 10);

    if( gcr.w == 0.0f || gct.w == 0.0f )
        return 1.0f; // if no alpha, invalid pixel from input mask

    simStat sst;
    for (int yp = -wsh; yp <= wsh; yp++)
    {
        float2 coord_i;
        coord_i.y = coordY + float(yp);
        if (int(coord_i.y) < 0 || int(coord_i.y) >= volDimY)
            continue;
        for (int xp = -wsh; xp <= wsh; xp++)
        {
            coord_i.x = coordX + float(xp);
            if (int(coord_i.x) < 0 || int(coord_i.x) >= volDimX)
                continue;

            const float4 gcr_i = tex2D<float4>(rc_tex, coord_i.x * volStepXY + 0.5f, coord_i.y * volStepXY + 0.5f);
            if (gcr_i.w == 0.0f)
                continue;

#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
            const float4 gct_i = tex3D<float4>(tc_tex3D, coord_i.x + 0.5f, coord_i.y + 0.5f, coordZ + 0.5f);
#else
            const float4 gct_i = *get3DBufferAt<float4>(volTcamColors, volTcamColors_s, volTcamColors_p, int(coord_i.x), int(coord_i.y), int(coordZ));
#endif
            if (gct_i.w == 0.0f)
                continue;

            // Weighting is based on:
            //  * color difference to the center pixel of the patch:
            //    ** low value (close to 0) means that the color is different from the center pixel (ie. strongly supported surface)
            //    ** high value (close to 1) means that the color is close the center pixel (ie. uniform color)
            //  * distance in image to the center pixel of the patch:
            //    ** low value (close to 0) means that the pixel is close to the center of the patch
            //    ** high value (close to 1) means that the pixel is far from the center of the patch
            const float w = CostYKfromLab(xp, yp, gcr, gcr_i, gammaC, gammaP) * CostYKfromLab(xp, yp, gct, gct_i, gammaC, gammaP);
            assert(w >= 0.f);
            assert(w <= 1.f);
            sst.update(gcr_i.x, gct_i.x, w);
        }
    }

    /*
    if (verbose)
    {
        printf("compNCCby3DptsYK_vol: coordX: %i, coordY: %i, coordZ: %i, scale: %i, volStepXY: %i\n", coordX, coordY, coordZ, scale, volStepXY);
        printf("compNCCby3DptsYK_vol: gcr: %f, %f, %f, %f\n", gcr.x, gcr.y, gcr.z, gcr.w);
        printf("compNCCby3DptsYK_vol: gct: %f, %f, %f, %f\n", gct.x, gct.y, gct.z, gct.w);
        printf("compNCCby3DptsYK_vol: sst.sim: %f\n", sst.sim);
    }*/
    return sst.computeWSim();
}

__device__ void getPixelFor3DPoint(const CameraStructBase& cam, float2& out, float3& X)
{
    float3 p = M3x4mulV3(cam.P, X);
    out = make_float2(p.x / p.z, p.y / p.z);

    if(p.z < 0.0f)
    {
        out.x = -1.0f;
        out.y = -1.0f;
    }
}

__device__ float3 get3DPointForPixelAndFrontoParellePlaneRC( const CameraStructBase& rc_cam,
                                                             const float2& pix,
                                                             float fpPlaneDepth)
{
    float3 planep = rc_cam.C + rc_cam.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(rc_cam.iP, pix);
    normalize(v);
    return linePlaneIntersect(rc_cam.C, v, planep, rc_cam.ZVect);
}

__device__ float3 get3DPointForPixelAndFrontoParellePlaneRC( const CameraStructBase& rc_cam,
                                                             const int2& pixi,
                                                             float fpPlaneDepth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndFrontoParellePlaneRC(rc_cam, pix, fpPlaneDepth);
}

__device__ float3 get3DPointForPixelAndDepthFromRC(const CameraStructBase& rc_cam, const float2& pix, float depth)
{
    float3 rpv = M3x3mulV2(rc_cam.iP, pix);
    normalize(rpv);
    return rc_cam.C + rpv * depth;
}

__device__ float3 get3DPointForPixelAndDepthFromRC(const CameraStructBase& rc_cam, const int2& pixi, float depth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndDepthFromRC(rc_cam, pix, depth);
}

__device__ float3 triangulateMatchRef(const CameraStructBase& rc_cam, const CameraStructBase& tc_cam, float2& refpix, float2& tarpix)
{
    float3 refvect = M3x3mulV2(rc_cam.iP, refpix);
    normalize(refvect);
    float3 refpoint = refvect + rc_cam.C;

    float3 tarvect = M3x3mulV2(tc_cam.iP, tarpix);
    normalize(tarvect);
    float3 tarpoint = tarvect + tc_cam.C;

    float k, l;
    float3 lli1, lli2;

    lineLineIntersect(&k, &l, &lli1, &lli2, rc_cam.C, refpoint, tc_cam.C, tarpoint);

    return rc_cam.C + refvect * k;
}

__device__ float computePixSize(const CameraStructBase& cam, const float3& p)
{
    float2 rp = project3DPoint(cam.P, p);
    float2 rp1 = rp + make_float2(1.0f, 0.0f);

    float3 refvect = M3x3mulV2(cam.iP, rp1);
    normalize(refvect);
    return pointLineDistance3D(p, cam.C, refvect);
}

__device__ float refineDepthSubPixel(const float3& depths, const float3& sims)
{
    // subpixel refinement
    // subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation, and
    // Occlusion Handling Qingxiong pami08
    // quadratic polynomial interpolation is used to approximate the cost function between three discrete depth
    // candidates: d, dA, and dB.
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as f?x? ? ax2
    // B bx B c.
    
    float simM1 = sims.x;
    float sim = sims.y;
    float simP1 = sims.z;
    simM1 = (simM1 + 1.0f) / 2.0f;
    sim = (sim + 1.0f) / 2.0f;
    simP1 = (simP1 + 1.0f) / 2.0f;

    // sim is supposed to be the best one (so the smallest one)
    if((simM1 < sim) || (simP1 < sim))
        return depths.y; // return the input

    float dispStep = -((simP1 - simM1) / (2.0f * (simP1 + simM1 - 2.0f * sim)));

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

    float interpDepth = a * dispStep + b;

    // Ensure that the interpolated value is isfinite  (i.e. neither infinite nor NaN)
    if(!isfinite(interpDepth) || interpDepth <= 0.0f)
        return depths.y; // return the input

    return interpDepth;
}

} // namespace depthMap
} // namespace aliceVision
