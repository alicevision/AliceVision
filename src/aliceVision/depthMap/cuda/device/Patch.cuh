// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/device/DeviceCameraParams.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
#include <aliceVision/depthMap/cuda/device/color.cuh>
#include <aliceVision/depthMap/cuda/device/matrix.cuh>
#include <aliceVision/depthMap/cuda/device/SimStat.cuh>

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

struct Patch
{
    float3 p; //< 3d point
    float3 n; //< normal
    float3 x; //< x axis
    float3 y; //< y axis
    float d;  //< pixel size
};

__device__ static void rotPointAroundVect(float3& out, float3& X, float3& vect, int angle)
{
    double ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    double sizeX = sqrt(dot(X, X));
    x = X.x / sizeX;
    y = X.y / sizeX;
    z = X.z / sizeX;
    u = vect.x;
    v = vect.y;
    w = vect.z;

    /*Rotate the point (x,y,z) around the vector (u,v,w)*/
    ux = u * x;
    uy = u * y;
    uz = u * z;
    vx = v * x;
    vy = v * y;
    vz = v * z;
    wx = w * x;
    wy = w * y;
    wz = w * z;
    sa = sin((double)angle * (M_PI / 180.0f));
    ca = cos((double)angle * (M_PI / 180.0f));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    u = sqrt(x * x + y * y + z * z);
    x /= u;
    y /= u;
    z /= u;

    out.x = x * sizeX;
    out.y = y * sizeX;
    out.z = z * sizeX;
}

__device__ static void rotatePatch(Patch& ptch, int rx, int ry)
{
    float3 n, y, x;

    // rotate patch around x axis by angle rx
    rotPointAroundVect(n, ptch.n, ptch.x, rx);
    rotPointAroundVect(y, ptch.y, ptch.x, rx);
    ptch.n = n;
    ptch.y = y;

    // rotate new patch around y axis by angle ry
    rotPointAroundVect(n, ptch.n, ptch.y, ry);
    rotPointAroundVect(x, ptch.x, ptch.y, ry);
    ptch.n = n;
    ptch.x = x;
}

__device__ static void movePatch(Patch& ptch, int pt)
{
    // float3 v = ptch.p-rC;
    // normalize(v);
    float3 v = ptch.n;

    float d = ptch.d * (float)pt;
    float3 p = ptch.p + v * d;
    ptch.p = p;
}

__device__ static void computeRotCS(float3& xax, float3& yax, float3& n)
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

__device__ static void computeRotCSEpip(int rcDeviceCamId, int tcDeviceCamId, Patch& ptch)
{
    // Vector from the reference camera to the 3d point
    float3 v1 = constantCameraParametersArray_d[rcDeviceCamId].C - ptch.p;
    // Vector from the target camera to the 3d point
    float3 v2 = constantCameraParametersArray_d[tcDeviceCamId].C - ptch.p;
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

__device__ static inline int angleBetwUnitV1andUnitV2(float3& V1, float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (CUDART_PI_F / 180.0f));
}

/*
__device__ static float getRefCamPixSize(Patch &ptch)
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

__device__ static float getTarCamPixSize(Patch &ptch)
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

__device__ static float getPatchPixSize(Patch &ptch)
{
        return fmaxf(getTarCamPixSize(ptch),getRefCamPixSize(ptch));
}
*/

__device__ static void computeHomography(int rcDeviceCamId, int tcDeviceCamId, float* _H, const float3& _p,
                                        const float3& _n)
{
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCamId];
    const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[tcDeviceCamId];

    // hartley zisserman second edition p.327 (13.2)
    float3 _tl = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(rcDeviceCamParams.R, rcDeviceCamParams.C);
    float3 _tr = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(tcDeviceCamParams.R, tcDeviceCamParams.C);

    float3 p = M3x3mulV3(rcDeviceCamParams.R, (_p - rcDeviceCamParams.C));
    float3 n = M3x3mulV3(rcDeviceCamParams.R, _n);
    normalize(n);
    float d = -dot(n, p);

    float RrT[9];
    M3x3transpose(RrT, rcDeviceCamParams.R);

    float tmpRr[9];
    M3x3mulM3x3(tmpRr, tcDeviceCamParams.R, RrT);
    float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    float tmp[9];
    float tmp1[9];
    outerMultiply(tmp, tr, n / d);
    M3x3minusM3x3(tmp, tmpRr, tmp);
    M3x3mulM3x3(tmp1, tcDeviceCamParams.K, tmp);
    M3x3mulM3x3(tmp, tmp1, rcDeviceCamParams.iK);

    for(int i = 0; i < 9; i++)
    {
        _H[i] = tmp[i];
    }
}

/*
__device__ static float compNCCbyH(const DeviceCameraParams& rc_cam, const DeviceCameraParams& tc_cam, const Patch& ptch, int
wsh)
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
 *         or invalid similarity (CUDART_INF_F) if uninitialized or masked
 */
__device__ static float compNCCby3DptsYK(cudaTextureObject_t rcTex, 
                                         cudaTextureObject_t tcTex, 
                                         int rcDeviceCamId,
                                         int tcDeviceCamId, 
                                         const Patch& ptch, 
                                         int rcWidth, int rcHeight,
                                         int tcWidth, int tcHeight, 
                                         int wsh, 
                                         float _gammaC, 
                                         float _gammaP)
{
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCamId];
    const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[tcDeviceCamId];

    float3 p = ptch.p;
    const float2 rp = project3DPoint(rcDeviceCamParams.P, p);
    const float2 tp = project3DPoint(tcDeviceCamParams.P, p);

    const float dd = wsh + 2.0f; // TODO FACA
    if((rp.x < dd) || (rp.x > float(rcWidth - 1) - dd) || (rp.y < dd) || (rp.y > float(rcHeight - 1) - dd) ||
       (tp.x < dd) || (tp.x > float(tcWidth - 1) - dd) || (tp.y < dd) || (tp.y > float(tcHeight - 1) - dd))
    {
        return CUDART_INF_F; // uninitialized
    }

    // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j exactly
    // value od I(i,j) ... it is what we want
    const float4 gcr = tex2D_float4(rcTex, rp.x + 0.5f, rp.y + 0.5f);
    const float4 gct = tex2D_float4(tcTex, tp.x + 0.5f, tp.y + 0.5f);

    // printf("gcr: R: %f, G: %f, B: %f, A: %f", gcr.x, gcr.y, gcr.z, gcr.w);
    // printf("gct: R: %f, G: %f, B: %f, A: %f", gct.x, gct.y, gct.z, gct.w);

    if(gcr.w == 0.0f || gct.w == 0.0f)
        return CUDART_INF_F; // if no alpha, invalid pixel from input mask

    const float gammaC = _gammaC;
    const float gammaP = _gammaP;
    // float gammaC = ((gcr.w>0)||(gct.w>0))?sigmoid(_gammaC,25.5f,20.0f,10.0f,fmaxf(gcr.w,gct.w)):_gammaC;
    // float gammaP = ((gcr.w>0)||(gct.w>0))?sigmoid(1.5,(float)(wsh+3),30.0f,20.0f,fmaxf(gcr.w,gct.w)):_gammaP;

    simStat sst;
    for(int yp = -wsh; yp <= wsh; yp++)
    {
        for(int xp = -wsh; xp <= wsh; xp++)
        {
            p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
            const float2 rp1 = project3DPoint(rcDeviceCamParams.P, p);
            const float2 tp1 = project3DPoint(tcDeviceCamParams.P, p);

            // see CUDA_C_Programming_Guide.pdf ... E.2 pp132-133 ... adding 0.5 caises that tex2D return for point i,j
            // exactly value od I(i,j) ... it is what we want
            const float4 gcr1 = tex2D_float4(rcTex, rp1.x + 0.5f, rp1.y + 0.5f);
            const float4 gct1 = tex2D_float4(tcTex, tp1.x + 0.5f, tp1.y + 0.5f);

            // TODO: Does it make a difference to accurately test it for each pixel of the patch?
            // if (gcr1.w == 0.0f || gct1.w == 0.0f)
            //     continue;

            // Weighting is based on:
            //  * color difference to the center pixel of the patch:
            //    ** low value (close to 0) means that the color is different from the center pixel (ie. strongly
            //    supported surface)
            //    ** high value (close to 1) means that the color is close the center pixel (ie. uniform color)
            //  * distance in image to the center pixel of the patch:
            //    ** low value (close to 0) means that the pixel is close to the center of the patch
            //    ** high value (close to 1) means that the pixel is far from the center of the patch
            const float w =
                CostYKfromLab(xp, yp, gcr, gcr1, gammaC, gammaP) * CostYKfromLab(xp, yp, gct, gct1, gammaC, gammaP);

            assert(w >= 0.f);
            assert(w <= 1.f);

            sst.update(gcr1.x, gct1.x, w);
        }
    }
    return sst.computeWSim();
}

__device__ static void getPixelFor3DPoint(int deviceCamId, float2& out, float3& X)
{
    const DeviceCameraParams& deviceCamParams = constantCameraParametersArray_d[deviceCamId];

    float3 p = M3x4mulV3(deviceCamParams.P, X);

    if(p.z <= 0.0f)
    {
        out = make_float2(-1.0f, -1.0f);
    }
    else
    {
        out = make_float2(p.x / p.z, p.y / p.z);
    }
}

__device__ static float3 get3DPointForPixelAndFrontoParellePlaneRC(int deviceCamId, const float2& pix, float fpPlaneDepth)
{
    const DeviceCameraParams& deviceCamParams = constantCameraParametersArray_d[deviceCamId];
    const float3 planep = deviceCamParams.C + deviceCamParams.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams.iP, pix);
    normalize(v);
    return linePlaneIntersect(deviceCamParams.C, v, planep, deviceCamParams.ZVect);
}

__device__ static float3 get3DPointForPixelAndFrontoParellePlaneRC(int deviceCamId, const int2& pixi, float fpPlaneDepth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndFrontoParellePlaneRC(deviceCamId, pix, fpPlaneDepth);
}

__device__ static float3 get3DPointForPixelAndDepthFromRC(int deviceCamId, const float2& pix, float depth)
{
    const DeviceCameraParams& deviceCamParams = constantCameraParametersArray_d[deviceCamId];
    float3 rpv = M3x3mulV2(deviceCamParams.iP, pix);
    normalize(rpv);
    return deviceCamParams.C + rpv * depth;
}

__device__ static float3 get3DPointForPixelAndDepthFromRC(int deviceCamId, const int2& pixi, float depth)
{
    float2 pix;
    pix.x = float(pixi.x);
    pix.y = float(pixi.y);
    return get3DPointForPixelAndDepthFromRC(deviceCamId, pix, depth);
}

__device__ static float3 triangulateMatchRef(int rcDeviceCamId, int tcDeviceCamId, float2& refpix, float2& tarpix)
{
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCamId];
    const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[tcDeviceCamId];

    float3 refvect = M3x3mulV2(rcDeviceCamParams.iP, refpix);
    normalize(refvect);
    float3 refpoint = refvect + rcDeviceCamParams.C;

    float3 tarvect = M3x3mulV2(tcDeviceCamParams.iP, tarpix);
    normalize(tarvect);
    float3 tarpoint = tarvect + tcDeviceCamParams.C;

    float k, l;
    float3 lli1, lli2;

    lineLineIntersect(&k, &l, &lli1, &lli2, rcDeviceCamParams.C, refpoint, tcDeviceCamParams.C, tarpoint);

    return rcDeviceCamParams.C + refvect * k;
}

__device__ static float computePixSize(int deviceCamId, const float3& p)
{
    const DeviceCameraParams& deviceCamParams = constantCameraParametersArray_d[deviceCamId];

    float2 rp = project3DPoint(deviceCamParams.P, p);
    float2 rp1 = rp + make_float2(1.0f, 0.0f);

    float3 refvect = M3x3mulV2(deviceCamParams.iP, rp1);
    normalize(refvect);
    return pointLineDistance3D(p, deviceCamParams.C, refvect);
}

__device__ static float refineDepthSubPixel(const float3& depths, const float3& sims)
{
    // subpixel refinement
    // subpixel refine by Stereo Matching with Color-Weighted Correlation, Hierarchical Belief Propagation, and
    // Occlusion Handling Qingxiong pami08
    // quadratic polynomial interpolation is used to approximate the cost function between three discrete depth
    // candidates: d, dA, and dB.
    // TODO: get formula back from paper as it has been lost by encoding.
    // d is the discrete depth with the minimal cost, dA ? d A 1, and dB ? d B 1. The cost function is approximated as
    // f?x? ? ax2 B bx B c.

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
