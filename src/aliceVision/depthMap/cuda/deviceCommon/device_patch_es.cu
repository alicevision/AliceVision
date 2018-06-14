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

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

__device__ void computeRotCSEpip(patch& ptch, const float3& p)
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

__device__ int angleBetwUnitV1andUnitV2(float3& V1, float3& V2)
{
    return (int)fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (CUDART_PI_F / 180.0f));
}

__device__ bool checkPatch(patch& ptch, int angThr)
{

    float3 rv = (sg_s_rC - ptch.p);
    float3 tv = (sg_s_tC - ptch.p);
    normalize(rv);
    normalize(tv);

    float3 n = ptch.n;

    if(size(sg_s_rC - (ptch.p + ptch.n)) > size(sg_s_rC - (ptch.p - ptch.n)))
    {
        n.x = -n.x;
        n.y = -n.y;
        n.z = -n.z;
    }

    return ((angleBetwUnitV1andUnitV2(rv, n) < angThr) && (angleBetwUnitV1andUnitV2(tv, n) < angThr));
}

/*
__device__ float getRefCamPixSize(patch &ptch)
{
        float2 rp = project3DPoint(sg_s_rP,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = rp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_riP,pix);
                float3 lpi = linePlaneIntersect(sg_s_rC, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
};


__device__ float getTarCamPixSize(patch &ptch)
{
        float2 tp = project3DPoint(sg_s_tP,ptch.p);

        float minstep=10000000.0f;
        for (int i=0;i<4;i++) {
                float2 pix = tp;
                if (i==0) {pix.x += 1.0f;};
                if (i==1) {pix.x -= 1.0f;};
                if (i==2) {pix.y += 1.0f;};
                if (i==3) {pix.y -= 1.0f;};
                float3 vect = M3x3mulV2(sg_s_tiP,pix);
                float3 lpi = linePlaneIntersect(sg_s_tC, vect, ptch.p, ptch.n);
                float step = dist(lpi,ptch.p);
                minstep = fminf(minstep,step);
        };

        return minstep;
};


__device__ float getPatchPixSize(patch &ptch)
{
        return fmaxf(getTarCamPixSize(ptch),getRefCamPixSize(ptch));
}
*/

__device__ void computeHomography(float* _H, float3& _p, float3& _n)
{
    // hartley zisserman second edition p.327 (13.2)
    float3 _tl = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(sg_s_rR, sg_s_rC);
    float3 _tr = make_float3(0.0, 0.0, 0.0) - M3x3mulV3(sg_s_tR, sg_s_tC);

    float3 p = M3x3mulV3(sg_s_rR, (_p - sg_s_rC));
    float3 n = M3x3mulV3(sg_s_rR, _n);
    normalize(n);
    float d = -dot(n, p);

    float RrT[9];
    M3x3transpose(RrT, sg_s_rR);

    float tmpRr[9];
    M3x3mulM3x3(tmpRr, sg_s_tR, RrT);
    float3 tr = _tr - M3x3mulV3(tmpRr, _tl);

    float tmp[9];
    float tmp1[9];
    outerMultiply(tmp, tr, n / d);
    M3x3minusM3x3(tmp, tmpRr, tmp);
    M3x3mulM3x3(tmp1, sg_s_tK, tmp);
    M3x3mulM3x3(tmp, tmp1, sg_s_riK);

    for(int i = 0; i < 9; i++)
    {
        _H[i] = tmp[i];
    };
}

__device__ float compNCC(float2& rpix, float2& tpix, int wsh)
{
    simStat sst = simStat();
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            float2 rp;
            float2 tp;
            rp.x = rpix.x + (float)xp;
            rp.y = rpix.y + (float)yp;
            tp.x = tpix.x + (float)xp;
            tp.y = tpix.y + (float)yp;

            float2 g;
            // g.x = 255.0f*tex2D(rtex, rp.x+0.5, rp.y+0.5);
            // g.y = 255.0f*tex2D(ttex, tp.x+0.5, tp.y+0.5);
            g.x = tex2D(rtex, rp.x + 0.5f, rp.y + 0.5f);
            g.y = tex2D(ttex, tp.x + 0.5f, tp.y + 0.5f);
            sst.update(g);
        };
    };
    sst.computeSim();

    return sst.sim;
}

__device__ float compNCCvarThr(float2& rpix, float2& tpix, int wsh, float varThr)
{
    simStat sst = simStat();
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            float2 rp;
            float2 tp;
            rp.x = rpix.x + (float)xp;
            rp.y = rpix.y + (float)yp;
            tp.x = tpix.x + (float)xp;
            tp.y = tpix.y + (float)yp;

            float2 g;
            g.x = 255.0f * tex2D(rtex, rp.x + 0.5, rp.y + 0.5);
            g.y = 255.0f * tex2D(ttex, tp.x + 0.5, tp.y + 0.5);
            // g.x = tex2D(rtex, rp.x+0.5f, rp.y+0.5f);
            // g.y = tex2D(ttex, tp.x+0.5f, tp.y+0.5f);
            sst.update(g);
        };
    };

    sst.computeSim();

    float sim = 1.0f;
    if((4.0f * sst.getVarianceX() > varThr * varThr) && (4.0f * sst.getVarianceY() > varThr * varThr))
    {
        sim = sst.sim;
    };
    return sim;
}

__device__ simStat compSimStat(float2& rpix, float2& tpix, int wsh)
{
    simStat sst = simStat();
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            float2 rp;
            float2 tp;
            rp.x = rpix.x + (float)xp;
            rp.y = rpix.y + (float)yp;
            tp.x = tpix.x + (float)xp;
            tp.y = tpix.y + (float)yp;

            float2 g;
            g.x = 255.0f * tex2D(rtex, rp.x + 0.5, rp.y + 0.5);
            g.y = 255.0f * tex2D(ttex, tp.x + 0.5, tp.y + 0.5);
            // g.x = tex2D(rtex, rp.x+0.5f, rp.y+0.5f);
            // g.y = tex2D(ttex, tp.x+0.5f, tp.y+0.5f);
            sst.update(g);
        };
    };

    return sst;
}

__device__ float compNCCbyH(patch& ptch, int wsh)
{
    float2 rpix = project3DPoint(sg_s_rP, ptch.p);
    float2 tpix = project3DPoint(sg_s_tP, ptch.p);

    float H[9];
    computeHomography(H, ptch.p, ptch.n);

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
            // g.x = tex2D(rtex, rp.x+0.5f, rp.y+0.5f);
            // g.y = tex2D(ttex, tp.x+0.5f, tp.y+0.5f);
            sst.update(g);
        };
    };
    sst.computeSim();

    return sst.sim;
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
__device__ float compNCCby3DptsYK(patch& ptch, int wsh, int width, int height, const float _gammaC, const float _gammaP,
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
    float4 gcr = 255.0f * tex2D(r4tex, rp.x + 0.5f, rp.y + 0.5f);
    float4 gct = 255.0f * tex2D(t4tex, tp.x + 0.5f, tp.y + 0.5f);
    // gcr = 255.0f*tex2D(r4tex, rp.x, rp.y);
    // gct = 255.0f*tex2D(t4tex, tp.x, tp.y);

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
            float4 gcr1f = tex2D(r4tex, rp1.x + 0.5f, rp1.y + 0.5f);
            float4 gct1f = tex2D(t4tex, tp1.x + 0.5f, tp1.y + 0.5f);
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

/*
__device__ float compNCCby3DptsYK(patch &ptch, int wsh, int width, int height, const float gammaC, const float gammaP,
const float epipShift)
{
        float3 p =  ptch.p;
        float2 rp = project3DPoint(sg_s_rP, p);
        float2 tp = project3DPoint(sg_s_tP, p);
        float2 tpUp = project3DPoint(sg_s_tP, p+ptch.y*(ptch.d*10.0f)); //assuming that ptch.y is ortogonal to epipolar
plane
        float2 vEpipShift = tpUp-tp;
        normalize(vEpipShift);
        vEpipShift = vEpipShift*epipShift;
        tp = tp + vEpipShift;


        float dd = (float)(wsh*2);
        if  (!((rp.x>dd)&&(rp.x<(float)width-dd)&&(rp.y>dd)&&(rp.y<(float)height-dd)&&
                   (tp.x>dd)&&(tp.x<(float)width-dd)&&(tp.y>dd)&&(tp.y<(float)height-dd)))
        {
                return 1.0f;
        };

        float3 gcr,gct, gcr1, gct1;


        gcr.x = 255.0f*tex2D(rtex0, rp.x+0.5f, rp.y+0.5f);
        gcr.y = 255.0f*tex2D(rtex1, rp.x+0.5f, rp.y+0.5f);
        gcr.z = 255.0f*tex2D(rtex2, rp.x+0.5f, rp.y+0.5f);
        gct.x = 255.0f*tex2D(ttex0, tp.x+0.5f, tp.y+0.5f);
        gct.y = 255.0f*tex2D(ttex1, tp.x+0.5f, tp.y+0.5f);
        gct.z = 255.0f*tex2D(ttex2, tp.x+0.5f, tp.y+0.5f);


        float sumsim = 0.0f;
        float wsim = 0.0f;

        for (int xp=-wsh;xp<=wsh;xp++)
        {
                for (int yp=-wsh;yp<=wsh;yp++)
                {


                        simStat sst = simStat();
                        for (int xpp=-1;xpp<=1;xpp++)
                        {
                                for (int ypp=-1;ypp<=1;ypp++)
                                {
                                        p =
ptch.p+ptch.x*(float)(ptch.d*(float)(xp+xpp))+ptch.y*(float)(ptch.d*(float)(yp+ypp));
                                        rp = project3DPoint(sg_s_rP, p);
                                        tp = project3DPoint(sg_s_tP, p)+vEpipShift;
                                        gcr1.x = 255.0f*tex2D(rtex0, rp.x+0.5f, rp.y+0.5f);
                                        gct1.x = 255.0f*tex2D(ttex0, tp.x+0.5f, tp.y+0.5f);
                                        sst.update((float)gcr1.x, (float)gct1.x, 1.0f);
                                };
                        };
                        sst.computeWSim();


                        p = ptch.p+ptch.x*(float)(ptch.d*(float)xp)+ptch.y*(float)(ptch.d*(float)yp);
                        rp = project3DPoint(sg_s_rP, p);
                        tp = project3DPoint(sg_s_tP, p)+vEpipShift;
                        gcr1.x = 255.0f*tex2D(rtex0, rp.x+0.5f, rp.y+0.5f);
                        gcr1.y = 255.0f*tex2D(rtex1, rp.x+0.5f, rp.y+0.5f);
                        gcr1.z = 255.0f*tex2D(rtex2, rp.x+0.5f, rp.y+0.5f);
                        gct1.x = 255.0f*tex2D(ttex0, tp.x+0.5f, tp.y+0.5f);
                        gct1.y = 255.0f*tex2D(ttex1, tp.x+0.5f, tp.y+0.5f);
                        gct1.z = 255.0f*tex2D(ttex2, tp.x+0.5f, tp.y+0.5f);

                        float w = CostYKfromLab(xp, yp, gcr, gcr1, gammaC, gammaP) * CostYKfromLab(xp, yp, gct, gct1,
gammaC, gammaP);
                        sumsim += sst.sim*w;
                        wsim += w;
                };
        };


        return sumsim/wsim;
}
*/

/**
 * @brief Compute Normalized Cross Correlation
 * @param[in] ptch image patch
 * @param[in] wsh half-size of the kernel
 * @param[in] width
 * @param[in] height
 */
__device__ float compNCCby3Dpts(patch& ptch, int wsh, int width, int height)
{
    float3 p = ptch.p;
    float2 rp = project3DPoint(sg_s_rP, p);
    float2 tp = project3DPoint(sg_s_tP, p);

    float dd = (float)(wsh + 2);
    if((rp.x < dd) || (rp.x > (float)width  - 1 - dd) ||
       (rp.y < dd) || (rp.y > (float)height - 1 - dd) ||
       (tp.x < dd) || (tp.x > (float)width  - 1 - dd) ||
       (tp.y < dd) || (tp.y > (float)height - 1 - dd))
    {
        return 1.0f;
    };

    simStat sst = simStat();
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
            rp = project3DPoint(sg_s_rP, p);
            tp = project3DPoint(sg_s_tP, p);
            float2 g;
            g.x = 255.0f * tex2D(rtex, rp.x + 0.5f, rp.y + 0.5f);
            g.y = 255.0f * tex2D(ttex, tp.x + 0.5f, tp.y + 0.5f);
            sst.update(g);
        };
    };
    sst.computeSim();
    return sst.sim;
}

__device__ float compNCCby3DptsEpipOpt(patch& ptch, int width, int height)
{
    float3 p = ptch.p;
    float2 rp = project3DPoint(sg_s_rP, p);
    float2 tp = project3DPoint(sg_s_tP, p);

    float dd = (float)(2 + 2);
    if(!((rp.x > dd) && (rp.x < (float)width - dd) && (rp.y > dd) && (rp.y < (float)height - dd) && (tp.x > dd) &&
         (tp.x < (float)width - dd) && (tp.y > dd) && (tp.y < (float)height - dd)))
    {
        return 1.0f;
    };

    const int wsh = 2;
    const int neer = 2;
    float sims[2 * neer + 1];

    float lim[2 * wsh + 1][2 * wsh + 1];
    for(int xp = -wsh; xp <= wsh; xp++)
    {
        for(int yp = -wsh; yp <= wsh; yp++)
        {
            p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
            rp = project3DPoint(sg_s_rP, p);
            lim[xp + wsh][yp + wsh] = 255.0f * tex2D(rtex, rp.x + 0.5f, rp.y + 0.5f);
        };
    };

    float2 v;
    p = ptch.p;
    tp = project3DPoint(sg_s_tP, p);
    p = ptch.p + ptch.y * (float)(ptch.d * 5.0f); // ptch.y - normal to epipolar plane
    v = tp - project3DPoint(sg_s_tP, p);
    normalize(v);
    v = v / 2.0f; // step by 0.5 pixel

    float minsim = 1.0f;
    float maxsim = -1.0f;

    for(int i = 0; i < 2 * neer + 1; i++)
    {
        simStat sst = simStat();
        for(int xp = -wsh; xp <= wsh; xp++)
        {
            for(int yp = -wsh; yp <= wsh; yp++)
            {
                p = ptch.p + ptch.x * (float)(ptch.d * (float)xp) + ptch.y * (float)(ptch.d * (float)yp);
                tp = project3DPoint(sg_s_tP, p);
                tp = tp + v * (float(i - neer));
                float2 g;
                g.x = lim[xp + wsh][yp + wsh];
                g.y = 255.0f * tex2D(ttex, tp.x + 0.5f, tp.y + 0.5f);
                sst.update(g);
            };
        };
        sst.computeSim();
        sims[i] = sst.sim;
        minsim = fminf(minsim, sst.sim);
        maxsim = fmaxf(maxsim, sst.sim);
    };

    if(minsim == sims[neer])
    {
        return minsim;
    }
    else
    {
        return 1.0f;
    };

    // return minsim;
}

__device__ void getPixelFor3DPointRC(float2& out, float3& X)
{
    float3 p = M3x4mulV3(sg_s_rP, X);
    out = make_float2(p.x / p.z, p.y / p.z);

    if(p.z < 0.0f)
    {
        out.x = -1.0f;
        out.y = -1.0f;
    };
}

__device__ void getPixelFor3DPointTC(float2& out, float3& X)
{
    float3 p = M3x4mulV3(sg_s_tP, X);
    out = make_float2(p.x / p.z, p.y / p.z);

    if(p.z < 0.0f)
    {
        out.x = -1.0f;
        out.y = -1.0f;
    };
}

__device__ float frontoParellePlaneRCDepthFor3DPoint(const float3& p)
{
    return fabsf(orientedPointPlaneDistanceNormalizedNormal(p, sg_s_rC, sg_s_rZVect));
}

__device__ float frontoParellePlaneTCDepthFor3DPoint(const float3& p)
{
    return fabsf(orientedPointPlaneDistanceNormalizedNormal(p, sg_s_tC, sg_s_tZVect));
}

__device__ float3 get3DPointForPixelAndFrontoParellePlaneRC(float2& pix, float fpPlaneDepth)
{
    float3 planep = sg_s_rC + sg_s_rZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(sg_s_riP, pix);
    normalize(v);
    return linePlaneIntersect(sg_s_rC, v, planep, sg_s_rZVect);
}

__device__ float3 get3DPointForPixelAndFrontoParellePlaneRC(int2& pixi, float fpPlaneDepth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
}

__device__ float3 get3DPointForPixelAndDepthFromRC(const float2& pix, float depth)
{
    float3 rpv = M3x3mulV2(sg_s_riP, pix);
    normalize(rpv);
    return sg_s_rC + rpv * depth;
}

__device__ float3 get3DPointForPixelAndDepthFromTC(const float2& pix, float depth)
{
    float3 tpv = M3x3mulV2(sg_s_tiP, pix);
    normalize(tpv);
    return sg_s_tC + tpv * depth;
}

__device__ float3 get3DPointForPixelAndDepthFromRC(const int2& pixi, float depth)
{
    float2 pix;
    pix.x = (float)pixi.x;
    pix.y = (float)pixi.y;
    return get3DPointForPixelAndDepthFromRC(pix, depth);
}

__device__ float3 triangulateMatchRef(float2& refpix, float2& tarpix)
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

__device__ float computeRcPixSize(const float3& p)
{
    /*
    patch ptcho;
    ptcho.p = p;
    computeRotCSEpip(ptcho,p);
    float2 rp = project3DPoint(sg_s_rP, p);

    float dRcTc = size(sg_s_rC-sg_s_tC)/100.0f;
    float3 pLeft = ptcho.p+ptcho.x*dRcTc; //assuming that ptch.x on epipolar plane
    float2 rvLeft = project3DPoint(sg_s_rP, pLeft); rvLeft = rvLeft-rp; normalize(rvLeft);

    float depth = size(sg_s_rC-p);
    float2 rp1 = rp + rvLeft;

    //float3 p1 = get3DPointForPixelAndDepthFromRC(rp1,depth);
    //float pixSize = size(p-p1);
    //return pixSize;

    float3 refvect = M3x3mulV2(sg_s_riP,rp1);
    normalize(refvect);
    return pointLineDistance3D(p, sg_s_rC, refvect);
    */

    float2 rp = project3DPoint(sg_s_rP, p);
    float2 rp1 = rp + make_float2(1.0f, 0.0f);

    float3 refvect = M3x3mulV2(sg_s_riP, rp1);
    normalize(refvect);
    return pointLineDistance3D(p, sg_s_rC, refvect);
}

__device__ float computePixSize(const float3& p)
{
    return computeRcPixSize(p);
}

__device__ float computeTcPixSize(const float3& p)
{
    float2 tp = project3DPoint(sg_s_tP, p);
    float2 tp1 = tp + make_float2(1.0f, 0.0f);

    float3 tarvect = M3x3mulV2(sg_s_tiP, tp1);
    normalize(tarvect);
    return pointLineDistance3D(p, sg_s_tC, tarvect);
}

__device__ float refineDepthSubPixel(const float3& depths, const float3& sims)
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
