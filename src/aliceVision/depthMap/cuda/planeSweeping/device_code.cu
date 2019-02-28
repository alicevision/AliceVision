// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/planeSweeping/device_utils.cu>

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

// Global data handlers and parameters

/*
// defines
texture<unsigned char, 2, cudaReadModeNormalizedFloat> rtex;
texture<unsigned char, 2, cudaReadModeNormalizedFloat> ttex;
*/

#define DCT_DIMENSION 7

#define MAX_CUDA_DEVICES 10

texture<int4, 2, cudaReadModeElementType> volPixsTex;

texture<int2, 2, cudaReadModeElementType> pixsTex;

texture<float2, 2, cudaReadModeElementType> gradTex;

texture<float, 2, cudaReadModeElementType> depthsTex;

texture<float, 2, cudaReadModeElementType> depthsTex1;

texture<float4, 2, cudaReadModeElementType> normalsTex;

texture<float, 2, cudaReadModeElementType> sliceTex;

texture<float2, 2, cudaReadModeElementType> sliceTexFloat2;

texture<unsigned char, 2, cudaReadModeElementType> sliceTexUChar;

texture<uint2, 2, cudaReadModeElementType> sliceTexUInt2;

texture<unsigned int, 2, cudaReadModeElementType> sliceTexUInt;

texture<uchar4, 2, cudaReadModeElementType> rTexU4;

texture<uchar4, 2, cudaReadModeElementType> tTexU4;

texture<float4, 2, cudaReadModeElementType> f4Tex;

//////////////////////////////////////////////////////////////////////////////////////////////////////

__device__ unsigned char computeGradientSizeOfL(int x, int y)

{

    float xM1 = 255.0f * (tex2D(r4tex, (float)(x - 1) + 0.5f, (float)(y + 0) + 0.5f).x);
    float xP1 = 255.0f * (tex2D(r4tex, (float)(x + 1) + 0.5f, (float)(y + 0) + 0.5f).x);
    float yM1 = 255.0f * (tex2D(r4tex, (float)(x + 0) + 0.5f, (float)(y - 1) + 0.5f).x);
    float yP1 = 255.0f * (tex2D(r4tex, (float)(x + 0) + 0.5f, (float)(y + 1) + 0.5f).x);

    // not divided by 2?
    float2 g = make_float2(xM1 - xP1, yM1 - yP1);

    return (unsigned char)size(g);
}

__global__ void compute_varLofLABtoW_kernel(uchar4* labMap, int labMap_p, int width, int height, int wsh)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        uchar4* val = get2DBufferAt(labMap, labMap_p, x, y);

        // unsigned char sigma = computeSigmaOfL(x, y, wsh);
        // val->w = sigma;
        unsigned char grad = computeGradientSizeOfL(x, y);

        val->w = grad;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

__device__ void move3DPointByRcPixSize(float3& p, float rcPixSize)
{
    float3 rpv = p - sg_s_rC;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

__device__ void move3DPointByTcPixStep(float3& p, float tcPixStep)

{
    float3 rpv = sg_s_rC - p;
    float3 prp = p;
    float3 prp1 = p + rpv / 2.0f;

    float2 rp;
    getPixelFor3DPointRC(rp, prp);

    float2 tpo;
    getPixelFor3DPointTC(tpo, prp);

    float2 tpv;
    getPixelFor3DPointTC(tpv, prp1);

    tpv = tpv - tpo;
    normalize(tpv);

    float2 tpd = tpo + tpv * tcPixStep;

    p = triangulateMatchRef(rp, tpd);
}

__device__ float move3DPointByTcOrRcPixStep(int2& pix, float3& p, float pixStep, bool moveByTcOrRc)
{
    if(moveByTcOrRc == true)
    {
        move3DPointByTcPixStep(p, pixStep);
        return 0.0f;
    }
    else
    {
        float pixSize = pixStep * computePixSize(p);
        move3DPointByRcPixSize(p, pixSize);

        return pixSize;
    }
}

__device__ float3 computeDepthPoint_fine(float& pixSize, int depthid, int ndepths, int2& pix, int pixid, int t)
{
    float depth = tex2D(depthsTex, pixid, t);
    float2 rp = make_float2((float)pix.x, (float)pix.y);
    float3 rpv = M3x3mulV2(sg_s_riP, rp);
    normalize(rpv);

    float3 prp = sg_s_rC + rpv * depth;
    pixSize = computePixSize(prp);

    float jump = (float)(depthid - ((ndepths - 1) / 2));

    return sg_s_rC + rpv * (depth + pixSize * jump);
}

__global__ void locmin_kernel(float* slice, int slice_p, int ndepths, int slicesAtTime,
                              int width, int height, int wsh, int t, int npixs,
                              int maxDepth,
                              bool doUsePixelsDepths, int kernelSizeHalf)
{
    int depthid = blockIdx.x * blockDim.x + threadIdx.x;
    int pixid = blockIdx.y * blockDim.y + threadIdx.y;

    if((depthid < ndepths) && (pixid < slicesAtTime) && (slicesAtTime * t + pixid < npixs))
    {
        float sim0 = tex2D(sliceTex, depthid - 1, pixid);
        float sim1 = tex2D(sliceTex, depthid, pixid);
        float sim2 = tex2D(sliceTex, depthid + 1, pixid);
        float sim = 1.0f;

        if((sim0 > sim1) && (sim1 < sim2))
        {
            sim = sim1;
            if(kernelSizeHalf > 1)
            {
                for(int i = -kernelSizeHalf; i < kernelSizeHalf; i++)
                {
                    float simi = tex2D(sliceTex, depthid + i, pixid);
                    if(simi < sim1)
                    {
                        sim = 1.0f;
                    }
                }
            }
        }

        // coalescent
        float* s = get2DBufferAt(slice, slice_p, depthid, pixid);
        *s = sim;
    }
}

__device__ float2 computeMagRot(float l, float r, float u, float d)
{
    float2 out;
    float dx = r - l;
    float dy = u - d;
    float grd = 0.5f * sqrtf(dx * dx + dy * dy);
    float rot = (grd == 0.0f ? 0.0f : (atan2(dy, dx) / (CUDART_PI_F / 180.0f)));

    rot = (rot < 0.0f ? 360.0f + rot : rot);
    rot = (rot < 0.0f ? 0.0f : rot);
    rot = (rot >= 360.0f ? 0.0f : rot);

    out.x = grd;
    out.y = rot;

    return out;
}

__device__ float2 computeMagRotL(int x, int y)
{
    float l = 255.0f * tex2D(rtex, (float)(x - 1) + 0.5f, (float)y + 0.5f);
    float r = 255.0f * tex2D(rtex, (float)(x + 1) + 0.5f, (float)y + 0.5f);
    float u = 255.0f * tex2D(rtex, (float)x + 0.5f, (float)(y - 1) + 0.5f);
    float d = 255.0f * tex2D(rtex, (float)x + 0.5f, (float)(y + 1) + 0.5f);

    return computeMagRot(l, r, u, d);
}

__global__ void grad_kernel(float2* grad, int grad_p,
                            int2* pixs, int pixs_p,
                            int slicesAtTime, int ntimes,
                            int width, int height, int wsh, int npixs)
{
    int pixid = blockIdx.x * blockDim.x + threadIdx.x;
    int t = blockIdx.y * blockDim.y + threadIdx.y;
    if((pixid < slicesAtTime) && (slicesAtTime * t + pixid < npixs))
    {
        int2 pix = *get2DBufferAt(pixs, pixs_p, pixid, t);
        *get2DBufferAt(grad, grad_p, pixid, t) = computeMagRotL(pix.x, pix.y);
    }
}

__global__ void getRefTexLAB_kernel(uchar4* texs, int texs_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        *get2DBufferAt(texs, texs_p, x, y) = float4_to_uchar4(255.0f * tex2D(r4tex, (float)x + 0.5f, (float)y + 0.5f));
    }
}

__global__ void getTarTexLAB_kernel(uchar4* texs, int texs_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        *get2DBufferAt(texs, texs_p, x, y) = float4_to_uchar4(255.0f * tex2D(t4tex, (float)x + 0.5f, (float)y + 0.5f));
    }
}

__global__ void reprojTarTexLAB_kernel(uchar4* texs, int texs_p, int width, int height, float fpPlaneDepth)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
        float2 tpc = project3DPoint(sg_s_tP, p);
        uchar4* tex = get2DBufferAt(texs, texs_p, x, y);
        if(((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) &&
           ((tpc.x + 0.5f) < (float)width - 1.0f) && ((tpc.y + 0.5f) < (float)height - 1.0f))
        {
            *tex = float4_to_uchar4(255.0f * tex2D(t4tex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
        }
        else
        {
            tex->x = 0;
            tex->y = 0;
            tex->z = 0;
            tex->w = 0;
        }
    }
}

__global__ void reprojTarTexRgb_kernel(uchar4* texs, int texs_p, int width, int height, float fpPlaneDepth)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int2 pix;
    pix.x = x;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
        float2 tpc = project3DPoint(sg_s_tP, p);
        uchar4* tex = get2DBufferAt(texs, texs_p, x, y);
        if(((tpc.x + 0.5f) > 0.0f) && ((tpc.y + 0.5f) > 0.0f) &&
           ((tpc.x + 0.5f) < (float)width - 1.0f) && ((tpc.y + 0.5f) < (float)height - 1.0f))
        {
            tex->x = (unsigned char)(255.0f * tex2D(rtex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            tex->y = (unsigned char)(255.0f * tex2D(gtex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            tex->z = (unsigned char)(255.0f * tex2D(btex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            tex->w = 1;
        }
        else
        {
            tex->x = 0;
            tex->y = 0;
            tex->z = 0;
            tex->w = 0;
        }
    }
}

__global__ void copyUchar4Dim2uchar_kernel(int dim, uchar4* src, int src_p, unsigned char* tar, int tar_p, int width,
                                           int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        unsigned char* t = get2DBufferAt(tar, tar_p, x, y);
        const uchar4& s = *get2DBufferAt(src, src_p, x, y);
        switch(dim)
        {
            case 0:
                *t = s.x;

            case 1:
                *t = s.y;

            case 2:
                *t = s.z;
        }
    }
}

__global__ void transpose_uchar4_kernel(uchar4* input, int input_p, uchar4* output, int output_p, int width, int height)
{
    __shared__ uchar4 temp[BLOCK_DIM][BLOCK_DIM];
    int xIndex = blockIdx.x * BLOCK_DIM + threadIdx.x;
    int yIndex = blockIdx.y * BLOCK_DIM + threadIdx.y;
    if((xIndex < width) && (yIndex < height))
    {
        temp[threadIdx.y][threadIdx.x] = *get2DBufferAt(input, input_p, xIndex, yIndex);
    }

    __syncthreads();

    xIndex = blockIdx.y * BLOCK_DIM + threadIdx.x;
    yIndex = blockIdx.x * BLOCK_DIM + threadIdx.y;
    if((xIndex < height) && (yIndex < width))
    {
        *get2DBufferAt(output, output_p, xIndex, yIndex) = temp[threadIdx.x][threadIdx.y];
    }
}

__global__ void transpose_float4_kernel(float4* input, int input_p, float4* output, int output_p, int width, int height)
{
    __shared__ float4 temp[BLOCK_DIM][BLOCK_DIM];
    int xIndex = blockIdx.x * BLOCK_DIM + threadIdx.x;
    int yIndex = blockIdx.y * BLOCK_DIM + threadIdx.y;

    if((xIndex < width) && (yIndex < height))
    {
        temp[threadIdx.y][threadIdx.x] = *get2DBufferAt(input, input_p, xIndex, yIndex);
    }

    __syncthreads();

    xIndex = blockIdx.y * BLOCK_DIM + threadIdx.x;

    yIndex = blockIdx.x * BLOCK_DIM + threadIdx.y;

    if((xIndex < height) && (yIndex < width))
    {
        *get2DBufferAt(output, output_p, xIndex, yIndex) = temp[threadIdx.x][threadIdx.y];
    }
}

__global__ void compAggrNccSim_kernel(
    float4* ostat1, int ostat1_p,
    float4* ostat2, int ostat2_p,
    uchar4* rImIn, int rImIn_p,
    uchar4* tImIn, int tImIn_p,
    int width, int height, int step, int orintation)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    const int blockSize = 64;

    __shared__ uchar4 rIm[blockSize];
    __shared__ uchar4 tIm[blockSize];

    if((x < width - step) && (y < height))
    {
        rIm[threadIdx.x] = *get2DBufferAt(rImIn, rImIn_p, x + step, y);
        tIm[threadIdx.x] = *get2DBufferAt(tImIn, tImIn_p, x + step, y);
    }

    __syncthreads();

    if((x < width) && (y < height) && (threadIdx.x >= 16) && (threadIdx.x < blockSize - 16))
    {
        uchar4 cr1 = rIm[threadIdx.x];
        uchar4 ct1 = tIm[threadIdx.x];
        float3 cr1f = xyz2lab(rgb2xyz(uchar4_to_float3(cr1)));
        float3 ct1f = xyz2lab(rgb2xyz(uchar4_to_float3(ct1)));

        if((ct1.x > 0) || (ct1.y > 0) || (ct1.z > 0))
        {
            simStat sst = simStat();
            {
                float w = 1.0f;
                sst.update(cr1f.x, ct1f.x, w);
            }

            for(int sign = 0; sign < 2; sign++)
            {
                for(int i = 1; i <= 16; i++)
                {
                    uchar4 cr2 = rIm[threadIdx.x + i * (sign * 2 - 1)];
                    uchar4 ct2 = tIm[threadIdx.x + i * (sign * 2 - 1)];
                    float3 cr2f = xyz2lab(rgb2xyz(uchar4_to_float3(cr2)));
                    float3 ct2f = xyz2lab(rgb2xyz(uchar4_to_float3(ct2)));
                    if((ct2.x > 0) || (ct2.y > 0) || (ct2.z > 0))
                    {
                        float w = CostYK(i, 0, cr1, cr2, 5.0f, 16.0f) * CostYK(i, 0, ct1, ct2, 5.0f, 16.0f);
                        sst.update(cr2f.x, ct2f.x, w);
                    }
                }
            }

            float4* s1 = get2DBufferAt(ostat1, ostat1_p, x + step, y);
            float4* s2 = get2DBufferAt(ostat2, ostat2_p, x + step, y);

            if(orintation == 0)
            {
                s1->x = sst.xsum;
                s1->y = sst.ysum;
                s1->z = sst.xxsum;

                s2->x = sst.yysum;
                s2->y = sst.xysum;
                s2->z = sst.count;
                s2->w = sst.wsum;
            }
            else
            {
                s1->x += sst.xsum;
                s1->y += sst.ysum;
                s1->z += sst.xxsum;

                s2->x += sst.yysum;
                s2->y += sst.xysum;
                s2->z += sst.count;
                s2->w += sst.wsum;
            }
        }
    }
}

__global__ void compNccSimFromStats_kernel(
    float* odepth, int odepth_p,
    float* osim, int osim_p,
    float4* stat1, int stat1_p,
    float4* stat2, int stat2_p,
    int width, int height, int d, float depth)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        simStat sst = simStat();
        const float4& s1 = *get2DBufferAt(stat1, stat1_p, x, y);
        sst.xsum = s1.x;
        sst.ysum = s1.y;
        sst.xxsum = s1.z;

        const float4& s2 = *get2DBufferAt(stat2, stat2_p, x, y);
        sst.yysum = s2.x;
        sst.xysum = s2.y;
        sst.count = s2.z;
        sst.wsum  = s2.w;

        float sim = 1.0f;

        if(sst.count > 9.0f)
        {
            sst.computeWSim();
            sim = sst.sim;
        }

        if(d == 0)
        {
            *get2DBufferAt(osim, osim_p, x, y) = sim;
            *get2DBufferAt(odepth, odepth_p, x, y) = depth;
        }
        else
        {
            float oldsim = *get2DBufferAt(osim, osim_p, x, y);
            float olddepth = *get2DBufferAt(odepth, odepth_p, x, y);

            if(sim < oldsim)
            {
                *get2DBufferAt(osim, osim_p, x, y) = sim;
                *get2DBufferAt(odepth, odepth_p, x, y) = depth;
            }
            else
            {
                *get2DBufferAt(osim, osim_p, x, y) = oldsim;
                *get2DBufferAt(odepth, osim_p, x, y) = olddepth;
            }
        }
    }
}

__global__ void compWshNccSim_kernel(float* osim, int osim_p, int width, int height, int wsh, int step)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step > wsh) && (y * step > wsh) && (x * step < width - wsh) && (y * step < height - wsh))
    {
        simStat sst = simStat();
        for(int yp = y * step - wsh; yp <= y * step + wsh; yp++)
        {
            for(int xp = x * step - wsh; xp <= x * step + wsh; xp++)
            {
                uchar4 rc = tex2D(rTexU4, xp, yp);
                uchar4 tc = tex2D(tTexU4, xp, yp);

                sst.update((float)rc.x, (float)tc.x, 1.0f);
            }
        }

        sst.computeWSim();
        *get2DBufferAt(osim, osim_p, x, y) = sst.sim;
    }
}

__global__ void aggrYKNCCSim_kernel(float* osim, int osim_p, int width, int height, int wsh, int step,
                                    const float gammaC, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step > wsh) && (y * step > wsh) && (x * step < width - wsh) && (y * step < height - wsh))
    {
        float sums = 0.0f;
        float sumw = 0.0f;

        float4 rcm = uchar4_to_float4(tex2D(rTexU4, x, y));
        float4 tcm = uchar4_to_float4(tex2D(tTexU4, x, y));

        for(int yp = -wsh; yp <= +wsh; yp++)
        {
            for(int xp = -wsh; xp <= +wsh; xp++)
            {
                float4 rc = uchar4_to_float4(tex2D(rTexU4, x * step + xp, y * step + yp));
                float4 tc = uchar4_to_float4(tex2D(tTexU4, x * step + xp, y * step + yp));
                float s = tex2D(sliceTex, x * step + xp, y * step + yp);

                float w =
                    CostYKfromLab(xp, yp, rcm, rc, gammaC, gammaP) * CostYKfromLab(xp, yp, tcm, tc, gammaC, gammaP);

                sums += s * w;
                sumw += w;
            }
        }

        *get2DBufferAt(osim, osim_p, x, y) = sums / sumw;
    }
}

__global__ void updateBestDepth_kernel(
    float* osim, int osim_p,
    float* odpt, int odpt_p,
    float* isim, int isim_p,
    int width, int height, int step, float fpPlaneDepth, int d)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step > 0) && (y * step > 0) && (x * step < width) && (y * step < height))
    {
        float is = *get2DBufferAt(isim, isim_p, x, y);
        float os = *get2DBufferAt(osim, osim_p, x, y);

        if((is < os) || (d == 0))
        {
            int2 pix = make_int2(x * step, y * step);
            float3 p = get3DPointForPixelAndFrontoParellePlaneRC(pix, fpPlaneDepth);
            float depth = size(sg_s_rC - p);

            *get2DBufferAt(osim, osim_p, x, y) = is;
            *get2DBufferAt(odpt, odpt_p, x, y) = depth;
        }
    }
}

__global__ void downscale_gauss_smooth_lab_kernel(
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float4 t = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sum = 0.0f;
        for(int i = -radius; i <= radius; i++)
        {
            for(int j = -radius; j <= radius; j++)
            {
                float4 curPix = 255.0f * tex2D(r4tex, (float)(x * scale + j) + (float)scale / 2.0f,
                                               (float)(y * scale + i) + (float)scale / 2.0f);
                float factor = (tex1D(gaussianTex, i + radius) * tex1D(gaussianTex, j + radius)); // domain factor
                t = t + curPix * factor;
                sum += factor;
            }
        }
        t.x = t.x / sum;
        t.y = t.y / sum;
        t.z = t.z / sum;
        t.w = t.w / sum;

        uchar4 tu4 = make_uchar4((unsigned char)t.x, (unsigned char)t.y, (unsigned char)t.z, (unsigned char)t.w);

        *get2DBufferAt(texLab, texLab_p, x, y) = tu4; 
    }
}

__global__ void getSilhoueteMap_kernel(bool* out, int out_p, int step, int width, int height, const uchar4 maskColorLab)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step < width) && (y * step < height))
    {
        uchar4 col = tex2D(rTexU4, x * step, y * step);
        *get2DBufferAt(out, out_p, x, y) = ((maskColorLab.x == col.x) && (maskColorLab.y == col.y) && (maskColorLab.z == col.z));
    }
}

__global__ void computeNormalMap_kernel(
  float3* nmap, int nmap_p,
  int width, int height, int wsh, const float gammaC, const float gammaP)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if ((x >= width) || (y >= height))
    return;

  float depth = tex2D(depthsTex, x, y);
  if(depth <= 0.0f)
  {
    *get2DBufferAt(nmap, nmap_p, x, y) = make_float3(-1.f, -1.f, -1.f);
    return;
  }

  int2 pix1 = make_int2(x, y);
  float3 p = get3DPointForPixelAndDepthFromRC(pix1, depth);
  float pixSize = 0.0f;
  {
    int2 pix2 = make_int2(x + 1, y);
    float3 p2 = get3DPointForPixelAndDepthFromRC(pix2, depth);
    pixSize = size(p - p2);
  }

  cuda_stat3d s3d = cuda_stat3d();

  for (int yp = -wsh; yp <= wsh; yp++)
  {
    for (int xp = -wsh; xp <= wsh; xp++)
    {
      float depthn = tex2D(depthsTex, x + xp, y + yp);
      if ((depth > 0.0f) && (fabs(depthn - depth) < 30.0f * pixSize))
      {
        float w = 1.0f;
        float2 pixn = make_float2(x + xp, y + yp);
        float3 pn = get3DPointForPixelAndDepthFromRC(pixn, depthn);
        s3d.update(pn, w);
      }
    }
  }

  float3 pp = p;
  float3 nn = make_float3(-1.f, -1.f, -1.f);
  if(!s3d.computePlaneByPCA(pp, nn))
  {
    *get2DBufferAt(nmap, nmap_p, x, y) = make_float3(-1.f, -1.f, -1.f);
    return;
  }

  float3 nc = sg_s_rC - p;
  normalize(nc);
  if (orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
  {
    nn.x = -nn.x;
    nn.y = -nn.y;
    nn.z = -nn.z;
  }
  *get2DBufferAt(nmap, nmap_p, x, y) = nn;
}

} // namespace depthMap
} // namespace aliceVision
