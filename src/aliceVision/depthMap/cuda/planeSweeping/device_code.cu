// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/planeSweeping/device_code.cuh"

#include "aliceVision/depthMap/cuda/planeSweeping/device_utils.cuh"

#include "aliceVision/depthMap/cuda/deviceCommon/device_global.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_matrix.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_eig33.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_simStat.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cuh"

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

//////////////////////////////////////////////////////////////////////////////////////////////////////

__device__ unsigned char computeSigmaOfL( cudaTextureObject_t r4tex, int x, int y, int r)
{
    double xxsum = 0.0;
    double xsum = 0.0;
    double count = 0.0;

    for(int xp = -r; xp <= r; xp++)
    {
        for(int yp = -r; yp <= r; yp++)
        {
            float lValf = 255.0f * (tex2D<float4>(r4tex, (float)(x + xp) + 0.5f, (float)(y + yp) + 0.5f).x);
            double lVal = lValf;
            xsum += lVal;
            xxsum += lVal * lVal;
            count += 1.0;
        }
    }

    float sigma = xxsum / count - (xsum * xsum) / (count * count);

    return (unsigned char)(fminf(255.0f, sigma));
}

__device__ inline unsigned char computeGradientSizeOfL(
    cudaTextureObject_t r4tex,
    int x, int y)
{

    float xM1 = 255.0f * (tex2D<float4>(r4tex, (float)(x - 1) + 0.5f, (float)(y + 0) + 0.5f).x);
    float xP1 = 255.0f * (tex2D<float4>(r4tex, (float)(x + 1) + 0.5f, (float)(y + 0) + 0.5f).x);
    float yM1 = 255.0f * (tex2D<float4>(r4tex, (float)(x + 0) + 0.5f, (float)(y - 1) + 0.5f).x);
    float yP1 = 255.0f * (tex2D<float4>(r4tex, (float)(x + 0) + 0.5f, (float)(y + 1) + 0.5f).x);

    // not divided by 2?
    float2 g = make_float2(xM1 - xP1, yM1 - yP1);

    return (unsigned char)size(g);
}

__global__ void compute_varLofLABtoW_kernel(
    cudaTextureObject_t r4tex,
    uchar4* labMap, int labMap_p, int width, int height, int wsh)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < width && y < height)
    {
        uchar4* val = get2DBufferAt(labMap, labMap_p, x, y);

        // unsigned char sigma = computeSigmaOfL(x, y, wsh);
        // val->w = sigma;
        unsigned char grad = computeGradientSizeOfL( r4tex, x, y );

        val->w = grad;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


__global__ void smoothDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float depth = tex2D<float>(depthsTex, x, y);
        float pixSize = 0.0f;
        {
            float depth = tex2D<float>(depthsTex, x, y);
            int2 pix1 = make_int2(x, y);
            float3 p1 = get3DPointForPixelAndDepthFromRC(pix1, depth);
            int2 pix2 = make_int2(x + 1, y);
            float3 p2 = get3DPointForPixelAndDepthFromRC(pix2, depth);
            pixSize = size(p1 - p2);
        }

        float4 gcr = 255.0f * tex2D<float4>(r4tex, (float)x + 0.5f, (float)y + 0.5f);
        float depthUp = 0.0f;
        float depthDown = 0.0f;

        for(int yp = -wsh; yp <= wsh; yp++)
        {
            for(int xp = -wsh; xp <= wsh; xp++)
            {
                float depthn = tex2D<float>(depthsTex, x + xp, y + yp);
                if((depth > 0.0f) && (fabs(depthn - depth) < 10.0f * pixSize))
                {
                    float4 gcr1 = 255.0f * tex2D<float4>(r4tex, (float)(x + xp) + 0.5f, (float)(y + yp) + 0.5f);
                    float w = CostYKfromLab(xp, yp, gcr, gcr1, gammaC, gammaP);
                    depthUp += w * depthn;
                    depthDown += w;
                }
            }
        }
        *get2DBufferAt(dmap, dmap_p, x, y) = ((depth > 0.0f) ? depthUp / depthDown : depth);
    }
}

__global__ void filterDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float minCostThr)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float depth = tex2D<float>(depthsTex, x, y);
        float pixSize = 0.0f;
        {
            float depth = tex2D<float>(depthsTex, x, y);
            int2 pix1 = make_int2(x, y);
            float3 p1 = get3DPointForPixelAndDepthFromRC(pix1, depth);
            int2 pix2 = make_int2(x + 1, y);
            float3 p2 = get3DPointForPixelAndDepthFromRC(pix2, depth);
            pixSize = size(p1 - p2);
        }

        float4 gcr = 255.0f * tex2D<float4>(r4tex, (float)x + 0.5f, (float)y + 0.5f);
        float depthDown = 0.0f;

        for(int yp = -wsh; yp <= wsh; yp++)
        {
            for(int xp = -wsh; xp <= wsh; xp++)
            {
                float depthn = tex2D<float>(depthsTex, x + xp, y + yp);
                if((depth > 0.0f) && (fabs(depthn - depth) < 10.0f * pixSize))
                {
                    float4 gcr1 = 255.0f * tex2D<float4>(r4tex, (float)(x + xp) + 0.5f, (float)(y + yp) + 0.5f);
                    float w = CostYKfromLab(gcr, gcr1, gammaC);

                    depthDown += w;
                }
            }
        }
        *get2DBufferAt(dmap, dmap_p, x, y) = (depthDown < minCostThr) ? -1.0f : depth;
    }
}

__global__ void alignSourceDepthMapToTarget_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    cudaTextureObject_t depthsTex1,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float maxPixelSizeDist)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float sourceDepthMid = tex2D<float>(depthsTex, x, y);
        float pixSize = 0.0f;
        {
            int2 pix1 = make_int2(x, y);
            float3 p1 = get3DPointForPixelAndDepthFromRC(pix1, sourceDepthMid);
            int2 pix2 = make_int2(x + 1, y);
            float3 p2 = get3DPointForPixelAndDepthFromRC(pix2, sourceDepthMid);
            pixSize = size(p1 - p2);
        }

        float maxDepthsDist = maxPixelSizeDist * pixSize;
        float4 gcr = 255.0f * tex2D<float4>(r4tex, (float)x + 0.5f, (float)y + 0.5f);
        float avdist = 0.0f;
        float navdist = 0.0f;

        for(int yp = -wsh; yp <= wsh; yp++)
        {
            for(int xp = -wsh; xp <= wsh; xp++)
            {
                float sourceDepth = tex2D<float>(depthsTex, x + xp, y + yp);
                float targetDepth = tex2D<float>(depthsTex1, x + xp, y + yp);

                if((sourceDepth > 0.0f) && (targetDepth > 0.0f) && (fabs(targetDepth - sourceDepth) < maxDepthsDist))
                {
                    float4 gcr1 = 255.0f * tex2D<float4>(r4tex, (float)(x + xp) + 0.5f, (float)(y + yp) + 0.5f);
                    float deltaC = Euclidean3(gcr, gcr1);
                    if(deltaC < gammaC)
                    {
                        avdist += targetDepth - sourceDepth;
                        navdist += 1.0f;
                    }
                }
            }
        }
        *get2DBufferAt(dmap, dmap_p, x, y) =
            (((navdist == 0.0f) || (sourceDepthMid < 0.0f)) ? sourceDepthMid : (sourceDepthMid + avdist / navdist));
    }
}

__global__ void computeNormalMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float3* nmap, int nmap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float depth = tex2D<float>(depthsTex, x, y);
        int2 pix1 = make_int2(x, y);
        float3 p1 = get3DPointForPixelAndDepthFromRC(pix1, depth);
        float pixSize = 0.0f;
        {
            int2 pix2 = make_int2(x + 1, y);
            float3 p2 = get3DPointForPixelAndDepthFromRC(pix2, depth);
            pixSize = size(p1 - p2);
        }

        float4 gcr = 255.0f * tex2D<float4>(r4tex, (float)x + 0.5f, (float)y + 0.5f);
        cuda_stat3d s3d = cuda_stat3d();

        for(int yp = -wsh; yp <= wsh; yp++)
        {
            for(int xp = -wsh; xp <= wsh; xp++)
            {
                float depthn = tex2D<float>(depthsTex, x + xp, y + yp);
                if((depth > 0.0f) && (fabs(depthn - depth) < 30.0f * pixSize))
                {
                    // float3 gcr1;
                    // gcr1.x = 255.0f*tex2D(rtex0, (float)(x+xp)+0.5f, (float)(y+yp)+0.5f);
                    // gcr1.y = 255.0f*tex2D(rtex1, (float)(x+xp)+0.5f, (float)(y+yp)+0.5f);
                    // gcr1.z = 255.0f*tex2D(rtex2, (float)(x+xp)+0.5f, (float)(y+yp)+0.5f);
                    // float w = CostYKfromLab(xp, yp, gcr, gcr1, gammaC, gammaP);

                    float w = 1.0f;
                    float2 pixn = make_float2(x + xp, y + yp);
                    float3 pn = get3DPointForPixelAndDepthFromRC(pixn, depthn);
                    s3d.update(pn, w);
                }
            }
        }

        float3 pp = p1;
        float3 nn = sg_s_rC - p1;
        normalize(nn);

        float3 nc = nn;
        s3d.computePlaneByPCA(pp, nn);

        if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
        {
            nn.x = -nn.x;
            nn.y = -nn.y;
            nn.z = -nn.z;
        }
        *get2DBufferAt(nmap, nmap_p, x, y) = nn;
    }
}

__global__ void locmin_kernel(
    cudaTextureObject_t sliceTex,
    float* slice, int slice_p, int ndepths, int slicesAtTime,
    int width, int height, int wsh, int t, int npixs,
    int maxDepth,
    bool doUsePixelsDepths, int kernelSizeHalf)
{
    int depthid = blockIdx.x * blockDim.x + threadIdx.x;
    int pixid = blockIdx.y * blockDim.y + threadIdx.y;

    if((depthid < ndepths) && (pixid < slicesAtTime) && (slicesAtTime * t + pixid < npixs))
    {
        float sim0 = tex2D<float>(sliceTex, depthid - 1, pixid);
        float sim1 = tex2D<float>(sliceTex, depthid, pixid);
        float sim2 = tex2D<float>(sliceTex, depthid + 1, pixid);
        float sim = 1.0f;

        if((sim0 > sim1) && (sim1 < sim2))
        {
            sim = sim1;
            if(kernelSizeHalf > 1)
            {
                for(int i = -kernelSizeHalf; i < kernelSizeHalf; i++)
                {
                    float simi = tex2D<float>(sliceTex, depthid + i, pixid);
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

__global__ void getRefTexLAB_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        *get2DBufferAt(texs, texs_p, x, y) = float4_to_uchar4(255.0f * tex2D<float4>(r4tex, (float)x + 0.5f, (float)y + 0.5f));
    }
}

__global__ void getTarTexLAB_kernel(
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p,
    int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        *get2DBufferAt(texs, texs_p, x, y) = float4_to_uchar4(255.0f * tex2D<float4>(t4tex, (float)x + 0.5f, (float)y + 0.5f));
    }
}

__global__ void reprojTarTexLAB_kernel(
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p, int width, int height, float fpPlaneDepth)
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
            *tex = float4_to_uchar4(255.0f * tex2D<float4>(t4tex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
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

#if 0
__global__ void reprojTarTexRgb_kernel(
    cudaTextureObject_t rtex,
    cudaTextureObject_t gtex,
    cudaTextureObject_t btex,
    uchar4* texs, int texs_p,
    int width, int height, float fpPlaneDepth )
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
            tex->x = (unsigned char)(255.0f * tex2D<float>( rtex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            tex->y = (unsigned char)(255.0f * tex2D<float>(gtex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
            tex->z = (unsigned char)(255.0f * tex2D<float>(btex, (float)tpc.x + 0.5f, (float)tpc.y + 0.5f));
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
#endif

#if 0
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
#endif

#if 0
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
#endif

#if 0
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
#endif

#if 0
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
#endif

#if 0
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
#endif

#if 0
__global__ void compWshNccSim_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* osim, int osim_p,
    int width, int height, int wsh, int step )
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
                uchar4 rc = tex2D<uchar4>(rTexU4, xp, yp);
                uchar4 tc = tex2D<uchar4>(tTexU4, xp, yp);

                sst.update((float)rc.x, (float)tc.x, 1.0f);
            }
        }

        sst.computeWSim();
        *get2DBufferAt(osim, osim_p, x, y) = sst.sim;
    }
}
#endif

#if 0
__global__ void aggrYKNCCSim_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    cudaTextureObject_t sliceTex,
    float* osim, int osim_p,
    int width, int height, int wsh, int step,
    const float gammaC, const float gammaP )
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step > wsh) && (y * step > wsh) && (x * step < width - wsh) && (y * step < height - wsh))
    {
        float sums = 0.0f;
        float sumw = 0.0f;

        float4 rcm = uchar4_to_float4(tex2D<uchar4>(rTexU4, x, y));
        float4 tcm = uchar4_to_float4(tex2D<uchar4>(tTexU4, x, y));

        for(int yp = -wsh; yp <= +wsh; yp++)
        {
            for(int xp = -wsh; xp <= +wsh; xp++)
            {
                float4 rc = uchar4_to_float4(tex2D<uchar4>(rTexU4, x * step + xp, y * step + yp));
                float4 tc = uchar4_to_float4(tex2D<uchar4>(tTexU4, x * step + xp, y * step + yp));
                float s = tex2D<float>(sliceTex, x * step + xp, y * step + yp);

                float w =
                    CostYKfromLab(xp, yp, rcm, rc, gammaC, gammaP) * CostYKfromLab(xp, yp, tcm, tc, gammaC, gammaP);

                sums += s * w;
                sumw += w;
            }
        }

        *get2DBufferAt(osim, osim_p, x, y) = sums / sumw;
    }
}
#endif

#if 0
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
#endif

__global__ void downscale_bilateral_smooth_lab_kernel(
    cudaTextureObject_t gaussianTex,
    cudaTextureObject_t r4tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius, float gammaC)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float4 t = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sum = 0.0f;
        float4 center =
            255.0f * tex2D<float4>(r4tex, (float)(x * scale) + (float)scale / 2.0f, (float)(y * scale) + (float)scale / 2.0f);

        for(int i = -radius; i <= radius; i++)
        {
            for(int j = -radius; j <= radius; j++)
            {
                float4 curPix = 255.0f * tex2D<float4>(r4tex, (float)(x * scale + j) + (float)scale / 2.0f,
                                               (float)(y * scale + i) + (float)scale / 2.0f);
                float factor = (tex1D<float>(gaussianTex, i + radius) * tex1D<float>(gaussianTex, j + radius)) // domain factor
                               * CostYKfromLab(center, curPix, gammaC); // range factor
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

__global__ void downscale_gauss_smooth_lab_kernel(
    cudaTextureObject_t gaussianTex,
    cudaTextureObject_t r4tex,
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
                float4 curPix = 255.0f * tex2D<float4>(r4tex, (float)(x * scale + j) + (float)scale / 2.0f,
                                               (float)(y * scale + i) + (float)scale / 2.0f);
                float factor = (tex1D<float>(gaussianTex, i + radius) * tex1D<float>(gaussianTex, j + radius)); // domain factor
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

__global__ void downscale_mean_smooth_lab_kernel(
    cudaTextureObject_t r4tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if((x < width) && (y < height))
    {
        float4 t = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        float sum = 0.0f;
        for(int i = 0; i < scale; i++)
        {
            for(int j = 0; j < scale; j++)
            {
                float4 curPix = 255.0f * tex2D<float4>(r4tex, (float)(x * scale + j) + 0.5f, (float)(y * scale + i) + 0.5f);
                t = t + curPix;
                sum += 1.0f;
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

#if 0
__global__ void ptsStatForRcDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float2* out, int out_p,
    float3* pts, int pts_p,
    int npts, int width, int height,
    int maxNPixSize, int wsh, const float gammaC, const float gammaP)
{
    int ptid = blockIdx.x * blockDim.x + threadIdx.x;

    if(ptid < npts)
    {
        float3 p = pts[ptid];
        float depthP = size(sg_s_rC - p);
        float2 pixf;

        getPixelFor3DPointRC(pixf, p);

        int2 pix = make_int2((int)pixf.x, (int)pixf.y);
        float2 depthVarianceGray = make_float2(0.0f, 0.0f);

        if((pix.x > wsh) && (pix.x < width - wsh) && (pix.y > wsh) && (pix.y < height - wsh))
        {
            float varianceGray = 255.0f * tex2D<float4>(r4tex, (float)pix.x + 0.5f, (float)pix.y + 0.5f).w;
            float depth = tex2D<float>(depthsTex, pix.x, pix.y);
            depthVarianceGray = make_float2(depth, varianceGray);
        }

        // TODO!
        out[ptid] = depthVarianceGray;
    }
}
#endif

__global__ void getSilhoueteMap_kernel(
    cudaTextureObject_t rTexU4,
    bool* out, int out_p,
    int step, int width, int height, const uchar4 maskColorLab )
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step < width) && (y * step < height))
    {
        uchar4 col = tex2D<uchar4>(rTexU4, x * step, y * step);
        *get2DBufferAt(out, out_p, x, y) = ((maskColorLab.x == col.x) && (maskColorLab.y == col.y) && (maskColorLab.z == col.z));
    }
}

#if 0
__global__ void retexture_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p, float4* retexturePixs, int retexturePixs_p, int width,
    int height, int npixs)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height) && (y * width + x < npixs))
    {
        float4 retPixs = *get2DBufferAt(retexturePixs, retexturePixs_p, x, y);
        int2 objPix = make_int2((int)retPixs.x, (int)retPixs.y);
        float2 origPix = make_float2(retPixs.z, retPixs.w);
        float4 colf4 = 255.0f * tex2D<float4>(r4tex, origPix.x, origPix.y);

        *get2DBufferAt(out, out_p, objPix.x, objPix.y) =
            make_uchar4((unsigned char)colf4.x, (unsigned char)colf4.y, (unsigned char)colf4.z, (unsigned char)colf4.w);
    }
}
#endif

#if 0
__global__ void retextureComputeNormalMap_kernel(
    uchar4* out, int out_p,
    float2* retexturePixs, int retexturePixs_p,
    float3* retexturePixsNorms, int retexturePixsNorms_p,
    int width, int height, int npixs)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height) && (y * width + x < npixs))
    {
        float2 objPix = *get2DBufferAt(retexturePixs, retexturePixs_p, x, y);
        float3 objPixNorm = *get2DBufferAt(retexturePixsNorms, retexturePixsNorms_p, x, y);
        *get2DBufferAt(out, out_p, objPix.x, objPix.y) =
            make_uchar4((unsigned char)(objPixNorm.x * 127.0f + 128.0f),
                        (unsigned char)(objPixNorm.y * 127.0f + 128.0f),
                        (unsigned char)(objPixNorm.z * 127.0f + 128.0f), 0);
    }
}
#endif

__global__ void pushPull_Push_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
        float4 colf4res = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
        int nres = 0;

        for(int yp = y * 2; yp <= y * 2 + 1; yp++)
        {
            for(int xp = x * 2; xp <= x * 2 + 1; xp++)
            {
                float4 colf4 = 255.0f * tex2D<float4>(r4tex, (float)xp + 0.5f, (float)yp + 0.5f);
                uchar4 col4 = make_uchar4((unsigned char)colf4.x, (unsigned char)colf4.y, (unsigned char)colf4.z,
                                          (unsigned char)colf4.w);

                if(!((col4.x == 0) && (col4.y == 0) && (col4.z == 0)))
                {
                    colf4res = colf4res + colf4;
                    nres = nres + 1;
                }
            }
        }

        if(nres > 0)
        {
            colf4res.x = colf4res.x / (float)nres;
            colf4res.y = colf4res.y / (float)nres;
            colf4res.z = colf4res.z / (float)nres;
        }

        *get2DBufferAt(out, out_p, x, y) =
            make_uchar4((unsigned char)colf4res.x, (unsigned char)colf4res.y, (unsigned char)colf4res.z, 0.0f);
    }
}

__global__ void pushPull_Pull_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if((x < width) && (y < height))
    {
        uchar4* col4 = get2DBufferAt(out, out_p, x, y);
        if((col4->x == 0) && (col4->y == 0) && (col4->z == 0))
        {
            float4 colf4 = 255.0f * tex2D<float4>(r4tex, ((float)x + 0.5f) / 2.0f, ((float)y + 0.5f) / 2.0f);
            *col4 = make_uchar4((unsigned char)colf4.x, (unsigned char)colf4.y, (unsigned char)colf4.z, 0);
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
