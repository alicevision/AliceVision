// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/planeSweeping/device_utils.cuh"

#include "aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cuh"

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

// Global data handlers and parameters

#define DCT_DIMENSION 7

#define MAX_CUDA_DEVICES 10

//////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void compute_varLofLABtoW_kernel( cudaTextureObject_t r4tex, uchar4* labMap, int labMap_p, int width, int height, int wsh);

//////////////////////////////////////////////////////////////////////////////////////////////////////

static __device__ void move3DPointByRcPixSize(float3& p, float rcPixSize)
{
    float3 rpv = p - sg_s_rC;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

static __device__ void move3DPointByTcPixStep(float3& p, float tcPixStep)
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

static __device__ float move3DPointByTcOrRcPixStep(float3& p, float pixStep, bool moveByTcOrRc)
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

__global__ void smoothDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP);

__global__ void filterDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float minCostThr );

__global__ void alignSourceDepthMapToTarget_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    cudaTextureObject_t depthsTex1,
    float* dmap, int dmap_p,
    int width, int height, int wsh, const float gammaC, const float maxPixelSizeDist );

__global__ void computeNormalMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float3* nmap, int nmap_p,
    int width, int height, int wsh, const float gammaC, const float gammaP );

__global__ void locmin_kernel(
    cudaTextureObject_t sliceTex,
    float* slice, int slice_p, int ndepths, int slicesAtTime,
    int width, int height, int wsh, int t, int npixs,
    int maxDepth,
    bool doUsePixelsDepths, int kernelSizeHalf );

__global__ void getRefTexLAB_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p, int width, int height);

__global__ void getTarTexLAB_kernel(
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p, int width, int height);

__global__ void reprojTarTexLAB_kernel(
    cudaTextureObject_t t4tex,
    uchar4* texs, int texs_p,
    int width, int height, float fpPlaneDepth);

__global__ void reprojTarTexRgb_kernel(
    cudaTextureObject_t rtex,
    cudaTextureObject_t gtex,
    cudaTextureObject_t btex,
    uchar4* texs, int texs_p,
    int width, int height, float fpPlaneDepth);

__global__ void copyUchar4Dim2uchar_kernel(int dim, uchar4* src, int src_p, unsigned char* tar, int tar_p, int width,
                                           int height);

__global__ void transpose_uchar4_kernel(uchar4* input, int input_p, uchar4* output, int output_p, int width, int height);

__global__ void transpose_float4_kernel(float4* input, int input_p, float4* output, int output_p, int width, int height);

__global__ void compAggrNccSim_kernel(
    float4* ostat1, int ostat1_p,
    float4* ostat2, int ostat2_p,
    uchar4* rImIn, int rImIn_p,
    uchar4* tImIn, int tImIn_p,
    int width, int height, int step, int orintation);

__global__ void compNccSimFromStats_kernel(
    float* odepth, int odepth_p,
    float* osim, int osim_p,
    float4* stat1, int stat1_p,
    float4* stat2, int stat2_p,
    int width, int height, int d, float depth);

__global__ void compWshNccSim_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    float* osim, int osim_p,
    int width, int height, int wsh, int step );

__global__ void aggrYKNCCSim_kernel(
    cudaTextureObject_t rTexU4,
    cudaTextureObject_t tTexU4,
    cudaTextureObject_t sliceTex,
    float* osim, int osim_p,
    int width, int height, int wsh, int step,
    const float gammaC, const float gammaP );

__global__ void updateBestDepth_kernel(
    float* osim, int osim_p,
    float* odpt, int odpt_p,
    float* isim, int isim_p,
    int width, int height, int step, float fpPlaneDepth, int d);

__global__ void downscale_bilateral_smooth_lab_kernel(
    cudaTextureObject_t gaussianTex,
    cudaTextureObject_t r4tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius, float gammaC );

__global__ void downscale_gauss_smooth_lab_kernel(
    cudaTextureObject_t gaussianTex,
    cudaTextureObject_t r4tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale, int radius );

__global__ void downscale_mean_smooth_lab_kernel(
    cudaTextureObject_t r4tex,
    uchar4* texLab, int texLab_p,
    int width, int height, int scale );

__global__ void ptsStatForRcDepthMap_kernel(
    cudaTextureObject_t r4tex,
    cudaTextureObject_t depthsTex,
    float2* out, int out_p,
    float3* pts, int pts_p,
    int npts, int width, int height,
    int maxNPixSize, int wsh, const float gammaC, const float gammaP );

__global__ void getSilhoueteMap_kernel(
    cudaTextureObject_t rTexU4,
    bool* out, int out_p,
    int step, int width, int height, const uchar4 maskColorLab );

__global__ void retexture_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p,
    float4* retexturePixs, int retexturePixs_p,
    int width,
    int height, int npixs);

__global__ void retextureComputeNormalMap_kernel(
    uchar4* out, int out_p,
    float2* retexturePixs, int retexturePixs_p,
    float3* retexturePixsNorms, int retexturePixsNorms_p,
    int width, int height, int npixs);

__global__ void pushPull_Push_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p, int width, int height);

__global__ void pushPull_Pull_kernel(
    cudaTextureObject_t r4tex,
    uchar4* out, int out_p, int width, int height);

} // namespace depthMap
} // namespace aliceVision
