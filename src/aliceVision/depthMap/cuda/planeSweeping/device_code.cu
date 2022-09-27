// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/deviceCommon/device_utils.cuh>

#include <math_constants.h>

namespace aliceVision {
namespace depthMap {

template<typename T>
inline __device__ void swap( T& a, T& b )
{
    T tmp = a;
    a = b;
    b = tmp;
}

__device__ float computeGradientSizeOfL( cudaTextureObject_t rc_tex, int x, int y)
{
    float xM1 = tex2D_float4(rc_tex, (float)(x - 1) + 0.5f, (float)(y + 0) + 0.5f).x;
    float xP1 = tex2D_float4(rc_tex, (float)(x + 1) + 0.5f, (float)(y + 0) + 0.5f).x;
    float yM1 = tex2D_float4(rc_tex, (float)(x + 0) + 0.5f, (float)(y - 1) + 0.5f).x;
    float yP1 = tex2D_float4(rc_tex, (float)(x + 0) + 0.5f, (float)(y + 1) + 0.5f).x;

    // not divided by 2?
    float2 g = make_float2(xM1 - xP1, yM1 - yP1);

    return size(g);
}

__global__ void compute_varLofLABtoW_kernel(cudaTextureObject_t rc_tex, 
                                            float* varianceMap, int varianceMap_p,
                                            int partWidth, int partHeight, int yFrom)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < partWidth && y < partHeight)
    {
        const float grad = computeGradientSizeOfL(rc_tex, x, y + yFrom);
        float* val = get2DBufferAt(varianceMap, varianceMap_p, x, y);
        *val = grad;
    }
}

__device__ void move3DPointByRcPixSize( int cam_cache_idx,
                                        float3& p, float rcPixSize)
{
    float3 rpv = p - camsBasesDev[cam_cache_idx].C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

__device__ void move3DPointByTcPixStep( int rc_cam_cache_idx,
                                        int tc_cam_cache_idx,
                                        float3& p, float tcPixStep)
{
    float3 rpv = camsBasesDev[rc_cam_cache_idx].C - p;
    float3 prp = p;
    float3 prp1 = p + rpv / 2.0f;

    float2 rp;
    getPixelFor3DPoint(rc_cam_cache_idx, rp, prp);

    float2 tpo;
    getPixelFor3DPoint(tc_cam_cache_idx, tpo, prp);

    float2 tpv;
    getPixelFor3DPoint(tc_cam_cache_idx, tpv, prp1);

    tpv = tpv - tpo;
    normalize(tpv);

    float2 tpd = tpo + tpv * tcPixStep;

    p = triangulateMatchRef(rc_cam_cache_idx, tc_cam_cache_idx, rp, tpd);
}

__device__ float move3DPointByTcOrRcPixStep(int rc_cam_cache_idx,
                                            int tc_cam_cache_idx,
                                            float3& p, float pixStep, bool moveByTcOrRc)
{
    if(moveByTcOrRc == true)
    {
        move3DPointByTcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, p, pixStep);
        return 0.0f;
    }
    else
    {
        float pixSize = pixStep * computePixSize(rc_cam_cache_idx, p);
        move3DPointByRcPixSize(rc_cam_cache_idx, p, pixSize);

        return pixSize;
    }
}

__global__ void getSilhoueteMap_kernel(cudaTextureObject_t rc_tex, bool* out, int out_p, int step, int width, int height, const uchar4 maskColorLab)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x * step < width) && (y * step < height))
    {
        uchar4 col = tex2D<uchar4>(rc_tex, x * step, y * step);
        *get2DBufferAt(out, out_p, x, y) = ((maskColorLab.x == col.x) && (maskColorLab.y == col.y) && (maskColorLab.z == col.z));
    }
}


} // namespace depthMap
} // namespace aliceVision
