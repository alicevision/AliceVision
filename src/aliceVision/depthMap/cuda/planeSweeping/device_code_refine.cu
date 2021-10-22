// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(int rc_cam_cache_idx,
                                                         int tc_cam_cache_idx,
                                                         cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                         float* osimMap, int osimMap_p,
                                                         float* odptMap, int odptMap_p,
                                                         const float* depthMap, int depthMap_p, int partWidth, int height,
                                                         int wsh, float gammaCInv, float gammaPInv,
                                                         float tcStep,
                                                         bool moveByTcOrRc, int xFrom,
                                                         int rcWidth, int rcHeight,
                                                         int tcWidth, int tcHeight)
{
    const int tile_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int tile_y = blockIdx.y * blockDim.y + threadIdx.y;

    if(tile_x >= partWidth || tile_y >= height)
        return;

    const int2 pix = make_int2(tile_x + xFrom, tile_y);

    float odpt = *get2DBufferAt(depthMap, depthMap_p, tile_x, tile_y);
    float osim = 1.0f;

    float* osim_ptr = get2DBufferAt(osimMap, osimMap_p, tile_x, tile_y);
    float* odpt_ptr = get2DBufferAt(odptMap, odptMap_p, tile_x, tile_y);

    const float4 gcr = tex2D_float4(rc_tex, pix.x + 0.5f, pix.y + 0.5f);
    if(odpt <= 0.0f || gcr.w == 0.0f)
    {
        *osim_ptr = osim;
        *odpt_ptr = odpt;
        return;
    }

    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, pix, odpt);
        move3DPointByTcOrRcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, p, tcStep, moveByTcOrRc);

        odpt = size(p - camsBasesDev[rc_cam_cache_idx].C);

        Patch ptch;
        ptch.p = p;
        ptch.d = computePixSize(rc_cam_cache_idx, p);
        // TODO: we could compute the orientation of the path from the input depth map instead of relying on the cameras orientations
        computeRotCSEpip(rc_cam_cache_idx, tc_cam_cache_idx, ptch);
        osim = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);
    }

    if(tcStep == 0.0f)
    {
        // For the first iteration, we initialize the values
        *osim_ptr = osim;
        *odpt_ptr = odpt;
    }
    else
    {
        // Then we update the similarity value if it's better
        float actsim = *osim_ptr;
        if(osim < actsim)
        {
            *osim_ptr = osim;
            *odpt_ptr = odpt;
        }
    }
}

__global__ void refine_compUpdateYKNCCSimMapPatch_loop_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx,
                                                              cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                              float* osimMap, int osimMap_p, float* odptMap,
                                                              int odptMap_p, float* depthMap, int depthMap_p, int width,
                                                              int height, int wsh, float gammaCInv, float gammaPInv,
                                                              bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight,
                                                              int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((pix.x>wsh)&&(pix.y>wsh)&&(pix.x<width-wsh)&&(pix.y<height-wsh))
    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        const float3 rc_cam_c = camsBasesDev[rc_cam_cache_idx].C;

        for(int i = 0; i < iterations; ++i)
        {
            move3DPointByTcOrRcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            Patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(rc_cam_cache_idx, p);
            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_cam_cache_idx, tc_cam_cache_idx, ptch);
            float osim_upd = compNCCby3DptsYK_WSH<3, 1, 7>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                           rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_cam_c);
            }
        }
    }

    float* osim_ptr = get2DBufferAt(osimMap, osimMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(odptMap, odptMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

__global__ void refine_compUpdateYKNCCSimMapPatch_optimized_fallback_kernel(
    int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex, float* simMap,
    int simMap_p, float* depthMap, int depthMap_p, int width, int height, int wsh, float gammaCInv, float gammaPInv,
    bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight, int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((pix.x>wsh)&&(pix.y>wsh)&&(pix.x<width-wsh)&&(pix.y<height-wsh))
    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        // for(int i = 0; i < 12; ++i)
        //{
        //    rc_P[i] = camsBasesDev[rc_cam_cache_idx].P[i];
        //    tc_P[i] = camsBasesDev[tc_cam_cache_idx].P[i];
        //}
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        for(int i = 0; i < iterations; ++i)
        {
            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            Patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(rc_P, rc_iP, rc_C, p);
            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd = compNCCby3DptsYK(rc_tex, tc_tex, rc_P, tc_P, ptch, wsh, rcWidth, rcHeight, tcWidth,
                                              tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }
    }

    float* osim_ptr = get2DBufferAt(simMap, simMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void refine_compUpdateYKNCCSimMapPatch_optimized_kernel(
    int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex, float* simMap,
    int simMap_p, float* depthMap, int depthMap_p, int width, int height, float gammaCInv, float gammaPInv,
    bool moveByTcOrRc,
    int xFrom, int rcWidth, int rcHeight, int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((pix.x>wsh)&&(pix.y>wsh)&&(pix.x<width-wsh)&&(pix.y<height-wsh))
    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        // for(int i = 0; i < 12; ++i)
        //{
        //    rc_P[i] = camsBasesDev[rc_cam_cache_idx].P[i];
        //    tc_P[i] = camsBasesDev[tc_cam_cache_idx].P[i];
        //}
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        for(int i = 0; i < iterations; ++i)
        {
            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            Patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(rc_P, rc_iP, rc_C, p);
            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd =
                compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, rcWidth, rcHeight,
                                                                           tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }
    }

    float* osim_ptr = get2DBufferAt(simMap, simMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

__global__ void refine_compYKNCCSimMapPatch_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx,
                                                   cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                   float* osimMap, int osimMap_p, float* depthMap, int depthMap_p,
                                                   int partWidth, int height, int wsh, float gammaCInv, float gammaPInv,
                                                   float tcStep,
                                                   bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight, int tcWidth, int tcHeight)
{
    const int tile_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int tile_y = blockIdx.y * blockDim.y + threadIdx.y;

    if(tile_x >= partWidth || tile_y >= height)
        return;

    const int2 pix = make_int2(tile_x + xFrom, tile_y);

    float depth = *get2DBufferAt(depthMap, depthMap_p, tile_x, tile_y);
    float osim = 1.1f;

    if(depth > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, pix, depth);
        // move3DPointByTcPixStep(p, tcStep);
        move3DPointByTcOrRcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, p, tcStep, moveByTcOrRc);

        Patch ptch;
        ptch.p = p;
        ptch.d = computePixSize(rc_cam_cache_idx, p);
        computeRotCSEpip(rc_cam_cache_idx, tc_cam_cache_idx, ptch);
        osim = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight,
                                tcWidth, tcHeight, gammaCInv, gammaPInv);
    }
    *get2DBufferAt(osimMap, osimMap_p, tile_x, tile_y) = osim;
}

__global__ void refine_setLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                  int simMap_p, int width, int height, int id)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    float sim = *get2DBufferAt(simMap, simMap_p, x, y);
    float3* lastThreeSims_ptr = get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, x, y);

    if(id == 0)
    {
        lastThreeSims_ptr->x = sim;
    }
    if(id == 1)
    {
        lastThreeSims_ptr->y = sim;
    }
    if(id == 2)
    {
        lastThreeSims_ptr->z = sim;
    }
}

__global__ void refine_computeDepthSimMapFromLastThreeSimsMap_kernel(int rc_cam_cache_idx,
                                                                     int tc_cam_cache_idx,
                                                                     float* osimMap, int osimMap_p, float* iodepthMap,
                                                                     int iodepthMap_p, float3* lastThreeSimsMap,
                                                                     int lastThreeSimsMap_p, int partWidth, int height,
                                                                     bool moveByTcOrRc, int xFrom)
{
    const int tile_x = blockIdx.x * blockDim.x + threadIdx.x;
    const int tile_y = blockIdx.y * blockDim.y + threadIdx.y;

    if(tile_x >= partWidth || tile_y >= height)
        return;

    const int2 pix = make_int2(tile_x + xFrom, tile_y);

    float midDepth = *get2DBufferAt(iodepthMap, iodepthMap_p, tile_x, tile_y);
    float3 sims = *get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, tile_x, tile_y);
    float outDepth = midDepth;
    float outSim = sims.y;

    if(outDepth > 0.0f)
    {
        float3 pMid = get3DPointForPixelAndDepthFromRC(rc_cam_cache_idx, pix, midDepth);
        float3 pm1 = pMid;
        float3 pp1 = pMid;
        move3DPointByTcOrRcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_cam_cache_idx, tc_cam_cache_idx, pp1, +1.0f, moveByTcOrRc);

        float3 depths;
        depths.x = size(pm1 - camsBasesDev[rc_cam_cache_idx].C);
        depths.y = midDepth;
        depths.z = size(pp1 - camsBasesDev[rc_cam_cache_idx].C);

        outDepth = refineDepthSubPixel(depths, sims);
    }

    *get2DBufferAt(osimMap, osimMap_p, tile_x, tile_y) = outSim;
    *get2DBufferAt(iodepthMap, iodepthMap_p, tile_x, tile_y) = outDepth;
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void refine_merged_optimized_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex,
                                               cudaTextureObject_t tc_tex, float* simMap, int simMap_p, float* depthMap,
                                               int depthMap_p, int width, int height, float gammaCInv, float gammaPInv,
                                               bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight, int tcWidth,
                                               int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        for(int i = 0; i < iterations; ++i)
        {
            Patch ptch;
            ptch.p = p;

            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, ptch.p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            ptch.d = computePixSize(rc_P, rc_iP, rc_C, ptch.p);

            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd =
                compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                              rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }

        //////////

        float3 sims;
        sims.y = osim;

        p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        float3 pm1 = p;
        float3 pp1 = p;
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pp1, +1.0f, moveByTcOrRc);

        Patch ptch;
        ptch.p = pm1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pm1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.x = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        ptch.p = pp1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pp1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.z = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        float3 depths;
        depths.x = size(pm1 - rc_C);
        depths.y = odpt;
        depths.z = size(pp1 - rc_C);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            odpt = refinedDepth;
        }

        //////////
    }

    float* osim_ptr = get2DBufferAt(simMap, simMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void refine_sweep_optimized_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex,
                                              cudaTextureObject_t tc_tex, float* simMap, int simMap_p, float* depthMap,
                                              int depthMap_p, int width, int height, float gammaCInv, float gammaPInv,
                                              bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight, int tcWidth,
                                              int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);

        for(int i = 0; i < iterations; ++i)
        {
            Patch ptch;
            ptch.p = p;

            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, ptch.p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            ptch.d = computePixSize(rc_P, rc_iP, rc_C, ptch.p);

            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd =
                compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, rcWidth, rcHeight,
                                                                           tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }
    }
    if(y * width + x == 2016)
    {
        printf("Kernel sep: %f\n", odpt);
    }
    float* osim_ptr = get2DBufferAt(simMap, simMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void refine_interpolate_optimized_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx,
                                                    cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                    float* simMap, int simMap_p, float* depthMap, int depthMap_p,
                                                    int width, int height, float gammaCInv, float gammaPInv,
                                                    bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight,
                                                    int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 sims;
        sims.y = *get2DBufferAt(simMap, simMap_p, x, y);

        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        float3 pm1 = p;
        float3 pp1 = p;
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pp1, +1.0f, moveByTcOrRc);

        Patch ptch;
        ptch.p = pm1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pm1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.x = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        ptch.p = pp1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pp1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.z = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        float3 depths;
        depths.x = size(pm1 - rc_C);
        depths.y = odpt;
        depths.z = size(pp1 - rc_C);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            odpt = refinedDepth;
        }
    }

    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *odpt_ptr = odpt;
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void refine_sweep_optimized_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex,
                                              cudaTextureObject_t tc_tex, const float2* rc_depth_sim_data,
                                              int rc_depth_sim_data_p, float2* tc_depth_sim_data,
                                              int tc_depth_sim_data_p, int width, int height, float gammaCInv,
                                              float gammaPInv, bool moveByTcOrRc, int rcWidth, int rcHeight, int tcWidth,
                                              int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = get2DBufferAt(rc_depth_sim_data, rc_depth_sim_data_p, x, y)->x;
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = __get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);

        for(int i = 0; i < iterations; ++i)
        {
            Patch ptch;
            ptch.p = p;

            __move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, ptch.p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            ptch.d = __computePixSize(rc_P, rc_iP, rc_C, ptch.p);

            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            __computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd =
                compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                              rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }
    }

    float2* out_data_ptr = get2DBufferAt(tc_depth_sim_data, tc_depth_sim_data_p, x, y);
    *out_data_ptr = make_float2(odpt, osim);
}

template <int WSH, int UNROLL_Y = 1, int UNROLL_X = 2 * WSH + 1>
__global__ void
refine_interpolate_optimized_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx, cudaTextureObject_t rc_tex,
                                    cudaTextureObject_t tc_tex, float2* tc_depth_sim_data, int tc_depth_sim_data_p,
                                    int width, int height, float gammaCInv, float gammaPInv, bool moveByTcOrRc,
                                    int rcWidth, int rcHeight, int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float2 odptsim = *get2DBufferAt(tc_depth_sim_data, tc_depth_sim_data_p, x, y);

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odptsim.x > 0.0f)
    {
        float3 sims;
        sims.y = odptsim.y;

        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odptsim.x);
        float3 pm1 = p;
        float3 pp1 = p;
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pp1, +1.0f, moveByTcOrRc);

        Patch ptch;
        ptch.p = pm1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pm1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.x = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        ptch.p = pp1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pp1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.z = compNCCby3DptsYK_WSH<WSH, UNROLL_Y, UNROLL_X>(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch,
                                                          rcWidth, rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);

        float3 depths;
        depths.x = size(pm1 - rc_C);
        depths.y = odptsim.x;
        depths.z = size(pp1 - rc_C);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            odptsim.x = refinedDepth;
        }
    }

    float2* out_data_ptr = get2DBufferAt(tc_depth_sim_data, tc_depth_sim_data_p, x, y);
    out_data_ptr->x = odptsim.x;
}

__global__ void refine_merged_optimized_fallback_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx,
                                                        cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                        float* simMap, int simMap_p, float* depthMap, int depthMap_p,
                                                        int width, int height, int wsh, float gammaCInv,
                                                        float gammaPInv,
                                                        bool moveByTcOrRc, int xFrom, int rcWidth, int rcHeight,
                                                        int tcWidth, int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        for(int i = 0; i < iterations; ++i)
        {
            Patch ptch;
            ptch.p = p;

            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, ptch.p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            ptch.d = computePixSize(rc_P, rc_iP, rc_C, ptch.p);

            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth,
                                              rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }

        //////////

        float3 sims;
        sims.y = osim;

        p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        float3 pm1 = p;
        float3 pp1 = p;
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pp1, +1.0f, moveByTcOrRc);

        Patch ptch;
        ptch.p = pm1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pm1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.x = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight,
                                  tcWidth, tcHeight, gammaCInv, gammaPInv);

        ptch.p = pp1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pp1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.z = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight,
                                  tcWidth, tcHeight, gammaCInv, gammaPInv);

        float3 depths;
        depths.x = size(pm1 - rc_C);
        depths.y = odpt;
        depths.z = size(pp1 - rc_C);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            odpt = refinedDepth;
        }

        //////////
    }

    float* osim_ptr = get2DBufferAt(simMap, simMap_p, x, y);
    float* odpt_ptr = get2DBufferAt(depthMap, depthMap_p, x, y);
    *osim_ptr = osim;
    *odpt_ptr = odpt;
}

__global__ void refine_merged_optimized_fallback_kernel(int rc_cam_cache_idx, int tc_cam_cache_idx,
                                                        cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                        const float2* rc_depth_sim_data, int rc_depth_sim_data_p,
                                                        float2* tc_depth_sim_data, int tc_depth_sim_data_p, int width,
                                                        int height, int wsh, float gammaCInv, float gammaPInv,
                                                        bool moveByTcOrRc, int rcWidth, int rcHeight, int tcWidth,
                                                        int tcHeight, int iterations)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if(x >= width || y >= height)
        return;

    float odpt = get2DBufferAt(rc_depth_sim_data, rc_depth_sim_data_p, x, y)->x;
    float osim = 1.0f;

    //__shared__ float rc_P[12];
    //__shared__ float tc_P[12];
    const float* rc_P = camsBasesDev[rc_cam_cache_idx].P;
    const float* tc_P = camsBasesDev[tc_cam_cache_idx].P;
    __shared__ float rc_iP[9];
    __shared__ float tc_iP[9];
    __shared__ float3 rc_C;
    __shared__ float3 tc_C;

    if(threadIdx.x == 0 && threadIdx.y == 0)
    {
        for(int i = 0; i < 9; ++i)
        {
            rc_iP[i] = camsBasesDev[rc_cam_cache_idx].iP[i];
            tc_iP[i] = camsBasesDev[tc_cam_cache_idx].iP[i];
        }
        rc_C = camsBasesDev[rc_cam_cache_idx].C;
        tc_C = camsBasesDev[tc_cam_cache_idx].C;
    }
    __syncthreads();

    // If we have an initial depth value, we can refine it
    if(odpt > 0.0f)
    {
        float3 p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        // move3DPointByTcPixStep(p, tcStep);

        for(int i = 0; i < iterations; ++i)
        {
            Patch ptch;
            ptch.p = p;

            move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, ptch.p, (float)(i - (iterations - 1) / 2),
                                       moveByTcOrRc);

            ptch.d = computePixSize(rc_P, rc_iP, rc_C, ptch.p);

            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the
            // cameras orientations
            computeRotCSEpip(rc_C, tc_C, ptch);
            float osim_upd = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth,
                                              rcHeight, tcWidth, tcHeight, gammaCInv, gammaPInv);
            if(osim_upd < osim)
            {
                osim = osim_upd;
                odpt = size(p - rc_C);
            }
        }

        //////////

        float3 sims;
        sims.y = osim;

        p = get3DPointForPixelAndDepthFromRC(rc_iP, rc_C, pix, odpt);
        float3 pm1 = p;
        float3 pp1 = p;
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pm1, -1.0f, moveByTcOrRc);
        move3DPointByTcOrRcPixStep(rc_P, rc_iP, rc_C, tc_P, tc_iP, tc_C, pp1, +1.0f, moveByTcOrRc);

        Patch ptch;
        ptch.p = pm1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pm1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.x = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight,
                                  tcWidth, tcHeight, gammaCInv, gammaPInv);

        ptch.p = pp1;
        ptch.d = computePixSize(rc_P, rc_iP, rc_C, pp1);
        computeRotCSEpip(rc_C, tc_C, ptch);
        sims.z = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight,
                                  tcWidth, tcHeight, gammaCInv, gammaPInv);

        float3 depths;
        depths.x = size(pm1 - rc_C);
        depths.y = odpt;
        depths.z = size(pp1 - rc_C);

        float refinedDepth = refineDepthSubPixel(depths, sims);
        if(refinedDepth > 0.0f)
        {
            odpt = refinedDepth;
        }

        //////////
    }

    float2* out_data_ptr = get2DBufferAt(tc_depth_sim_data, tc_depth_sim_data_p, x, y);
    *out_data_ptr = make_float2(odpt, osim);
}

__device__ __forceinline__ float2 getPixelValueInterpolated(const float2* data, const int data_p, const int data_w,
                                                            const int data_h, const float x, const float y)
{
    // get the image index in the memory

    int xp = static_cast<int>(x);
    int yp = static_cast<int>(y);
    xp = min(xp, data_w - 2);
    yp = min(yp, data_h - 2);

    const float2 lu = *get2DBufferAt(data, data_p, xp, yp);
    const float2 ru = *get2DBufferAt(data, data_p, xp + 1, yp);
    const float2 rd = *get2DBufferAt(data, data_p, xp + 1, yp + 1);
    const float2 ld = *get2DBufferAt(data, data_p, xp, yp + 1);

    // bilinear interpolation
    const float ui = x - static_cast<float>(xp);
    const float vi = y - static_cast<float>(yp);
    const float2 u = lu + (ru - lu) * ui;
    const float2 d = ld + (rd - ld) * ui;
    const float2 out = u + (d - u) * vi;

    return out;
}

__global__ void initFromSmaller_kernel(float2* out, const int out_p, const int out_w, const int out_h, const float2* in,
                                       const int in_p, const int in_w, const int in_h, const float ratio)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x < out_w && y < out_h)
    {
        *get2DBufferAt(out, out_p, x, y) = getPixelValueInterpolated(in, in_p, in_w, in_h, x * ratio, y * ratio);
    }
}

__global__ void setCamSizesInit_kernel(int rc_cam_cache_idx, float2* rc_data, const int rc_data_p, int rc_w, int rc_h,
                                int tc_cam_cache_idx)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if(x < rc_w && y < rc_h)
    {
        float odpt = get2DBufferAt(rc_data, rc_data_p, x, y)->x;

        float3 p = get3DPointForPixelAndDepthFromRC(camsBasesDev[rc_cam_cache_idx].iP, camsBasesDev[rc_cam_cache_idx].C,
                                                    pix, odpt);

        get2DBufferAt(rc_data, rc_data_p, x, y)->y = computePixSize(
            camsBasesDev[tc_cam_cache_idx].P, camsBasesDev[tc_cam_cache_idx].iP, camsBasesDev[tc_cam_cache_idx].C, p);
    }
}

__global__ void setCamSizesMin_kernel(int rc_cam_cache_idx, float2* rc_data, const int rc_data_p, int rc_w, int rc_h,
                               int tc_cam_cache_idx)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x;
    pix.y = y;

    if(x < rc_w && y < rc_h)
    {
        float2 odptsim = *get2DBufferAt(rc_data, rc_data_p, x, y);

        float3 p = get3DPointForPixelAndDepthFromRC(camsBasesDev[rc_cam_cache_idx].iP, camsBasesDev[rc_cam_cache_idx].C,
                                                    pix, odptsim.x);

        get2DBufferAt(rc_data, rc_data_p, x, y)->y =
            fminf(odptsim.y, computePixSize(camsBasesDev[tc_cam_cache_idx].P, camsBasesDev[tc_cam_cache_idx].iP,
                                            camsBasesDev[tc_cam_cache_idx].C, p));
    }
}

} // namespace depthMap
} // namespace aliceVision
