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
                                                         int wsh, float gammaC, float gammaP,
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
        osim = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight, tcWidth, tcHeight, gammaC, gammaP);
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

__global__ void refine_compYKNCCSimMapPatch_kernel(int rc_cam_cache_idx,
                                                   int tc_cam_cache_idx,
                                                   cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                   float* osimMap, int osimMap_p, float* depthMap, int depthMap_p,
                                                   int partWidth, int height, int wsh, float gammaC,
                                                   float gammaP, float tcStep,
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
        osim = compNCCby3DptsYK(rc_tex, tc_tex, rc_cam_cache_idx, tc_cam_cache_idx, ptch, wsh, rcWidth, rcHeight, tcWidth, tcHeight, gammaC, gammaP);
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

} // namespace depthMap
} // namespace aliceVision
