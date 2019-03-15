// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

namespace aliceVision {
namespace depthMap {

__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                         float* osimMap, int osimMap_p, float* odptMap, int odptMap_p,
                                                         float* depthMap, int depthMap_p, int width, int height,
                                                         int wsh, const float gammaC, const float gammaP,
                                                         const float epipShift, const float tcStep, int id,
                                                         bool moveByTcOrRc, int xFrom, int imWidth, int imHeight)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((pix.x>wsh)&&(pix.y>wsh)&&(pix.x<width-wsh)&&(pix.y<height-wsh))
    if((x >= 0) && (y >= 0) && (x < width) && (y < height))
    {
        float odpt = *get2DBufferAt(depthMap, depthMap_p, x, y);
        float osim = 1.0f;

        // If we have an initial depth value, we can refine it
        if(odpt > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, odpt);
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);

            odpt = size(p - sg_s_r.C);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            // TODO: we could compute the orientation of the path from the input depth map instead of relying on the cameras orientations
            computeRotCSEpip(ptch, p);
            osim = compNCCby3DptsYK(rc_tex, tc_tex, ptch, wsh, imWidth, imHeight, gammaC, gammaP, epipShift);
        }

        float* osim_ptr = get2DBufferAt(osimMap, osimMap_p, x, y);
        float* odpt_ptr = get2DBufferAt(odptMap, odptMap_p, x, y);
        if(id == 0)
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
}

__global__ void refine_compYKNCCSimMapPatch_kernel(cudaTextureObject_t rc_tex, cudaTextureObject_t tc_tex,
                                                   float* osimMap, int osimMap_p, float* depthMap, int depthMap_p,
                                                   int width, int height, int wsh, const float gammaC,
                                                   const float gammaP, const float epipShift, const float tcStep,
                                                   bool moveByTcOrRc, int xFrom, int imWidth, int imHeight)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    // if ((x>wsh)&&(y>wsh)&&(x<width-wsh)&&(y<height-wsh))
    if((x >= 0) && (y >= 0) && (x < width) && (y < height))
    {
        float depth = *get2DBufferAt(depthMap, depthMap_p, x, y);
        float osim = 1.1f;

        if(depth > 0.0f)
        {
            float3 p = get3DPointForPixelAndDepthFromRC(pix, depth);
            // move3DPointByTcPixStep(p, tcStep);
            move3DPointByTcOrRcPixStep(pix, p, tcStep, moveByTcOrRc);

            patch ptch;
            ptch.p = p;
            ptch.d = computePixSize(p);
            computeRotCSEpip(ptch, p);
            osim = compNCCby3DptsYK(rc_tex, tc_tex, ptch, wsh, imWidth, imHeight, gammaC, gammaP, epipShift);
        };
        *get2DBufferAt(osimMap, osimMap_p, x, y) = osim;
    };
}

__global__ void refine_setLastThreeSimsMap_kernel(float3* lastThreeSimsMap, int lastThreeSimsMap_p, float* simMap,
                                                  int simMap_p, int width, int height, int id)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if((x < width) && (y < height))
    {
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
}

__global__ void refine_computeDepthSimMapFromLastThreeSimsMap_kernel(float* osimMap, int osimMap_p, float* iodepthMap,
                                                                     int iodepthMap_p, float3* lastThreeSimsMap,
                                                                     int lastThreeSimsMap_p, int width, int height,
                                                                     bool moveByTcOrRc, int xFrom)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int2 pix;
    pix.x = x + xFrom;
    pix.y = y;

    if((x < width) && (y < height))
    {
        float midDepth = *get2DBufferAt(iodepthMap, iodepthMap_p, x, y);
        float3 sims = *get2DBufferAt(lastThreeSimsMap, lastThreeSimsMap_p, x, y);
        float outDepth = midDepth;
        float outSim = sims.y;

        if(outDepth > 0.0f)
        {
            float3 pMid = get3DPointForPixelAndDepthFromRC(pix, midDepth);
            float3 pm1 = pMid;
            float3 pp1 = pMid;
            move3DPointByTcOrRcPixStep(pix, pm1, -1.0f, moveByTcOrRc);
            move3DPointByTcOrRcPixStep(pix, pp1, +1.0f, moveByTcOrRc);

            float3 depths;
            depths.x = size(pm1 - sg_s_r.C);
            depths.y = midDepth;
            depths.z = size(pp1 - sg_s_r.C);

            float refinedDepth = refineDepthSubPixel(depths, sims);
            if(refinedDepth > 0.0f)
            {
                outDepth = refinedDepth;
            };
        };

        *get2DBufferAt(osimMap, osimMap_p, x, y) = outSim;
        *get2DBufferAt(iodepthMap, iodepthMap_p, x, y) = outDepth;
    };
}

} // namespace depthMap
} // namespace aliceVision
