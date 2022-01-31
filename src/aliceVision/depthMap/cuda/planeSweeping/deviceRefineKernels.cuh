// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/device/matrix.cuh>
#include <aliceVision/depthMap/cuda/device/Patch.cuh>

namespace aliceVision {
namespace depthMap {

__device__ void move3DPointByRcPixSize(int deviceCamId, float3& p, float rcPixSize)
{
    float3 rpv = p - constantCameraParametersArray_d[deviceCamId].C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

__device__ void move3DPointByTcPixStep(int rcDeviceCamId, int tcDeviceCamId, float3& p, float tcPixStep)
{
    float3 rpv = constantCameraParametersArray_d[rcDeviceCamId].C - p;
    float3 prp = p;
    float3 prp1 = p + rpv / 2.0f;

    float2 rp;
    getPixelFor3DPoint(rcDeviceCamId, rp, prp);

    float2 tpo;
    getPixelFor3DPoint(tcDeviceCamId, tpo, prp);

    float2 tpv;
    getPixelFor3DPoint(tcDeviceCamId, tpv, prp1);

    tpv = tpv - tpo;
    normalize(tpv);

    float2 tpd = tpo + tpv * tcPixStep;

    p = triangulateMatchRef(rcDeviceCamId, tcDeviceCamId, rp, tpd);
}

__device__ float move3DPointByTcOrRcPixStep(int rcDeviceCamId, int tcDeviceCamId, float3& p, float pixStep,
                                            bool moveByTcOrRc)
{
    if(moveByTcOrRc == true)
    {
        move3DPointByTcPixStep(rcDeviceCamId, tcDeviceCamId, p, pixStep);
        return 0.0f;
    }
    else
    {
        float pixSize = pixStep * computePixSize(rcDeviceCamId, p);
        move3DPointByRcPixSize(rcDeviceCamId, p, pixSize);

        return pixSize;
    }
}

__global__ void refine_compUpdateYKNCCSimMapPatch_kernel(int rcDeviceCamId,
                                                         int tcDeviceCamId,
                                                         cudaTextureObject_t rcTex, 
                                                         cudaTextureObject_t tcTex,
                                                         const float2* in_sgmDepthSimMap, int in_sgmDepthSimMap_p, 
                                                         float2* out_bestDepthSimMap, int out_bestDepthSimMap_p, 
                                                         int wsh, 
                                                         float gammaC, 
                                                         float gammaP, 
                                                         bool moveByTcOrRc,
                                                         int rcWidth, int rcHeight,
                                                         int tcWidth, int tcHeight,
                                                         int tcCurrentStep,
                                                         int tcFirstStep,
                                                         const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding texture coordinates
    const int2 pix = make_int2(roi.beginX + roiX, roi.beginY + roiY);

    // get the output best depth/sim pointer
    float2* out_bestDepthSimPtr = get2DBufferAt(out_bestDepthSimMap, out_bestDepthSimMap_p, roiX, roiY);

    // initialize depth/sim from upscaled SGM value
    float2 depthSim = *get2DBufferAt(in_sgmDepthSimMap, in_sgmDepthSimMap_p, roiX, roiY);
    depthSim.y = 1.0f;

    // get the corresponding color (to check alpha)
    const float4 gcr = tex2D_float4(rcTex, pix.x + 0.5f, pix.y + 0.5f);

    // case SGM depth <= 0 or Rc alpha == 0
    if(depthSim.x <= 0.0f || gcr.w == 0.0f) 
    {
        *out_bestDepthSimPtr = depthSim;
        return;
    }

    // get a 3d point from SGM result depth (middle depth)
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, pix, depthSim.x);

    // move the 3d point in depth by tcStep
    move3DPointByTcOrRcPixStep(rcDeviceCamId, tcDeviceCamId, p, float(tcCurrentStep), moveByTcOrRc);

    // get the new depth from the point move
    depthSim.x = size(p - constantCameraParametersArray_d[rcDeviceCamId].C);

    // compute similarity at the new depth
    {
        Patch ptch;
        ptch.p = p;
        ptch.d = computePixSize(rcDeviceCamId, p);
        // TODO: we could compute the orientation of the path from the input depth map instead 
        // of relying on the cameras orientations
        computeRotCSEpip(rcDeviceCamId, tcDeviceCamId, ptch);
        depthSim.y = compNCCby3DptsYK(rcTex, tcTex, rcDeviceCamId, tcDeviceCamId, ptch, rcWidth, rcHeight, tcWidth, tcHeight, wsh, gammaC, gammaP);
    }

    // update output best depth/sim map 
    if(tcCurrentStep == tcFirstStep)
    {
        // for the first iteration, we initialize the values
        *out_bestDepthSimPtr = depthSim;
    }
    else
    {
        // update the similarity value if it's better
        const float actSim = (*out_bestDepthSimPtr).y;
        if(depthSim.y < actSim)
        {
            *out_bestDepthSimPtr = depthSim;
        }
    }
}

__global__ void refine_compYKNCCSimMapPatch_kernel(int rcDeviceCamId,
                                                   int tcDeviceCamId,
                                                   cudaTextureObject_t rcTex, 
                                                   cudaTextureObject_t tcTex,
                                                   float2* inout_bestDepthSimMap, int inout_bestDepthSimMap_p, 
                                                   int wsh, 
                                                   float gammaC, 
                                                   float gammaP, 
                                                   bool moveByTcOrRc,
                                                   int rcWidth, int rcHeight,
                                                   int tcWidth, int tcHeight,
                                                   float tcStep,
                                                   const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding texture coordinates
    const int2 pix = make_int2(roi.beginX + roiX, roi.beginY + roiY);

    // get best depth/sim value 
    float2 depthSim = *get2DBufferAt(inout_bestDepthSimMap, inout_bestDepthSimMap_p, roiX, roiY);
  
    if(depthSim.x > 0.0f) 
    {
        // depth > 0
        // get a 3d point from best depth (middle depth)
        float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, pix, depthSim.x);

        // move the 3d point in depth by tcStep
        move3DPointByTcOrRcPixStep(rcDeviceCamId, tcDeviceCamId, p, tcStep, moveByTcOrRc);

        // compute similarity at the new depth
        Patch ptch;
        ptch.p = p;
        ptch.d = computePixSize(rcDeviceCamId, p);
        computeRotCSEpip(rcDeviceCamId, tcDeviceCamId, ptch);
        depthSim.y  = compNCCby3DptsYK(rcTex, tcTex, rcDeviceCamId, tcDeviceCamId, ptch, rcWidth, rcHeight, tcWidth, tcHeight, wsh, gammaC, gammaP);
    }
    else 
    {
        // depth <= 0, set bad similarity
        depthSim.y = 1.1f;
    }

    // update output similarity, best depth is unchanged
    *get2DBufferAt(inout_bestDepthSimMap, inout_bestDepthSimMap_p, roiX, roiY) = depthSim;
}

__global__ void refine_setLastThreeSimsMap_kernel(float3* out_lastThreeSimsMap, int out_lastThreeSimsMap_p, 
                                                  const float2* in_depthSimMap, int in_depthSimMap_p, 
                                                  int index, 
                                                  const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    const float2 in_depthSim = *get2DBufferAt(in_depthSimMap, in_depthSimMap_p, roiX, roiY); 
    float3* out_lastThreeSimsPtr = get2DBufferAt(out_lastThreeSimsMap, out_lastThreeSimsMap_p, roiX, roiY);

    if(index == 0)
    {
        out_lastThreeSimsPtr->x = in_depthSim.y; // in_depthSim.y is similarity
    }
    else if(index == 1)
    {
        out_lastThreeSimsPtr->y = in_depthSim.y;
    }
    else if(index == 2)
    {
        out_lastThreeSimsPtr->z = in_depthSim.y;
    }
}

__global__ void refine_interpolateDepthFromThreeSimsMap_kernel(int rcDeviceCamId,
                                                               int tcDeviceCamId,
                                                               const float3* in_lastThreeSimsMap, int in_lastThreeSimsMap_p,
                                                               float2* inout_bestDepthSimMap, int inout_bestDepthSimMap_p, 
                                                               bool moveByTcOrRc,
                                                               const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding texture coordinates
    const int2 pix = make_int2(roi.beginX + roiX, roi.beginY + roiY);

    // get best middle depth
    const float midDepth = get2DBufferAt(inout_bestDepthSimMap, inout_bestDepthSimMap_p, roiX, roiY)->x;

    // get best similarity values
    const float3 sims = *get2DBufferAt(in_lastThreeSimsMap, in_lastThreeSimsMap_p, roiX, roiY);

    float2 out_depthSim = make_float2(midDepth, sims.y);

    if(midDepth > 0.0f) // middle depth > 0
    {
        // get the three 3d points
        const float3 pMid = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, pix, midDepth);
        float3 pm1 = pMid;
        float3 pp1 = pMid;

        move3DPointByTcOrRcPixStep(rcDeviceCamId, tcDeviceCamId, pm1, -1.0f, moveByTcOrRc); // move the 3d point in depth by -1
        move3DPointByTcOrRcPixStep(rcDeviceCamId, tcDeviceCamId, pp1, +1.0f, moveByTcOrRc); // move the 3d point in depth by +1

        float3 depths;
       
        depths.x = size(pm1 - constantCameraParametersArray_d[rcDeviceCamId].C); // get the new depth from the point move
        depths.y = midDepth; // middle depth
        depths.z = size(pp1 - constantCameraParametersArray_d[rcDeviceCamId].C); // get the new depth from the point move

        // interpolate depth
        out_depthSim.x = refineDepthSubPixel(depths, sims);
    }

    *get2DBufferAt(inout_bestDepthSimMap, inout_bestDepthSimMap_p, roiX, roiY) = out_depthSim;
}

} // namespace depthMap
} // namespace aliceVision
