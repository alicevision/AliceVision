// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/ROI.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
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

__device__ float move3DPointByTcOrRcPixStep(int rcDeviceCamId, int tcDeviceCamId, float3& p, float pixStep, bool moveByTcOrRc)
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

__device__ float computeGradientSizeOfL(cudaTextureObject_t rc_tex, int x, int y)
{
    float xM1 = tex2D_float4(rc_tex, (float)(x - 1) + 0.5f, (float)(y + 0) + 0.5f).x;
    float xP1 = tex2D_float4(rc_tex, (float)(x + 1) + 0.5f, (float)(y + 0) + 0.5f).x;
    float yM1 = tex2D_float4(rc_tex, (float)(x + 0) + 0.5f, (float)(y - 1) + 0.5f).x;
    float yP1 = tex2D_float4(rc_tex, (float)(x + 0) + 0.5f, (float)(y + 1) + 0.5f).x;

    // not divided by 2?
    float2 g = make_float2(xM1 - xP1, yM1 - yP1);

    return size(g);
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

__global__ void compute_varLofLABtoW_kernel(cudaTextureObject_t rcTex, 
                                            float* out_varianceMap, int out_varianceMap_p,
                                            const ROI roi)
{
    // roi and varianceMap coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding device image coordinates
    const int x = roi.beginX + roiX;
    const int y = roi.beginY + roiY;

    const float grad = computeGradientSizeOfL(rcTex, x, y);

    // write output
    *get2DBufferAt(out_varianceMap, out_varianceMap_p, roiX, roiY) = grad;
}

/**
 * @param[in] s: iteration over nSamplesHalf
 */
__global__ void fuse_computeGaussianKernelVotingSampleMap_kernel(float* out_gsvSampleMap, int out_gsvSampleMap_p,
                                                                 const float2* in_depthSimMap, int in_depthSimMap_p,
                                                                 const float2* in_midDepthPixSizeMap, int in_midDepthPixSizeMap_p, 
                                                                 int idCam,
                                                                 float sample, 
                                                                 float samplesPerPixSize, 
                                                                 float twoTimesSigmaPowerTwo,
                                                                 const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    const float2 in_midDepthPixSize = *get2DBufferAt(in_midDepthPixSizeMap, in_midDepthPixSizeMap_p, roiX, roiY);
    const float2 in_depthSim = *get2DBufferAt(in_depthSimMap, in_depthSimMap_p, roiX, roiY);
    float* out_gsvSamplePtr = get2DBufferAt(out_gsvSampleMap, out_gsvSampleMap_p, roiX, roiY);
    float gsvSample = (idCam == 0) ? 0.0f : *out_gsvSamplePtr;

    if((in_midDepthPixSize.x > 0.0f) && (in_depthSim.x > 0.0f)) // depth > 0
    {
        const float depthStep = in_midDepthPixSize.y / samplesPerPixSize;
        const float i = (in_midDepthPixSize.x - in_depthSim.x) / depthStep;
        const float sim = -sigmoid(0.0f, 1.0f, 0.7f, -0.7f, in_depthSim.y);
        gsvSample += sim * expf(-((i - sample) * (i - sample)) / twoTimesSigmaPowerTwo);
    }

    *out_gsvSamplePtr = gsvSample;
}


__global__ void fuse_updateBestGaussianKernelVotingSampleMap_kernel(float2* inout_bestGsvSampleMap, int inout_bestGsvSampleMap_p,
                                                                    const float* in_gsvSampleMap, int in_gsvSampleMap_p, 
                                                                    int id, 
                                                                    float sample, 
                                                                    const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    const float in_gsvSampleX = *get2DBufferAt(in_gsvSampleMap, in_gsvSampleMap_p, roiX, roiY);
    float2* inout_bestGsvSamplePtr = get2DBufferAt(inout_bestGsvSampleMap, inout_bestGsvSampleMap_p, roiX, roiY);

    if(id == 0 || in_gsvSampleX < inout_bestGsvSamplePtr->x) 
    {
        *inout_bestGsvSamplePtr = make_float2(in_gsvSampleX, sample);
    }
}

__global__ void fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel(float2* out_depthSimMap, int out_depthSimMap_p, 
                                                                                         const float2* in_bestGsvSampleMap, int in_bestGsvSampleMap_p,
                                                                                         const float2* in_midDepthPixSizeMap, int in_midDepthPixSizeMap_p, 
                                                                                         float samplesPerPixSize,
                                                                                         const ROI roi)
{
    // roi and depth/sim map part coordinates
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    const float2 in_bestGsvSample = *get2DBufferAt(in_bestGsvSampleMap, in_bestGsvSampleMap_p, roiX, roiY);
    const float2 in_midDepthPixSize = *get2DBufferAt(in_midDepthPixSizeMap, in_midDepthPixSizeMap_p, roiX, roiY);
    const float depthStep = in_midDepthPixSize.y / samplesPerPixSize;

    // normalize similarity to -1,0
    // figure; t = -5.0:0.01:0.0; plot(t,sigmoid(0.0,-1.0,6.0,-0.4,t,0));
    // in_bestGsvSample.x = sigmoid(0.0f, -1.0f, 6.0f, -0.4f, in_bestGsvSample.x);
    float2* out_depthSimPtr = get2DBufferAt(out_depthSimMap, out_depthSimMap_p, roiX, roiY);

    if(in_midDepthPixSize.x <= 0.0f)
    {
        *out_depthSimPtr = make_float2(-1.0f, 1.0f);
    }
    else
    {
        *out_depthSimPtr = make_float2(in_midDepthPixSize.x - in_bestGsvSample.y * depthStep, in_bestGsvSample.x);
    }
}

__global__ void fuse_getOptDeptMapFromOptDepthSimMap_kernel(float* out_optDepthMapPart, int out_optDepthMapPart_p,
                                                            const float2* in_optDepthSimMap, int in_optDepthSimMap_p,
                                                            const ROI roi)
{
    // roi and depth/sim map part coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    *get2DBufferAt(out_optDepthMapPart, out_optDepthMapPart_p, roiX, roiY) = get2DBufferAt(in_optDepthSimMap, in_optDepthSimMap_p, roiX, roiY)->x; // depth
}

/**
 * @return (smoothStep, energy)
 */
__device__ float2 getCellSmoothStepEnergy(int rcDeviceCamId, cudaTextureObject_t depthTex, const int2& cell0, const int2& textureOffset)
{
    float2 out = make_float2(0.0f, 180.0f);

    // Get pixel depth from the depth texture
    // Note: we do not use 0.5f offset as we use nearest neighbor interpolation
    float d0 = tex2D<float>(depthTex, float(cell0.x), float(cell0.y));

    // Early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // Consider the neighbor pixels
    const int2 cellL = cell0 + make_int2(0, -1); // Left
    const int2 cellR = cell0 + make_int2(0, 1);	 // Right
    const int2 cellU = cell0 + make_int2(-1, 0); // Up
    const int2 cellB = cell0 + make_int2(1, 0);	 // Bottom

    // Get associated depths from depth texture
    const float dL = tex2D<float>(depthTex, float(cellL.x), float(cellL.y));
    const float dR = tex2D<float>(depthTex, float(cellR.x), float(cellR.y));
    const float dU = tex2D<float>(depthTex, float(cellU.x), float(cellU.y));
    const float dB = tex2D<float>(depthTex, float(cellB.x), float(cellB.y));

    // Get associated 3D points
    const float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, cell0 + textureOffset, d0);
    const float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, cellL + textureOffset, dL);
    const float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, cellR + textureOffset, dR);
    const float3 pU = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, cellU + textureOffset, dU);
    const float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, cellB + textureOffset, dB);

    // Compute the average point based on neighbors (cg)
    float3 cg = make_float3(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dU > 0.0f) { cg = cg + pU; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // If we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        float3 vcn = constantCameraParametersArray_d[rcDeviceCamId].C - p0;
        normalize(vcn);
        // pS: projection of cg on the line from p0 to camera
        const float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x = size(constantCameraParametersArray_d[rcDeviceCamId].C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // Large angle between neighbors == flat area => low energy
        // Small angle between neighbors == non-flat area => high energy
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dU > 0.0f && dB > 0.0f)
    {
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pU, pB)));
        n++;
    }
    // The higher the energy, the less flat the area
    if(n > 0.0f)
        out.y = e;

    return out;
}

__global__ void fuse_optimizeDepthSimMap_kernel(cudaTextureObject_t rc_tex,
                                                int rcDeviceCamId,
                                                cudaTextureObject_t imgVarianceTex,
                                                cudaTextureObject_t depthTex,
                                                float2* out_optDepthSimMap, int out_optDepthSimMap_p,
                                                const float2* in_roughDepthPixSizeMap, int in_roughDepthPixSizeMap_p,
                                                const float2* in_fineDepthSimMap, int in_fineDepthSimMap_p, 
                                                int iter, float samplesPerPixSize,
                                                const ROI roi)
{
    // roi and imgVarianceTex, depthTex coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // SGM upscale depth/pixSize
    const float2 roughDepthPixSize = *get2DBufferAt(in_roughDepthPixSizeMap, in_roughDepthPixSizeMap_p, roiX, roiY);
    const float roughDepth = roughDepthPixSize.x;
    const float roughPixSize = roughDepthPixSize.y;

    // refinedFused depth/sim
    const float2 fineDepthSim = *get2DBufferAt(in_fineDepthSimMap, in_fineDepthSimMap_p, roiX, roiY);
    const float fineDepth = fineDepthSim.x;
    const float fineSim = fineDepthSim.y;

    // output optimized depth/sim
    float2* out_optDepthSimPtr = get2DBufferAt(out_optDepthSimMap, out_optDepthSimMap_p, roiX, roiY);
    float2 out_optDepthSim = (iter == 0) ? make_float2(roughDepth, fineSim) : *out_optDepthSimPtr;
    const float depthOpt = out_optDepthSim.x;

    if (depthOpt > 0.0f)
    {
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamId, depthTex, {roiX, roiY}, {int(roi.beginX), int(roi.beginY)}); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysignf(fminf(fabsf(stepToSmoothDepth), roughPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = fineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysignf(fminf(fabsf(stepToFineDM), roughPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = roughDepth - depthOpt; // distance to smooth/robust input depth map
        const float imgColorVariance = tex2D<float>(imgVarianceTex, float(roiX) + 0.5f, float(roiY) + 0.5f);
        const float colorVarianceThresholdForSmoothing = 20.0f;
        const float angleThresholdForSmoothing = 30.0f; // 30

        // https://www.desmos.com/calculator/kob9lxs9qf
        const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

        // https://www.desmos.com/calculator/jwhpjq6ppj
        const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fineSim);

        // if geometry variation is bigger than color variation => the fineDM is considered noisy

        // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
        // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
        // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
        // https://www.desmos.com/calculator/jzbweilb85
        const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

        // https://www.desmos.com/calculator/ilsk7pthvz
        const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, fabsf(stepToRoughDM / roughPixSize)); // TODO: 10 => 30

        // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

        const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                                   (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                                 (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

        out_optDepthSim.x = depthOpt + depthOptStep;

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * fineSim + (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }

    *out_optDepthSimPtr = out_optDepthSim;
}

} // namespace depthMap
} // namespace aliceVision
