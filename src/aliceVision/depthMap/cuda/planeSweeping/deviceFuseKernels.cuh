// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/ROI.hpp>
#include <aliceVision/depthMap/cuda/device/utils.cuh>
#include <aliceVision/depthMap/cuda/device/matrix.cuh>
#include <aliceVision/depthMap/cuda/device/Patch.cuh>

namespace aliceVision {
namespace depthMap {

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

__global__ void compute_varLofLABtoW_kernel(cudaTextureObject_t rcTex, 
                                            float* out_varianceMap, int out_varianceMap_p,
                                            const ROI roi)
{
    // roi and varianceMap coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding texture coordinates
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

    // corresponding output depth/sim map coordinates
    const int x = roi.beginX + roiX;
    const int y = roi.beginY + roiY;

    const float2 in_bestGsvSample = *get2DBufferAt(in_bestGsvSampleMap, in_bestGsvSampleMap_p, roiX, roiY);
    const float2 in_midDepthPixSize = *get2DBufferAt(in_midDepthPixSizeMap, in_midDepthPixSizeMap_p, roiX, roiY);
    const float depthStep = in_midDepthPixSize.y / samplesPerPixSize;

    // normalize similarity to -1,0
    // figure; t = -5.0:0.01:0.0; plot(t,sigmoid(0.0,-1.0,6.0,-0.4,t,0));
    // in_bestGsvSample.x = sigmoid(0.0f, -1.0f, 6.0f, -0.4f, in_bestGsvSample.x);
    float2* out_depthSimPtr = get2DBufferAt(out_depthSimMap, out_depthSimMap_p, x, y);

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

    // corresponding input depth/sim map coordinates
    const int x = roi.beginX + roiX;
    const int y = roi.beginY + roiY;

    *get2DBufferAt(out_optDepthMapPart, out_optDepthMapPart_p, roiX, roiY) = get2DBufferAt(in_optDepthSimMap, in_optDepthSimMap_p, x, y)->x; // depth
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

    // corresponding image coordinates
    const int x = roi.beginX + roiX;
    const int y = roi.beginY + roiY;

    // SGM upscale depth/pixSize
    const float2 roughDepthPixSize = *get2DBufferAt(in_roughDepthPixSizeMap, in_roughDepthPixSizeMap_p, x, y);
    const float roughDepth = roughDepthPixSize.x;
    const float roughPixSize = roughDepthPixSize.y;

    // refinedFused depth/sim
    const float2 fineDepthSim = *get2DBufferAt(in_fineDepthSimMap, in_fineDepthSimMap_p, x, y);
    const float fineDepth = fineDepthSim.x;
    const float fineSim = fineDepthSim.y;

    // output optimized depth/sim
    float2* out_optDepthSimPtr = get2DBufferAt(out_optDepthSimMap, out_optDepthSimMap_p, x, y);
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

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * fineSim +
            (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }

    *out_optDepthSimPtr = out_optDepthSim;
}

} // namespace depthMap
} // namespace aliceVision
