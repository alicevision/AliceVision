// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap/cuda/device/buffer.cuh>
#include <aliceVision/depthMap/cuda/device/matrix.cuh>
#include <aliceVision/depthMap/cuda/device/Patch.cuh>
#include <aliceVision/depthMap/cuda/device/eig33.cuh>
#include <aliceVision/depthMap/cuda/device/DeviceCameraParams.hpp>

#define ALICEVISION_DEPTHMAP_UPSCALE_NEAREST_NEIGHBOR

namespace aliceVision {
namespace depthMap {

/**
 * @return (smoothStep, energy)
 */
__device__ float2 getCellSmoothStepEnergy(int rcDeviceCamId, cudaTextureObject_t depthTex, const int2& cell0, const int2& textureOffset)
{
    float2 out = make_float2(0.0f, 180.0f);

    // Get pixel depth from the depth texture
    // Note: we do not use 0.5f offset because depthTex use nearest neighbor interpolation
    const float d0 = tex2D<float>(depthTex, float(cell0.x), float(cell0.y));

    // Early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // Consider the neighbor pixels
    const int2 cellL = cell0 + make_int2( 0, -1); // Left
    const int2 cellR = cell0 + make_int2( 0,  1);	// Right
    const int2 cellU = cell0 + make_int2(-1,  0); // Up
    const int2 cellB = cell0 + make_int2( 1,  0);	// Bottom

    // Get associated depths from depth texture
    // Note: we do not use 0.5f offset because depthTex use nearest neighbor interpolation
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

__device__ static inline float orientedPointPlaneDistanceNormalizedNormal(const float3& point,
                                                                          const float3& planePoint,
                                                                          const float3& planeNormalNormalized)
{
    return (dot(point, planeNormalNormalized) - dot(planePoint, planeNormalNormalized));
}

__global__ void depthSimMapCopyDepthOnly_kernel(float2* out_deptSimMap_d, int out_deptSimMap_p,
                                                const float2* in_depthSimMap_d, int in_depthSimMap_p,
                                                int width, int height, 
                                                float defaultSim)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= width || y >= height)
        return;

    // write output
    float2* out_depthSim = get2DBufferAt(out_deptSimMap_d, out_deptSimMap_p, x, y);
    out_depthSim->x = get2DBufferAt(in_depthSimMap_d, in_depthSimMap_p, x, y)->x;
    out_depthSim->y = defaultSim;
}

template<class T>
__global__ void mapUpscale_kernel(T* out_upscaledMap_d, int out_upscaledMap_p,
                                  const T* in_map_d, int in_map_p, 
                                  const ROI roi,
                                  float ratio)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= roi.width() || y >= roi.height())
        return;

    const float ox = (float(x) - 0.5f) * ratio;
    const float oy = (float(y) - 0.5f) * ratio;

    // nearest neighbor, no interpolation
    const int xp = min(int(floor(ox + 0.5)), int(roi.width()  * ratio) - 1);
    const int yp = min(int(floor(oy + 0.5)), int(roi.height() * ratio) - 1);

    // write output upscaled map
    *get2DBufferAt(out_upscaledMap_d, out_upscaledMap_p, x, y) = *get2DBufferAt(in_map_d, in_map_p, xp, yp);
}


__global__ void depthSimMapUpscaleAndFilter_kernel(cudaTextureObject_t rcTex,
                                                   float2* out_upscaledDeptSimMap_d, int out_upscaledDeptSimMap_p,
                                                   const float2* in_otherDepthSimMap_d, int in_otherDepthSimMap_p,
                                                   const ROI roi,
                                                   float ratio)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;

    if(x >= roi.width() || y >= roi.height())
        return;

    // filter masked pixels (alpha < 0.9f)
    if(tex2D_float4(rcTex, x + 0.5f, y + 0.5f).w < 0.9f)
    {
        *get2DBufferAt(out_upscaledDeptSimMap_d, out_upscaledDeptSimMap_p, x, y) = make_float2(-2.f, 1.f);
        return;
    }

    const float oy = (float(y) - 0.5f) * ratio;
    const float ox = (float(x) - 0.5f) * ratio;

    float2 out_depthSim;

#ifdef ALICEVISION_DEPTHMAP_UPSCALE_NEAREST_NEIGHBOR
    // nearest neighbor, no interpolation
    int xp = floor(ox + 0.5);
    int yp = floor(oy + 0.5);

    xp = min(xp, int(roi.width()  * ratio) - 1);
    yp = min(yp, int(roi.height() * ratio) - 1);

    out_depthSim = *get2DBufferAt(in_otherDepthSimMap_d, in_otherDepthSimMap_p, xp, yp);
#else
    // interpolate using the distance to the pixels center
    int xp = floor(ox);
    int yp = floor(oy);

    xp = min(xp, in_width  - 2);
    yp = min(yp, in_height - 2);

    const float2 lu = *get2DBufferAt(in_otherDepthSimMap_d, in_otherDepthSimMap_p, xp, yp);
    const float2 ru = *get2DBufferAt(in_otherDepthSimMap_d, in_otherDepthSimMap_p, xp + 1, yp);
    const float2 rd = *get2DBufferAt(in_otherDepthSimMap_d, in_otherDepthSimMap_p, xp + 1, yp + 1);
    const float2 ld = *get2DBufferAt(in_otherDepthSimMap_d, in_otherDepthSimMap_p, xp, yp + 1);

    if(lu.x <= 0.0f || ru.x <= 0.0f || rd.x <= 0.0f || ld.x <= 0.0f)
    {
        float2 acc = {0.0f, 0.0f};
        int count = 0;

        if(lu.x > 0.0f)
        {
            acc = acc + lu;
            ++count;
        }
        if(ru.x > 0.0f)
        {
            acc = acc + ru;
            ++count;
        }
        if(rd.x > 0.0f)
        {
            acc = acc + rd;
            ++count;
        }
        if(ld.x > 0.0f)
        {
            acc = acc + ld;
            ++count;
        }
        if(count != 0)
        {
            out_depthSim = {acc.x / float(count), acc.y / float(count)};
            return;
        }
        else
        {
            out_depthSim = {-1.0f, 1.0f};
            return;
        }
    }

    // bilinear interpolation
    const float ui = x - float(xp);
    const float vi = y - float(yp);
    const float2 u = lu + (ru - lu) * ui;
    const float2 d = ld + (rd - ld) * ui;
    out_depthSim = u + (d - u) * vi;
#endif

    // write output
    *get2DBufferAt(out_upscaledDeptSimMap_d, out_upscaledDeptSimMap_p, x, y) = out_depthSim;
}

__global__ void depthSimMapComputePixSize_kernel(int rcDeviceCamId, float2* inout_deptPixSizeMap_d, int inout_deptPixSizeMap_p, int stepXY, const ROI roi)
{
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height()) 
        return;

    // corresponding device image coordinates
    const int x = (roi.x.begin + roiX) * stepXY;
    const int y = (roi.y.begin + roiY) * stepXY;

    // corresponding input/output depthSim
    float2* inout_depthPixSize = get2DBufferAt(inout_deptPixSizeMap_d, inout_deptPixSizeMap_p, roiX, roiY);

    // original depth invalid or masked, pixSize set to 0
    if(inout_depthPixSize->x < 0.0f) 
    {
        inout_depthPixSize->y = 0;
        return; 
    }

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, make_int2(x, y), inout_depthPixSize->x);

    inout_depthPixSize->y = computePixSize(rcDeviceCamId, p);
}

__global__ void depthSimMapComputeNormal_kernel(int rcDeviceCamId,
                                                float3* out_normalMap_d, int out_normalMap_p,
                                                const float2* in_depthSimMap_d, int in_depthSimMap_p,
                                                int wsh,
                                                int gammaC,
                                                int gammaP, 
                                                int stepXY,
                                                const ROI roi)
{
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding image coordinates
    const int x = (roi.x.begin + roiX) * stepXY;
    const int y = (roi.y.begin + roiY) * stepXY;

    // corresponding input depth
    const float in_depth = get2DBufferAt(in_depthSimMap_d, in_depthSimMap_p, roiX, roiY)->x; // use only depth

    // corresponding output normal
    float3* out_normal = get2DBufferAt(out_normalMap_d, out_normalMap_p, roiX, roiY);

    // no depth
    if(in_depth <= 0.0f)
    {
        *out_normal = make_float3(-1.f, -1.f, -1.f);
        return;
    }

    const int2 pix = make_int2(x, y);
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, pix, in_depth);
    const float pixSize = size(p - get3DPointForPixelAndDepthFromRC(rcDeviceCamId, make_int2(x + 1, y), in_depth));

    cuda_stat3d s3d = cuda_stat3d();

    for(int yp = -wsh; yp <= wsh; ++yp)
    {
        const int roiYp = roiY + yp;
        if(roiYp < 0)
            continue;

        for(int xp = -wsh; xp <= wsh; ++xp)
        {
            const int roiXp = roiX + xp;
            if(roiXp < 0)
                continue;

            const float depthP = get2DBufferAt(in_depthSimMap_d, in_depthSimMap_p, roiXp, roiYp)->x;  // use only depth

            if((depthP > 0.0f) && (fabs(depthP - in_depth) < 30.0f * pixSize))
            {
                const float w = 1.0f;
                const float2 pixP = make_float2(x + xp, y + yp);
                const float3 pP = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, pixP, depthP);
                s3d.update(pP, w);
            }
        }
    }

    float3 pp = p;
    float3 nn = make_float3(-1.f, -1.f, -1.f);

    if(!s3d.computePlaneByPCA(pp, nn))
    {
        *out_normal = make_float3(-1.f, -1.f, -1.f);
        return;
    }

    float3 nc = constantCameraParametersArray_d[rcDeviceCamId].C - p;
    normalize(nc);

    if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
    {
        nn.x = -nn.x;
        nn.y = -nn.y;
        nn.z = -nn.z;
    }

    *out_normal = nn;
}

__global__ void optimize_varLofLABtoW_kernel(cudaTextureObject_t rcTex, float* out_varianceMap_d, int out_varianceMap_p, const ROI roi)
{
    // roi and varianceMap coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding device image coordinates
    const int x = roi.x.begin + roiX;
    const int y = roi.y.begin + roiY;

    // compute gradient size of L
    // note: we use 0.5f offset because rcTex texture use interpolation
    const float xM1 = tex2D_float4(rcTex, float(x - 1) + 0.5f, float(y + 0) + 0.5f).x;
    const float xP1 = tex2D_float4(rcTex, float(x + 1) + 0.5f, float(y + 0) + 0.5f).x;
    const float yM1 = tex2D_float4(rcTex, float(x + 0) + 0.5f, float(y - 1) + 0.5f).x;
    const float yP1 = tex2D_float4(rcTex, float(x + 0) + 0.5f, float(y + 1) + 0.5f).x;
    const float2 g = make_float2(xM1 - xP1, yM1 - yP1); // TODO: not divided by 2?
    const float grad = size(g);

    // write output
    *get2DBufferAt(out_varianceMap_d, out_varianceMap_p, roiX, roiY) = grad;
}

__global__ void optimize_getOptDeptMapFromOptDepthSimMap_kernel(float* out_tmpOptDepthMap_d, int out_tmpOptDepthMap_p,
                                                                const float2* in_optDepthSimMap_d, int in_optDepthSimMap_p,
                                                                const ROI roi)
{
    // roi and depth/sim map part coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    *get2DBufferAt(out_tmpOptDepthMap_d, out_tmpOptDepthMap_p, roiX, roiY) = get2DBufferAt(in_optDepthSimMap_d, in_optDepthSimMap_p, roiX, roiY)->x; // depth
}

__global__ void optimize_depthSimMap_kernel(int rcDeviceCamId,
                                            cudaTextureObject_t imgVarianceTex,
                                            cudaTextureObject_t depthTex,
                                            float2* out_optimizeDepthSimMap_d, int out_optimizeDepthSimMap_p,    // output optimized depth/sim map
                                            const float2* in_sgmDepthPixSizeMap_d, int in_sgmDepthPixSizeMap_p,  // input upscaled rough depth/pixSize map
                                            const float2* in_refineDepthSimMap_d, int in_refineDepthSimMap_p,    // input fine depth/sim map
                                            int iter,
                                            const ROI roi)
{
    // roi and imgVarianceTex, depthTex coordinates 
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // SGM upscale (rough) depth/pixSize
    const float2 sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, in_sgmDepthPixSizeMap_p, roiX, roiY);
    const float sgmDepth = sgmDepthPixSize.x;
    const float sgmPixSize = sgmDepthPixSize.y;

    // refined and fused (fine) depth/sim
    const float2 refineDepthSim = *get2DBufferAt(in_refineDepthSimMap_d, in_refineDepthSimMap_p, roiX, roiY);
    const float refineDepth = refineDepthSim.x;
    const float refineSim = refineDepthSim.y;

    // output optimized depth/sim
    float2* out_optDepthSimPtr = get2DBufferAt(out_optimizeDepthSimMap_d, out_optimizeDepthSimMap_p, roiX, roiY);
    float2 out_optDepthSim = (iter == 0) ? make_float2(sgmDepth, refineSim) : *out_optDepthSimPtr;
    const float depthOpt = out_optDepthSim.x;

    if (depthOpt > 0.0f)
    {
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamId, depthTex, {roiX, roiY}, {int(roi.x.begin), int(roi.y.begin)}); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysignf(fminf(fabsf(stepToSmoothDepth), sgmPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = refineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysignf(fminf(fabsf(stepToFineDM), sgmPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = sgmDepth - depthOpt; // distance to smooth/robust input depth map
        const float imgColorVariance = tex2D<float>(imgVarianceTex, float(roiX), float(roiY)); // do not use 0.5f offset because imgVarianceTex use nearest neighbor interpolation
        const float colorVarianceThresholdForSmoothing = 20.0f;
        const float angleThresholdForSmoothing = 30.0f; // 30

        // https://www.desmos.com/calculator/kob9lxs9qf
        const float weightedColorVariance = sigmoid2(5.0f, angleThresholdForSmoothing, 40.0f, colorVarianceThresholdForSmoothing, imgColorVariance);

        // https://www.desmos.com/calculator/jwhpjq6ppj
        const float fineSimWeight = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, refineSim);

        // if geometry variation is bigger than color variation => the fineDM is considered noisy

        // if depthEnergy > weightedColorVariance   => energyLowerThanVarianceWeight=0 => smooth
        // else:                                    => energyLowerThanVarianceWeight=1 => use fineDM
        // weightedColorVariance max value is 30, so if depthEnergy > 30 (which means depthAngle < 150�) energyLowerThanVarianceWeight will be 0
        // https://www.desmos.com/calculator/jzbweilb85
        const float energyLowerThanVarianceWeight = sigmoid(0.0f, 1.0f, 30.0f, weightedColorVariance, depthEnergy); // TODO: 30 => 60

        // https://www.desmos.com/calculator/ilsk7pthvz
        const float closeToRoughWeight = 1.0f - sigmoid(0.0f, 1.0f, 10.0f, 17.0f, fabsf(stepToRoughDM / sgmPixSize)); // TODO: 10 => 30

        // f(z) = c1 * s1(z_rought - z)^2 + c2 * s2(z-z_fused)^2 + coeff3 * s3*(z-z_smooth)^2

        const float depthOptStep = closeToRoughWeight * stepToRoughDM + // distance to smooth/robust input depth map
                                   (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * stepToFineDM + // distance to refined/noisy
                                                                 (1.0f - energyLowerThanVarianceWeight) * stepToSmoothDepth); // max angle in current depthMap

        out_optDepthSim.x = depthOpt + depthOptStep;

        out_optDepthSim.y = (1.0f - closeToRoughWeight) * (energyLowerThanVarianceWeight * fineSimWeight * refineSim + (1.0f - energyLowerThanVarianceWeight) * (depthEnergy / 20.0f));
    }

    *out_optDepthSimPtr = out_optDepthSim;
}

} // namespace depthMap
} // namespace aliceVision
