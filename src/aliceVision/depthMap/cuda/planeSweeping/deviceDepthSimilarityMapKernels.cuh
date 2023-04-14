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

// compute per pixel pixSize instead of using Sgm depth thickness
//#define ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP

namespace aliceVision {
namespace depthMap {

/**
 * @return (smoothStep, energy)
 */
__device__ float2 getCellSmoothStepEnergy(const DeviceCameraParams& rcDeviceCamParams,
                                          const cudaTextureObject_t in_depth_tex,
                                          const float2& cell0,
                                          const float2& offsetRoi)
{
    float2 out = make_float2(0.0f, 180.0f);

    // get pixel depth from the depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float d0 = tex2D<float>(in_depth_tex, cell0.x, cell0.y);

    // early exit: depth is <= 0
    if(d0 <= 0.0f)
        return out;

    // consider the neighbor pixels
    const float2 cellL = cell0 + make_float2( 0.f, -1.f); // Left
    const float2 cellR = cell0 + make_float2( 0.f,  1.f); // Right
    const float2 cellU = cell0 + make_float2(-1.f,  0.f); // Up
    const float2 cellB = cell0 + make_float2( 1.f,  0.f); // Bottom

    // get associated depths from depth texture
    // note: we do not use 0.5f offset because in_depth_tex use nearest neighbor interpolation
    const float dL = tex2D<float>(in_depth_tex, cellL.x, cellL.y);
    const float dR = tex2D<float>(in_depth_tex, cellR.x, cellR.y);
    const float dU = tex2D<float>(in_depth_tex, cellU.x, cellU.y);
    const float dB = tex2D<float>(in_depth_tex, cellB.x, cellB.y);

    // get associated 3D points
    const float3 p0 = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cell0 + offsetRoi, d0);
    const float3 pL = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellL + offsetRoi, dL);
    const float3 pR = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellR + offsetRoi, dR);
    const float3 pU = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellU + offsetRoi, dU);
    const float3 pB = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, cellB + offsetRoi, dB);

    // compute the average point based on neighbors (cg)
    float3 cg = make_float3(0.0f, 0.0f, 0.0f);
    float n = 0.0f;

    if(dL > 0.0f) { cg = cg + pL; n++; }
    if(dR > 0.0f) { cg = cg + pR; n++; }
    if(dU > 0.0f) { cg = cg + pU; n++; }
    if(dB > 0.0f) { cg = cg + pB; n++; }

    // if we have at least one valid depth
    if(n > 1.0f)
    {
        cg = cg / n; // average of x, y, depth
        float3 vcn = rcDeviceCamParams.C - p0;
        normalize(vcn);
        // pS: projection of cg on the line from p0 to camera
        const float3 pS = closestPointToLine3D(cg, p0, vcn);
        // keep the depth difference between pS and p0 as the smoothing step
        out.x = size(rcDeviceCamParams.C - pS) - d0;
    }

    float e = 0.0f;
    n = 0.0f;

    if(dL > 0.0f && dR > 0.0f)
    {
        // large angle between neighbors == flat area => low energy
        // small angle between neighbors == non-flat area => high energy
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pL, pR)));
        n++;
    }
    if(dU > 0.0f && dB > 0.0f)
    {
        e = fmaxf(e, (180.0f - angleBetwABandAC(p0, pU, pB)));
        n++;
    }
    // the higher the energy, the less flat the area
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
                                                const float2* in_depthSimMap_d, const int in_depthSimMap_p,
                                                const unsigned int width,
                                                const unsigned int height,
                                                const float defaultSim)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

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
                                  const float ratio,
                                  const ROI roi)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

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

__global__ void depthThicknessMapSmoothThickness_kernel(float2* inout_depthThicknessMap_d, int inout_depthThicknessMap_p,
                                                      const float minThicknessInflate,
                                                      const float maxThicknessInflate,
                                                      const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding output depth/thickness (depth unchanged)
    float2* inout_depthThickness = get2DBufferAt(inout_depthThicknessMap_d, inout_depthThicknessMap_p, roiX, roiY);

    // depth invalid or masked
    if(inout_depthThickness->x <= 0.0f)
        return;

    const float minThickness = minThicknessInflate * inout_depthThickness->y;
    const float maxThickness = maxThicknessInflate * inout_depthThickness->y;

    // compute average depth distance to the center pixel
    float sumCenterDepthDist = 0.f;
    int nbValidPatchPixels = 0;

    // patch 3x3
    for(int yp = -1; yp <= 1; ++yp)
    {
        for(int xp = -1; xp <= 1; ++xp)
        {
            // compute patch coordinates
            const int roiXp = int(roiX) + xp;
            const int roiYp = int(roiY) + yp;

            if((xp == 0 && yp == 0) ||                // avoid pixel center
               roiXp < 0 || roiXp >= roi.width() ||   // avoid pixel outside the ROI
               roiYp < 0 || roiYp >= roi.height())    // avoid pixel outside the ROI
            {
                continue;
            }

            // corresponding path depth/thickness
            const float2 in_depthThicknessPatch = *get2DBufferAt(inout_depthThicknessMap_d, inout_depthThicknessMap_p, roiXp, roiYp);

            // patch depth valid
            if(in_depthThicknessPatch.x > 0.0f)
            {
                const float depthDistance = abs(inout_depthThickness->x - in_depthThicknessPatch.x);
                sumCenterDepthDist += max(minThickness, min(maxThickness, depthDistance)); // clamp (minThickness, maxThickness)
                ++nbValidPatchPixels;
            }
        }
    }

    // we require at least 3 valid patch pixels (over 8)
    if(nbValidPatchPixels < 3)
        return;

    // write output smooth thickness
    inout_depthThickness->y = sumCenterDepthDist / nbValidPatchPixels;
}
__global__ void computeSgmUpscaledDepthPixSizeMap_nearestNeighbor_kernel(float2* out_upscaledDepthPixSizeMap_d, int out_upscaledDepthPixSizeMap_p,
                                                                         const float2* in_sgmDepthThicknessMap_d, const int in_sgmDepthThicknessMap_p,
                                                                         const int rcDeviceCameraParamsId, // useful for direct pixSize computation
                                                                         const cudaTextureObject_t rcMipmapImage_tex,
                                                                         const unsigned int rcLevelWidth,
                                                                         const unsigned int rcLevelHeight,
                                                                         const float rcMipmapLevel,
                                                                         const int stepXY,
                                                                         const int halfNbDepths,
                                                                         const float ratio,
                                                                         const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding image coordinates
    const unsigned int x = (roi.x.begin + roiX) * (unsigned int)(stepXY);
    const unsigned int y = (roi.y.begin + roiY) * (unsigned int)(stepXY);

    // corresponding output upscaled depth/pixSize map
    float2* out_depthPixSize = get2DBufferAt(out_upscaledDepthPixSizeMap_d, out_upscaledDepthPixSizeMap_p, roiX, roiY);

    // filter masked pixels (alpha < 0.9f)
    if(tex2DLod<float4>(rcMipmapImage_tex, (float(x) + 0.5f) / float(rcLevelWidth), (float(y) + 0.5f) / float(rcLevelHeight), rcMipmapLevel).w < 0.9f)
    {
        *out_depthPixSize = make_float2(-2.f, 0.f);
        return;
    }

    // find corresponding depth/thickness
    // nearest neighbor, no interpolation
    const float oy = (float(roiY) - 0.5f) * ratio;
    const float ox = (float(roiX) - 0.5f) * ratio;

    int xp = floor(ox + 0.5);
    int yp = floor(oy + 0.5);

    xp = min(xp, int(roi.width()  * ratio) - 1);
    yp = min(yp, int(roi.height() * ratio) - 1);

    const float2 out_depthThickness = *get2DBufferAt(in_sgmDepthThicknessMap_d, in_sgmDepthThicknessMap_p, xp, yp);

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
    // R camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), out_depthThickness.x);

    // compute and write rc 3d point pixSize
    const float out_pixSize = computePixSize(rcDeviceCamParams, p);
#else
    // compute pixSize from depth thickness
    const float out_pixSize = out_depthThickness.y / halfNbDepths;
#endif

    // write output depth/pixSize
    out_depthPixSize->x = out_depthThickness.x;
    out_depthPixSize->y = out_pixSize;
}

__global__ void computeSgmUpscaledDepthPixSizeMap_bilinear_kernel(float2* out_upscaledDepthPixSizeMap_d, int out_upscaledDepthPixSizeMap_p,
                                                                  const float2* in_sgmDepthThicknessMap_d, const int in_sgmDepthThicknessMap_p,
                                                                  const int rcDeviceCameraParamsId, // useful for direct pixSize computation
                                                                  const cudaTextureObject_t rcMipmapImage_tex,
                                                                  const unsigned int rcLevelWidth,
                                                                  const unsigned int rcLevelHeight,
                                                                  const float rcMipmapLevel,
                                                                  const int stepXY,
                                                                  const int halfNbDepths,
                                                                  const float ratio,
                                                                  const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding image coordinates
    const unsigned int x = (roi.x.begin + roiX) * (unsigned int)(stepXY);
    const unsigned int y = (roi.y.begin + roiY) * (unsigned int)(stepXY);

    // corresponding output upscaled depth/pixSize map
    float2* out_depthPixSize = get2DBufferAt(out_upscaledDepthPixSizeMap_d, out_upscaledDepthPixSizeMap_p, roiX, roiY);

    // filter masked pixels with alpha
    if(tex2DLod<float4>(rcMipmapImage_tex, (float(x) + 0.5f) / float(rcLevelWidth), (float(y) + 0.5f) / float(rcLevelHeight), rcMipmapLevel).w < ALICEVISION_DEPTHMAP_RC_MIN_ALPHA)
    {
        *out_depthPixSize = make_float2(-2.f, 0.f);
        return;
    }

    // find adjacent pixels
    const float oy = (float(roiY) - 0.5f) * ratio;
    const float ox = (float(roiX) - 0.5f) * ratio;

    int xp = floor(ox);
    int yp = floor(oy);

    xp = min(xp, int(roi.width()  * ratio) - 2);
    yp = min(yp, int(roi.height() * ratio) - 2);

    const float2 lu = *get2DBufferAt(in_sgmDepthThicknessMap_d, in_sgmDepthThicknessMap_p, xp, yp);
    const float2 ru = *get2DBufferAt(in_sgmDepthThicknessMap_d, in_sgmDepthThicknessMap_p, xp + 1, yp);
    const float2 rd = *get2DBufferAt(in_sgmDepthThicknessMap_d, in_sgmDepthThicknessMap_p, xp + 1, yp + 1);
    const float2 ld = *get2DBufferAt(in_sgmDepthThicknessMap_d, in_sgmDepthThicknessMap_p, xp, yp + 1);

    // find corresponding depth/thickness
    float2 out_depthThickness;

    if(lu.x <= 0.0f || ru.x <= 0.0f || rd.x <= 0.0f || ld.x <= 0.0f)
    {
        // at least one corner depth is invalid
        // average the other corners to get a proper depth/thickness
        float2 sumDepthThickness = {0.0f, 0.0f};
        int count = 0;

        if(lu.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + lu;
            ++count;
        }
        if(ru.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + ru;
            ++count;
        }
        if(rd.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + rd;
            ++count;
        }
        if(ld.x > 0.0f)
        {
            sumDepthThickness = sumDepthThickness + ld;
            ++count;
        }
        if(count != 0)
        {
            out_depthThickness = {sumDepthThickness.x / float(count), sumDepthThickness.y / float(count)};
        }
        else
        {
            // invalid depth
            *out_depthPixSize = {-1.0f, 1.0f};
            return;
        }
    }
    else
    {
        // bilinear interpolation
        const float ui = ox - float(xp);
        const float vi = oy - float(yp);
        const float2 u = lu + (ru - lu) * ui;
        const float2 d = ld + (rd - ld) * ui;
        out_depthThickness = u + (d - u) * vi;
    }

#ifdef ALICEVISION_DEPTHMAP_COMPUTE_PIXSIZEMAP
    // R camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];

    // get rc 3d point
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), out_depthThickness.x);

    // compute and write rc 3d point pixSize
    const float out_pixSize = computePixSize(rcDeviceCamParams, p);
#else
    // compute pixSize from depth thickness
    const float out_pixSize = out_depthThickness.y / halfNbDepths;
#endif

    // write output depth/pixSize
    out_depthPixSize->x = out_depthThickness.x;
    out_depthPixSize->y = out_pixSize;
}

template<int TWsh>
__global__ void depthSimMapComputeNormal_kernel(float3* out_normalMap_d, int out_normalMap_p,
                                                const float2* in_depthSimMap_d, int in_depthSimMap_p,
                                                const int rcDeviceCameraParamsId,
                                                const int stepXY,
                                                const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // R camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];

    // corresponding image coordinates
    const unsigned int x = (roi.x.begin + roiX) * (unsigned int)(stepXY);
    const unsigned int y = (roi.y.begin + roiY) * (unsigned int)(stepXY);

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

    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x), float(y)), in_depth);
    const float pixSize = size(p - get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(float(x + 1), float(y)), in_depth));

    cuda_stat3d s3d = cuda_stat3d();

#pragma unroll
    for(int yp = -TWsh; yp <= TWsh; ++yp)
    {
        const int roiYp = int(roiY) + yp;
        if(roiYp < 0)
            continue;

#pragma unroll
        for(int xp = -TWsh; xp <= TWsh; ++xp)
        {
            const int roiXp = int(roiX) + xp;
            if(roiXp < 0)
                continue;

            const float depthP = get2DBufferAt(in_depthSimMap_d, in_depthSimMap_p, roiXp, roiYp)->x;  // use only depth

            if((depthP > 0.0f) && (fabs(depthP - in_depth) < 30.0f * pixSize))
            {
                const float w = 1.0f;
                const float2 pixP = make_float2(float(int(x) + xp), float(int(y) + yp));
                const float3 pP = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, pixP, depthP);
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

    float3 nc = rcDeviceCamParams.C - p;
    normalize(nc);

    if(orientedPointPlaneDistanceNormalizedNormal(pp + nn, pp, nc) < 0.0f)
    {
        nn.x = -nn.x;
        nn.y = -nn.y;
        nn.z = -nn.z;
    }

    *out_normal = nn;
}

__global__ void optimize_varLofLABtoW_kernel(float* out_varianceMap_d, int out_varianceMap_p,
                                             const cudaTextureObject_t rcMipmapImage_tex,
                                             const unsigned int rcLevelWidth,
                                             const unsigned int rcLevelHeight,
                                             const float rcMipmapLevel,
                                             const int stepXY,
                                             const ROI roi)
{
    // roi and varianceMap coordinates 
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding image coordinates
    const float x = float(roi.x.begin + roiX) * float(stepXY);
    const float y = float(roi.y.begin + roiY) * float(stepXY);

    // compute inverse width / height
    // note: useful to compute p1 / m1 normalized coordinates
    const float invLevelWidth  = 1.f / float(rcLevelWidth);
    const float invLevelHeight = 1.f / float(rcLevelHeight);

    // compute gradient size of L
    // note: we use 0.5f offset because rcTex texture use interpolation
    const float xM1 = tex2DLod<float4>(rcMipmapImage_tex, ((x - 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight, rcMipmapLevel).x;
    const float xP1 = tex2DLod<float4>(rcMipmapImage_tex, ((x + 1.f) + 0.5f) * invLevelWidth, ((y + 0.f) + 0.5f) * invLevelHeight, rcMipmapLevel).x;
    const float yM1 = tex2DLod<float4>(rcMipmapImage_tex, ((x + 0.f) + 0.5f) * invLevelWidth, ((y - 1.f) + 0.5f) * invLevelHeight, rcMipmapLevel).x;
    const float yP1 = tex2DLod<float4>(rcMipmapImage_tex, ((x + 0.f) + 0.5f) * invLevelWidth, ((y + 1.f) + 0.5f) * invLevelHeight, rcMipmapLevel).x;

    const float2 g = make_float2(xM1 - xP1, yM1 - yP1); // TODO: not divided by 2?
    const float grad = size(g);

    // write output
    *get2DBufferAt(out_varianceMap_d, out_varianceMap_p, roiX, roiY) = grad;
}

__global__ void optimize_getOptDeptMapFromOptDepthSimMap_kernel(float* out_tmpOptDepthMap_d, int out_tmpOptDepthMap_p,
                                                                const float2* in_optDepthSimMap_d, const int in_optDepthSimMap_p,
                                                                const ROI roi)
{
    // roi and depth/sim map part coordinates 
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    *get2DBufferAt(out_tmpOptDepthMap_d, out_tmpOptDepthMap_p, roiX, roiY) = get2DBufferAt(in_optDepthSimMap_d, in_optDepthSimMap_p, roiX, roiY)->x; // depth
}

__global__ void optimize_depthSimMap_kernel(float2* out_optimizeDepthSimMap_d, int out_optimizeDepthSimMap_p,         // output optimized depth/sim map
                                            const float2* in_sgmDepthPixSizeMap_d, const int in_sgmDepthPixSizeMap_p, // input upscaled rough depth/pixSize map
                                            const float2* in_refineDepthSimMap_d, const int in_refineDepthSimMap_p,   // input fine depth/sim map
                                            const int rcDeviceCameraParamsId,
                                            const cudaTextureObject_t imgVariance_tex,
                                            const cudaTextureObject_t depth_tex,
                                            const int iter,
                                            const ROI roi)
{
    // roi and imgVariance_tex, depth_tex coordinates
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // R camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];

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
        const float2 depthSmoothStepEnergy = getCellSmoothStepEnergy(rcDeviceCamParams, depth_tex, {float(roiX), float(roiY)}, {float(roi.x.begin), float(roi.y.begin)}); // (smoothStep, energy)
        float stepToSmoothDepth = depthSmoothStepEnergy.x;
        stepToSmoothDepth = copysignf(fminf(fabsf(stepToSmoothDepth), sgmPixSize / 10.0f), stepToSmoothDepth);
        const float depthEnergy = depthSmoothStepEnergy.y; // max angle with neighbors
        float stepToFineDM = refineDepth - depthOpt; // distance to refined/noisy input depth map
        stepToFineDM = copysignf(fminf(fabsf(stepToFineDM), sgmPixSize / 10.0f), stepToFineDM);

        const float stepToRoughDM = sgmDepth - depthOpt; // distance to smooth/robust input depth map
        const float imgColorVariance = tex2D<float>(imgVariance_tex, float(roiX), float(roiY)); // do not use 0.5f offset because imgVariance_tex use nearest neighbor interpolation
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
