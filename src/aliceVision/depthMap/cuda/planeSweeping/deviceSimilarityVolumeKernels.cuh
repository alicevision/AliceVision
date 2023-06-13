// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/depthMap/cuda/device/matrix.cuh>
#include <aliceVision/depthMap/cuda/device/Patch.cuh>
#include <aliceVision/depthMap/cuda/planeSweeping/similarity.hpp>

namespace aliceVision {
namespace depthMap {

inline __device__ void move3DPointByRcPixSize(float3& p,
                                              const DeviceCameraParams& rcDeviceCamParams,
                                              const float rcPixSize)
{
    float3 rpv = p - rcDeviceCamParams.C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

inline __device__ void volume_computePatch(Patch& patch,
                                           const DeviceCameraParams& rcDeviceCamParams,
                                           const DeviceCameraParams& tcDeviceCamParams,
                                           const float fpPlaneDepth,
                                           const float2& pix)
{
    patch.p = get3DPointForPixelAndFrontoParellePlaneRC(rcDeviceCamParams, pix, fpPlaneDepth);
    patch.d = computePixSize(rcDeviceCamParams, patch.p);
    computeRotCSEpip(patch, rcDeviceCamParams, tcDeviceCamParams);
}

inline __device__ float depthPlaneToDepth(const DeviceCameraParams& deviceCamParams,
                                          const float fpPlaneDepth,
                                          const float2& pix)
{
    const float3 planep = deviceCamParams.C + deviceCamParams.ZVect * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams.iP, pix);
    normalize(v);
    float3 p = linePlaneIntersect(deviceCamParams.C, v, planep, deviceCamParams.ZVect);
    return size(deviceCamParams.C - p);
}

template <typename T>
__global__ void volume_init_kernel(T* inout_volume_d, int inout_volume_s, int inout_volume_p,
                                   const unsigned int volDimX,
                                   const unsigned int volDimY,
                                   const T value)
{
    const unsigned int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int vz = blockIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    *get3DBufferAt(inout_volume_d, inout_volume_s, inout_volume_p, vx, vy, vz) = value;
}

__global__ void volume_add_kernel(TSimRefine* inout_volume_d, int inout_volume_s, int inout_volume_p, 
                                  const TSimRefine* in_volume_d, const int in_volume_s, const int in_volume_p,
                                  const unsigned int volDimX,
                                  const unsigned int volDimY)
{
    const unsigned int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int vz = blockIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    TSimRefine* outSimPtr = get3DBufferAt(inout_volume_d, inout_volume_s, inout_volume_p, vx, vy, vz);

#ifdef TSIM_REFINE_USE_HALF
    // note: using built-in half addition can give bad results on some gpus
    //*outSimPtr = __hadd(*outSimPtr, *get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, vz));
    *outSimPtr = __float2half(__half2float(*outSimPtr) + __half2float(*get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, vz))); // perform the addition in float
#else
    *outSimPtr += *get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, vz);
#endif
}

__global__ void volume_updateUninitialized_kernel(TSim* inout_volume2nd_d, int inout_volume2nd_s, int inout_volume2nd_p, 
                                                  const TSim* in_volume1st_d, const int in_volume1st_s, const int in_volume1st_p,
                                                  const unsigned int volDimX,
                                                  const unsigned int volDimY)
{
    const unsigned int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int vz = blockIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    // input/output second best similarity value
    TSim* inout_simPtr = get3DBufferAt(inout_volume2nd_d, inout_volume2nd_s, inout_volume2nd_p, vx, vy, vz);

    if(*inout_simPtr >= 255.f) // invalid or uninitialized similarity value
    {
        // update second best similarity value with first best similarity value
        *inout_simPtr = *get3DBufferAt(in_volume1st_d, in_volume1st_s, in_volume1st_p, vx, vy, vz);
    }
}

__global__ void volume_computeSimilarity_kernel(TSim* out_volume1st_d, int out_volume1st_s, int out_volume1st_p,
                                                TSim* out_volume2nd_d, int out_volume2nd_s, int out_volume2nd_p,
                                                const float* in_depths_d, const int in_depths_p,
                                                const int rcDeviceCameraParamsId,
                                                const int tcDeviceCameraParamsId,
                                                const cudaTextureObject_t rcMipmapImage_tex,
                                                const cudaTextureObject_t tcMipmapImage_tex,
                                                const unsigned int rcSgmLevelWidth,
                                                const unsigned int rcSgmLevelHeight,
                                                const unsigned int tcSgmLevelWidth,
                                                const unsigned int tcSgmLevelHeight,
                                                const float rcMipmapLevel,
                                                const int stepXY,
                                                const int wsh,
                                                const float invGammaC,
                                                const float invGammaP,
                                                const bool useConsistentScale,
                                                const bool useCustomPatchPattern,
                                                const Range depthRange,
                                                const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int roiZ = blockIdx.z;

    if(roiX >= roi.width() || roiY >= roi.height()) // no need to check roiZ
        return;

    // R and T camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];
    const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[tcDeviceCameraParamsId];

    // corresponding volume coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = depthRange.begin + roiZ;

    // corresponding image coordinates
    const float x = float(roi.x.begin + vx) * float(stepXY);
    const float y = float(roi.y.begin + vy) * float(stepXY);

    // corresponding depth plane
    const float depthPlane = *get2DBufferAt(in_depths_d, in_depths_p, size_t(vz), 0);

    // compute patch
    Patch patch;
    volume_computePatch(patch, rcDeviceCamParams, tcDeviceCamParams, depthPlane, make_float2(x, y));

    // we do not need positive and filtered similarity values
    constexpr bool invertAndFilter = false;

    float fsim = CUDART_INF_F;

    // compute patch similarity
    if(useCustomPatchPattern)
    {
        fsim = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                    tcDeviceCamParams,
                                                                    rcMipmapImage_tex,
                                                                    tcMipmapImage_tex,
                                                                    rcSgmLevelWidth,
                                                                    rcSgmLevelHeight,
                                                                    tcSgmLevelWidth,
                                                                    tcSgmLevelHeight,
                                                                    rcMipmapLevel,
                                                                    invGammaC,
                                                                    invGammaP,
                                                                    useConsistentScale,
                                                                    patch);
    }
    else
    {
        fsim = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                 tcDeviceCamParams,
                                                 rcMipmapImage_tex,
                                                 tcMipmapImage_tex,
                                                 rcSgmLevelWidth,
                                                 rcSgmLevelHeight,
                                                 tcSgmLevelWidth,
                                                 tcSgmLevelHeight,
                                                 rcMipmapLevel,
                                                 wsh,
                                                 invGammaC,
                                                 invGammaP,
                                                 useConsistentScale,
                                                 patch);
    }

    if(fsim == CUDART_INF_F) // invalid similarity
    {
      fsim = 255.0f; // 255 is the invalid similarity value
    }
    else // valid similarity
    {
      // remap similarity value
      constexpr const float fminVal = -1.0f;
      constexpr const float fmaxVal = 1.0f;
      constexpr const float fmultiplier = 1.0f / (fmaxVal - fminVal);

      fsim = (fsim - fminVal) * fmultiplier;

#ifdef TSIM_USE_FLOAT
      // no clamp
#else
      fsim = fminf(1.0f, fmaxf(0.0f, fsim));
#endif
      // convert from (0, 1) to (0, 254)
      // needed to store in the volume in uchar
      // 255 is reserved for the similarity initialization, i.e. undefined values
      fsim *= 254.0f;
    }

    TSim* fsim_1st = get3DBufferAt(out_volume1st_d, out_volume1st_s, out_volume1st_p, size_t(vx), size_t(vy), size_t(vz));
    TSim* fsim_2nd = get3DBufferAt(out_volume2nd_d, out_volume2nd_s, out_volume2nd_p, size_t(vx), size_t(vy), size_t(vz));

    if(fsim < *fsim_1st)
    {
        *fsim_2nd = *fsim_1st;
        *fsim_1st = TSim(fsim);
    }
    else if(fsim < *fsim_2nd)
    {
        *fsim_2nd = TSim(fsim);
    }
}

__global__ void volume_refineSimilarity_kernel(TSimRefine* inout_volSim_d, int inout_volSim_s, int inout_volSim_p,
                                               const float2* in_sgmDepthPixSizeMap_d, const int in_sgmDepthPixSizeMap_p,
                                               const float3* in_sgmNormalMap_d, const int in_sgmNormalMap_p,
                                               const int rcDeviceCameraParamsId,
                                               const int tcDeviceCameraParamsId,
                                               const cudaTextureObject_t rcMipmapImage_tex,
                                               const cudaTextureObject_t tcMipmapImage_tex,
                                               const unsigned int rcRefineLevelWidth,
                                               const unsigned int rcRefineLevelHeight,
                                               const unsigned int tcRefineLevelWidth,
                                               const unsigned int tcRefineLevelHeight,
                                               const float rcMipmapLevel,
                                               const int volDimZ,
                                               const int stepXY,
                                               const int wsh,
                                               const float invGammaC,
                                               const float invGammaP,
                                               const bool useConsistentScale,
                                               const bool useCustomPatchPattern,
                                               const Range depthRange,
                                               const ROI roi)
{
    const unsigned int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int roiY = blockIdx.y * blockDim.y + threadIdx.y;
    const unsigned int roiZ = blockIdx.z;

    if(roiX >= roi.width() || roiY >= roi.height()) // no need to check roiZ
        return;

    // R and T camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];
    const DeviceCameraParams& tcDeviceCamParams = constantCameraParametersArray_d[tcDeviceCameraParamsId];

    // corresponding volume and depth/sim map coordinates
    const unsigned int vx = roiX;
    const unsigned int vy = roiY;
    const unsigned int vz = depthRange.begin + roiZ;

    // corresponding image coordinates
    const float x = float(roi.x.begin + vx) * float(stepXY);
    const float y = float(roi.y.begin + vy) * float(stepXY);

    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, in_sgmDepthPixSizeMap_p, vx, vy);

    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
        return; 

    // initialize rc 3d point at sgm depth (middle depth)
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamParams, make_float2(x, y), in_sgmDepthPixSize.x);

    // compute relative depth index offset from z center
    const int relativeDepthIndexOffset = vz - ((volDimZ - 1) / 2);

    if(relativeDepthIndexOffset != 0)
    {
        // not z center
        // move rc 3d point by relative depth index offset * sgm pixSize
        const float pixSizeOffset = relativeDepthIndexOffset * in_sgmDepthPixSize.y; // input sgm pixSize
        move3DPointByRcPixSize(p, rcDeviceCamParams, pixSizeOffset);
    }

    // compute patch
    Patch patch;
    patch.p = p;
    patch.d = computePixSize(rcDeviceCamParams, p);

    // computeRotCSEpip
    {
      // vector from the reference camera to the 3d point
      float3 v1 = rcDeviceCamParams.C - patch.p;
      // vector from the target camera to the 3d point
      float3 v2 = tcDeviceCamParams.C - patch.p;
      normalize(v1);
      normalize(v2);

      // y has to be ortogonal to the epipolar plane
      // n has to be on the epipolar plane
      // x has to be on the epipolar plane

      patch.y = cross(v1, v2);
      normalize(patch.y);

      if(in_sgmNormalMap_d != nullptr) // initialize patch normal from input normal map
      {
        patch.n = *get2DBufferAt(in_sgmNormalMap_d, in_sgmNormalMap_p, vx, vy);
      }
      else // initialize patch normal from v1 & v2
      {
        patch.n = (v1 + v2) / 2.0f;
        normalize(patch.n);
      }

      patch.x = cross(patch.y, patch.n);
      normalize(patch.x);
    }

    // we need positive and filtered similarity values
    constexpr bool invertAndFilter = true;

    float fsimInvertedFiltered = CUDART_INF_F;

    // compute similarity
    if(useCustomPatchPattern)
    {
        fsimInvertedFiltered = compNCCby3DptsYK_customPatchPattern<invertAndFilter>(rcDeviceCamParams,
                                                                                    tcDeviceCamParams,
                                                                                    rcMipmapImage_tex,
                                                                                    tcMipmapImage_tex,
                                                                                    rcRefineLevelWidth,
                                                                                    rcRefineLevelHeight,
                                                                                    tcRefineLevelWidth,
                                                                                    tcRefineLevelHeight,
                                                                                    rcMipmapLevel,
                                                                                    invGammaC,
                                                                                    invGammaP,
                                                                                    useConsistentScale,
                                                                                    patch);
    }
    else
    {
        fsimInvertedFiltered = compNCCby3DptsYK<invertAndFilter>(rcDeviceCamParams,
                                                                 tcDeviceCamParams,
                                                                 rcMipmapImage_tex,
                                                                 tcMipmapImage_tex,
                                                                 rcRefineLevelWidth,
                                                                 rcRefineLevelHeight,
                                                                 tcRefineLevelWidth,
                                                                 tcRefineLevelHeight,
                                                                 rcMipmapLevel,
                                                                 wsh,
                                                                 invGammaC,
                                                                 invGammaP,
                                                                 useConsistentScale,
                                                                 patch);
    }

    if(fsimInvertedFiltered == CUDART_INF_F) // invalid similarity
    {
        // do nothing
        return;
    }

    // get output similarity pointer
    TSimRefine* outSimPtr = get3DBufferAt(inout_volSim_d, inout_volSim_s, inout_volSim_p, vx, vy, vz);

    // add the output similarity value
#ifdef TSIM_REFINE_USE_HALF
    // note: using built-in half addition can give bad results on some gpus
    //*outSimPtr = __hadd(*outSimPtr, TSimRefine(fsimInvertedFiltered));
    //*outSimPtr = __hadd(*outSimPtr, __float2half(fsimInvertedFiltered));
    *outSimPtr = __float2half(__half2float(*outSimPtr) + fsimInvertedFiltered); // perform the addition in float
#else
    *outSimPtr += TSimRefine(fsimInvertedFiltered);
#endif
}

__global__ void volume_retrieveBestDepth_kernel(float2* out_sgmDepthThicknessMap_d, int out_sgmDepthThicknessMap_p,
                                                float2* out_sgmDepthSimMap_d, int out_sgmDepthSimMap_p, // output depth/sim map is optional (nullptr)
                                                const float* in_depths_d, const int in_depths_p,
                                                const TSim* in_volSim_d, const int in_volSim_s, const int in_volSim_p,
                                                const int rcDeviceCameraParamsId,
                                                const int volDimZ, // useful for depth/sim interpolation
                                                const int scaleStep,
                                                const float thicknessMultFactor, // default 1
                                                const float maxSimilarity,
                                                const Range depthRange,
                                                const ROI roi)
{
    const unsigned int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if(vx >= roi.width() || vy >= roi.height())
        return;

    // R camera parameters
    const DeviceCameraParams& rcDeviceCamParams = constantCameraParametersArray_d[rcDeviceCameraParamsId];

    // corresponding image coordinates
    const float2 pix{float((roi.x.begin + vx) * scaleStep), float((roi.y.begin + vy) * scaleStep)};

    // corresponding output depth/thickness pointer
    float2* out_bestDepthThicknessPtr = get2DBufferAt(out_sgmDepthThicknessMap_d, out_sgmDepthThicknessMap_p, vx, vy);

    // corresponding output depth/sim pointer or nullptr
    float2* out_bestDepthSimPtr = (out_sgmDepthSimMap_d == nullptr) ? nullptr : get2DBufferAt(out_sgmDepthSimMap_d, out_sgmDepthSimMap_p, vx, vy);

    // find the best depth plane index for the current pixel
    // the best depth plane has the best similarity value
    // - best possible similarity value is 0
    // - worst possible similarity value is 254
    // - invalid similarity value is 255
    float bestSim = 255.f;
    int bestZIdx = -1;

    for(int vz = depthRange.begin; vz < depthRange.end; ++vz)
    {
      const float simAtZ = *get3DBufferAt(in_volSim_d, in_volSim_s, in_volSim_p, vx, vy, vz);

      if(simAtZ < bestSim)
      {
        bestSim = simAtZ;
        bestZIdx = vz;
      }
    }

    // filtering out invalid values and values with a too bad score (above the user maximum similarity threshold)
    // note: this helps to reduce following calculations and also the storage volume of the depth maps.
    if((bestZIdx == -1) || (bestSim > maxSimilarity))
    {
        out_bestDepthThicknessPtr->x = -1.f; // invalid depth
        out_bestDepthThicknessPtr->y = -1.f; // invalid thickness

        if(out_bestDepthSimPtr != nullptr)
        {
            out_bestDepthSimPtr->x = -1.f; // invalid depth
            out_bestDepthSimPtr->y =  1.f; // worst similarity value
        }
        return;
    }

    // find best depth plane previous and next indexes
    const int bestZIdx_m1 = max(0, bestZIdx - 1);           // best depth plane previous index
    const int bestZIdx_p1 = min(volDimZ - 1, bestZIdx + 1); // best depth plane next index

    // get best best depth current, previous and next plane depth values
    // note: float3 struct is useful for depth interpolation
    float3 depthPlanes;
    depthPlanes.x = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx_m1, 0);  // best depth previous plane
    depthPlanes.y = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx, 0);     // best depth plane
    depthPlanes.z = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx_p1, 0);  // best depth next plane

    const float bestDepth    = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.y, pix); // best depth
    const float bestDepth_m1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.x, pix); // previous best depth
    const float bestDepth_p1 = depthPlaneToDepth(rcDeviceCamParams, depthPlanes.z, pix); // next best depth

#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
    // with depth/sim interpolation
    // note: disable by default

    float3 sims;
    sims.x = *get3DBufferAt(in_volSim_d, in_volSim_s, in_volSim_p, vx, vy, bestZIdx_m1);
    sims.y = bestSim;
    sims.z = *get3DBufferAt(in_volSim_d, in_volSim_s, in_volSim_p, vx, vy, bestZIdx_p1);

    // convert sims from (0, 255) to (-1, +1)
    sims.x = (sims.x / 255.0f) * 2.0f - 1.0f;
    sims.y = (sims.y / 255.0f) * 2.0f - 1.0f;
    sims.z = (sims.z / 255.0f) * 2.0f - 1.0f;

    // interpolation between the 3 depth planes candidates
    const float refinedDepthPlane = refineDepthSubPixel(depthPlanes, sims);

    const float out_bestDepth = depthPlaneToDepth(rcDeviceCamParams, refinedDepthPlane, pix);
    const float out_bestSim = sims.y;
#else
    // without depth interpolation
    const float out_bestDepth = bestDepth;
    const float out_bestSim = (bestSim / 255.0f) * 2.0f - 1.0f; // convert from (0, 255) to (-1, +1)
#endif

    // compute output best depth thickness
    // thickness is the maximum distance between output best depth and previous or next depth
    // thickness can be inflate with thicknessMultFactor
    const float out_bestDepthThickness = max(bestDepth_p1 - out_bestDepth, out_bestDepth - bestDepth_m1) * thicknessMultFactor;

    // write output depth/thickness
    out_bestDepthThicknessPtr->x = out_bestDepth;
    out_bestDepthThicknessPtr->y = out_bestDepthThickness;

    if(out_sgmDepthSimMap_d != nullptr)
    {
        // write output depth/sim
        out_bestDepthSimPtr->x = out_bestDepth;
        out_bestDepthSimPtr->y = out_bestSim;
    }
}


__global__ void volume_refineBestDepth_kernel(float2* out_refineDepthSimMap_d, int out_refineDepthSimMap_p,
                                              const float2* in_sgmDepthPixSizeMap_d, int in_sgmDepthPixSizeMap_p,
                                              const TSimRefine* in_volSim_d, int in_volSim_s, int in_volSim_p,
                                              int volDimZ,
                                              int samplesPerPixSize, // number of subsamples (samples between two depths)
                                              int halfNbSamples,     // number of samples (in front and behind mid depth)
                                              int halfNbDepths,      // number of depths  (in front and behind mid depth) should be equal to (volDimZ - 1) / 2
                                              float twoTimesSigmaPowerTwo,
                                              const ROI roi)
{
    const unsigned int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if(vx >= roi.width() || vy >= roi.height())
        return;

    // corresponding input sgm depth/pixSize (middle depth)
    const float2 in_sgmDepthPixSize = *get2DBufferAt(in_sgmDepthPixSizeMap_d, in_sgmDepthPixSizeMap_p, vx, vy);

    // corresponding output depth/sim pointer
    float2* out_bestDepthSimPtr = get2DBufferAt(out_refineDepthSimMap_d, out_refineDepthSimMap_p, vx, vy);

    // sgm depth (middle depth) invalid or masked
    if(in_sgmDepthPixSize.x <= 0.0f)
    {
        out_bestDepthSimPtr->x = in_sgmDepthPixSize.x;  // -1 (invalid) or -2 (masked)
        out_bestDepthSimPtr->y = 1.0f;                  // similarity between (-1, +1)
        return;
    }

    // find best z sample per pixel
    float bestSampleSim = 0.f;      // all sample sim <= 0.f
    int bestSampleOffsetIndex = 0;  // default is middle depth (SGM)

    // sliding gaussian window
    for(int sample = -halfNbSamples; sample <= halfNbSamples; ++sample)
    {
        float sampleSim = 0.f; 

        for(int vz = 0; vz < volDimZ; ++vz)
        {
            const int rz = (vz - halfNbDepths);    // relative depth index offset
            const int zs = rz * samplesPerPixSize; // relative sample offset

            // get the inverted similarity sum value
            // best value is the HIGHEST
            // worst value is 0
            const float invSimSum = *get3DBufferAt(in_volSim_d, in_volSim_s, in_volSim_p, vx, vy, vz);

            // reverse the inverted similarity sum value
            // best value is the LOWEST
            // worst value is 0
            const float simSum = -invSimSum;

            // apply gaussian
            // see: https://www.desmos.com/calculator/ribalnoawq
            sampleSim += simSum * expf(-((zs - sample) * (zs - sample)) / twoTimesSigmaPowerTwo); 
        }

        if(sampleSim < bestSampleSim)
        {
            bestSampleOffsetIndex = sample;
            bestSampleSim = sampleSim;
        }
    }

    // compute sample size
    const float sampleSize = in_sgmDepthPixSize.y / samplesPerPixSize; // input sgm pixSize / samplesPerPixSize

    // compute sample size offset from z center
    const float sampleSizeOffset = bestSampleOffsetIndex * sampleSize;

    // compute best depth
    // input sgm depth (middle depth) + sample size offset from z center
    const float bestDepth = in_sgmDepthPixSize.x + sampleSizeOffset;

    // write output best depth/sim
    out_bestDepthSimPtr->x = bestDepth;
    out_bestDepthSimPtr->y = bestSampleSim;
}

template <typename T>
__global__ void volume_initVolumeYSlice_kernel(T* volume_d, int volume_s, int volume_p, const int3 volDim, const int3 axisT, int y, T cst)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if ((x >= 0) && (x < (&volDim.x)[axisT.x]) && (z >= 0) && (z < (&volDim.x)[axisT.z]))
    {
        T* volume_zyx = get3DBufferAt(volume_d, volume_s, volume_p, v.x, v.y, v.z);
        *volume_zyx = cst;
    }
}

template <typename T1, typename T2>
__global__ void volume_getVolumeXZSlice_kernel(T1* slice_d, int slice_p,
                                               const T2* volume_d, int volume_s, int volume_p,
                                               const int3 volDim, const int3 axisT, int y)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if (x >= (&volDim.x)[axisT.x] || z >= (&volDim.x)[axisT.z])
      return;

    const T2* volume_xyz = get3DBufferAt(volume_d, volume_s, volume_p, v);
    T1* slice_xz = get2DBufferAt(slice_d, slice_p, x, z);
    *slice_xz = (T1)(*volume_xyz);
}

__global__ void volume_computeBestZInSlice_kernel(TSimAcc* xzSlice_d, int xzSlice_p, TSimAcc* ySliceBestInColCst_d, int volDimX, int volDimZ)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x >= volDimX)
        return;

    TSimAcc bestCst = *get2DBufferAt(xzSlice_d, xzSlice_p, x, 0);

    for(int z = 1; z < volDimZ; ++z)
    {
        const TSimAcc cst = *get2DBufferAt(xzSlice_d, xzSlice_p, x, z);
        bestCst = cst < bestCst ? cst : bestCst;  // min(cst, bestCst);
    }
    ySliceBestInColCst_d[x] = bestCst;
}

/**
 * @param[inout] xySliceForZ input similarity plane
 * @param[in] xySliceForZM1
 * @param[in] xSliceBestInColCst
 * @param[out] volSimT output similarity volume
 */
__global__ void volume_agregateCostVolumeAtXinSlices_kernel(const cudaTextureObject_t rcMipmapImage_tex,
                                                            const unsigned int rcSgmLevelWidth,
                                                            const unsigned int rcSgmLevelHeight,
                                                            const float rcMipmapLevel,
                                                            TSimAcc* xzSliceForY_d, int xzSliceForY_p,
                                                            const TSimAcc* xzSliceForYm1_d, const int xzSliceForYm1_p,
                                                            const TSimAcc* bestSimInYm1_d,
                                                            TSim* volAgr_d, const int volAgr_s, const int volAgr_p,
                                                            const int3 volDim,
                                                            const int3 axisT,
                                                            const float step,
                                                            const int y,
                                                            const float P1,
                                                            const float _P2,
                                                            const int ySign,
                                                            const int filteringIndex,
                                                            const ROI roi)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if (x >= (&volDim.x)[axisT.x] || z >= volDim.z)
        return;

    // find texture offset
    const int beginX = (axisT.x == 0) ? roi.x.begin : roi.y.begin;
    const int beginY = (axisT.x == 0) ? roi.y.begin : roi.x.begin;

    TSimAcc* sim_xz = get2DBufferAt(xzSliceForY_d, xzSliceForY_p, x, z);
    float pathCost = 255.0f;

    if((z >= 1) && (z < volDim.z - 1))
    {
        float P2 = 0;

        if(_P2 < 0)
        {
          // _P2 convention: use negative value to skip the use of deltaC.
          P2 = std::abs(_P2);
        }
        else
        {
          const int imX0 = (beginX + v.x) * step; // current
          const int imY0 = (beginY + v.y) * step;

          const int imX1 = imX0 - ySign * step * (axisT.y == 0); // M1
          const int imY1 = imY0 - ySign * step * (axisT.y == 1);

          const float4 gcr0 = tex2DLod<float4>(rcMipmapImage_tex, (float(imX0) + 0.5f) / float(rcSgmLevelWidth), (float(imY0) + 0.5f) / float(rcSgmLevelHeight), rcMipmapLevel);
          const float4 gcr1 = tex2DLod<float4>(rcMipmapImage_tex, (float(imX1) + 0.5f) / float(rcSgmLevelWidth), (float(imY1) + 0.5f) / float(rcSgmLevelHeight), rcMipmapLevel);
          const float deltaC = euclideanDist3(gcr0, gcr1);

          // sigmoid f(x) = i + (a - i) * (1 / ( 1 + e^(10 * (x - P2) / w)))
          // see: https://www.desmos.com/calculator/1qvampwbyx
          // best values found from tests: i = 80, a = 255, w = 80, P2 = 100
          // historical values: i = 15, a = 255, w = 80, P2 = 20
          P2 = sigmoid(80.f, 255.f, 80.f, _P2, deltaC);
        }

        const TSimAcc bestCostInColM1 = bestSimInYm1_d[x];
        const TSimAcc pathCostMDM1 = *get2DBufferAt(xzSliceForYm1_d, xzSliceForYm1_p, x, z - 1); // M1: minus 1 over depths
        const TSimAcc pathCostMD   = *get2DBufferAt(xzSliceForYm1_d, xzSliceForYm1_p, x, z);
        const TSimAcc pathCostMDP1 = *get2DBufferAt(xzSliceForYm1_d, xzSliceForYm1_p, x, z + 1); // P1: plus 1 over depths
        const float minCost = multi_fminf(pathCostMD, pathCostMDM1 + P1, pathCostMDP1 + P1, bestCostInColM1 + P2);

        // if 'pathCostMD' is the minimal value of the depth
        pathCost = (*sim_xz) + minCost - bestCostInColM1;
    }

    // fill the current slice with the new similarity score
    *sim_xz = TSimAcc(pathCost);

#ifndef TSIM_USE_FLOAT
    // clamp if TSim = uchar (TSimAcc = unsigned int)
    pathCost = fminf(255.0f, fmaxf(0.0f, pathCost));
#endif

    // aggregate into the final output
    TSim* volume_xyz = get3DBufferAt(volAgr_d, volAgr_s, volAgr_p, v.x, v.y, v.z);
    const float val = (float(*volume_xyz) * float(filteringIndex) + pathCost) / float(filteringIndex + 1);
    *volume_xyz = TSim(val);
}

} // namespace depthMap
} // namespace aliceVision
