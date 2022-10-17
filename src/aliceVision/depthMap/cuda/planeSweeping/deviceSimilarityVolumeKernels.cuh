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

inline __device__ void move3DPointByRcPixSize(int deviceCamId, float3& p, float rcPixSize)
{
    float3 rpv = p - constantCameraParametersArray_d[deviceCamId].C;
    normalize(rpv);
    p = p + rpv * rcPixSize;
}

inline __device__ void volume_computePatch(int rcDeviceCamId, int tcDeviceCamId, Patch& ptch, const float fpPlaneDepth, const int2& pix)
{
    ptch.p = get3DPointForPixelAndFrontoParellePlaneRC(rcDeviceCamId, pix, fpPlaneDepth); // no texture use
    ptch.d = computePixSize(rcDeviceCamId, ptch.p);                                       // no texture use
    computeRotCSEpip(rcDeviceCamId, tcDeviceCamId, ptch);                                 // no texture use
}

__device__ float depthPlaneToDepth(int deviceCamId, const float2& pix, float fpPlaneDepth)
{
    const DeviceCameraParams& deviceCamParams = constantCameraParametersArray_d[deviceCamId];
    float3 planen = M3x3mulV3(deviceCamParams.iR, make_float3(0.0f, 0.0f, 1.0f));
    normalize(planen);
    float3 planep = deviceCamParams.C + planen * fpPlaneDepth;
    float3 v = M3x3mulV2(deviceCamParams.iP, pix);
    normalize(v);
    float3 p = linePlaneIntersect(deviceCamParams.C, v, planep, planen);
    float depth = size(deviceCamParams.C - p);
    return depth;
}

template <typename T>
__global__ void volume_init_kernel(T* inout_volume_d, int inout_volume_s, int inout_volume_p, int volDimX, int volDimY, T value)
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    *get3DBufferAt(inout_volume_d, inout_volume_s, inout_volume_p, vx, vy, vz) = value;
}

__global__ void volume_add_kernel(TSimRefine* inout_volume_d, int inout_volume_s, int inout_volume_p, 
                                  const TSimRefine* in_volume_d, int in_volume_s, int in_volume_p, 
                                  int volDimX, int volDimY)
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    TSimRefine* outSimPtr = get3DBufferAt(inout_volume_d, inout_volume_s, inout_volume_p, vx, vy, vz);
    *outSimPtr += *get3DBufferAt(in_volume_d, in_volume_s, in_volume_p, vx, vy, vz);
}

__global__ void volume_slice_kernel(cudaTextureObject_t rcTex,
                                    cudaTextureObject_t tcTex,
                                    int rcDeviceCamId,
                                    int tcDeviceCamId,
                                    int rcWidth, int rcHeight,
                                    int tcWidth, int tcHeight,
                                    const float gammaC, 
                                    const float gammaP, 
                                    const int wsh,
                                    const int stepXY,
                                    const float* in_depths_d, int in_depths_p, 
                                    TSim* out_volume_1st, int out_volume1st_s, int out_volume1st_p,
                                    TSim* out_volume_2nd, int out_volume2nd_s, int out_volume2nd_p,
                                    const Range depthRange,
                                    const ROI roi)
{
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;
    const int roiZ = blockIdx.z;

    if(roiX >= roi.width() || roiY >= roi.height()) // no need to check roiZ
        return;

    // corresponding volume coordinates
    const int vx = roiX;
    const int vy = roiY;
    const int vz = depthRange.begin + roiZ;

    // corresponding device image coordinates
    const int x = (roi.x.begin + vx) * stepXY;
    const int y = (roi.y.begin + vy) * stepXY;

    // corresponding depth plane
    const float depthPlane = *get2DBufferAt(in_depths_d, in_depths_p, vz, 0);

    // compute patch
    Patch ptcho;
    volume_computePatch(rcDeviceCamId, tcDeviceCamId, ptcho, depthPlane, make_int2(x, y)); // no texture use

    // compute patch similarity
    float fsim = compNCCby3DptsYK(rcTex, tcTex, rcDeviceCamId, tcDeviceCamId, ptcho, rcWidth, rcHeight, tcWidth, tcHeight, wsh, gammaC, gammaP);

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

    TSim* fsim_1st = get3DBufferAt(out_volume_1st, out_volume1st_s, out_volume1st_p, vx, vy, vz);
    TSim* fsim_2nd = get3DBufferAt(out_volume_2nd, out_volume2nd_s, out_volume2nd_p, vx, vy, vz);

    if (fsim < *fsim_1st)
    {
        *fsim_2nd = *fsim_1st;
        *fsim_1st = TSim(fsim);
    }
    else if (fsim < *fsim_2nd)
    {
        *fsim_2nd = TSim(fsim);
    }
}

__global__ void volume_refine_kernel(cudaTextureObject_t rcTex, 
                                     cudaTextureObject_t tcTex, 
                                     int rcDeviceCamId,
                                     int tcDeviceCamId, 
                                     int rcWidth, int rcHeight, 
                                     int tcWidth, int tcHeight,
                                     int volDimZ,
                                     int stepXY,
                                     int wsh, 
                                     float gammaC, 
                                     float gammaP, 
                                     const float2* in_midDepthSimMap_d, int in_midDepthSimMap_p,
                                     const float3* in_normalMap_d, int in_normalMap_p,
                                     TSimRefine* inout_volSim_d, int inout_volSim_s, int inout_volSim_p, 
                                     const Range depthRange,
                                     const ROI roi)
{
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;
    const int roiZ = blockIdx.z;

    if(roiX >= roi.width() || roiY >= roi.height()) // no need to check roiZ
        return;

    // corresponding volume and depth/sim map coordinates
    const int vx = roiX;
    const int vy = roiY;
    const int vz = depthRange.begin + roiZ;

    // corresponding device image coordinates
    const int x = (roi.x.begin + vx) * stepXY;
    const int y = (roi.y.begin + vy) * stepXY;

    // corresponding original plane depth
    const float originalDepth = get2DBufferAt(in_midDepthSimMap_d, in_midDepthSimMap_p, vx, vy)->x; // input original middle depth

    // original depth invalid or masked, similarity value remain at 255
    if(originalDepth < 0.0f) 
        return; 

    // get rc 3d point at original depth (z center)
    float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, make_int2(x, y), originalDepth);

    // move rc 3d point according to the relative depth
    const int relativeDepthIndexOffset = vz - ((volDimZ - 1) / 2);
    if(relativeDepthIndexOffset != 0)
    {
        const float pixSizeOffset = relativeDepthIndexOffset * computePixSize(rcDeviceCamId, p);
        move3DPointByRcPixSize(rcDeviceCamId, p, pixSizeOffset);
    }

    // compute patch
    Patch ptch;
    ptch.p = p;
    ptch.d = computePixSize(rcDeviceCamId, p);

    // computeRotCSEpip
    {
      // Vector from the reference camera to the 3d point
      float3 v1 = constantCameraParametersArray_d[rcDeviceCamId].C - ptch.p;
      // Vector from the target camera to the 3d point
      float3 v2 = constantCameraParametersArray_d[tcDeviceCamId].C - ptch.p;
      normalize(v1);
      normalize(v2);

      // y has to be ortogonal to the epipolar plane
      // n has to be on the epipolar plane
      // x has to be on the epipolar plane

      ptch.y = cross(v1, v2);
      normalize(ptch.y);

      if(in_normalMap_d != nullptr) // initialize patch normal from input normal map
      {
        ptch.n = *get2DBufferAt(in_normalMap_d, in_normalMap_p, vx, vy);
      }
      else // initialize patch normal from v1 & v2
      {
        ptch.n = (v1 + v2) / 2.0f;
        normalize(ptch.n);
      }

      ptch.x = cross(ptch.y, ptch.n);
      normalize(ptch.x);
    }

    // compute similarity
    // TODO: this function should return a similarity value between -1 and 0 or 1 for infinite.
    //       in practice this function return value between -1 and 1.
    float fsim = compNCCby3DptsYK(rcTex, tcTex, rcDeviceCamId, tcDeviceCamId, ptch, rcWidth, rcHeight, tcWidth, tcHeight, wsh, gammaC, gammaP);

    if(fsim == 1.f || fsim == CUDART_INF_F) // infinite or invalid similarity
    {
        fsim = 0.0f; // 0 is the worst similarity value at this point
    }

    // invert and filter similarity between 0 and 1
    // apply sigmoid see: https://www.desmos.com/calculator/skmhf1gpyf
    // best similarity value was -1, worst was 0
    // best similarity value is 1, worst is still 0
    const float fsimInvertedFiltered = sigmoid(0.0f, 1.0f, 0.7f, -0.7f, fsim);

    // get output similarity pointer
    TSimRefine* outSimPtr = get3DBufferAt(inout_volSim_d, inout_volSim_s, inout_volSim_p, vx, vy, vz);

    // add the output similarity value
    *outSimPtr += TSimRefine(fsimInvertedFiltered);
}

__global__ void volume_retrieveBestZ_kernel(float2* out_bestDepthSimMap_d, int out_bestDepthSimMap_p,
                                            const float* in_depths_d, int in_depths_p, 
                                            const TSim* in_simVolume, int in_simVolume_s, int in_simVolume_p,
                                            int volDimZ, 
                                            int rcDeviceCamId,
                                            int scaleStep, 
                                            const Range depthRange,
                                            const ROI roi)
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if(vx >= roi.width() || vy >= roi.height())
        return;

    // corresponding device image coordinates
    const float2 pix{float((roi.x.begin + vx) * scaleStep), float((roi.y.begin + vy) * scaleStep)};

    // corresponding output depth/sim pointer
    float2* out_bestDepthSimPtr = get2DBufferAt(out_bestDepthSimMap_d, out_bestDepthSimMap_p, vx, vy);

    // find best depth
    float bestSim = 255.0f;
    int bestZIdx = -1;
    for(int vz = depthRange.begin; vz < depthRange.end; ++vz)
    {
      const float simAtZ = *get3DBufferAt(in_simVolume, in_simVolume_s, in_simVolume_p, vx, vy, vz);
      if (simAtZ < bestSim)
      {
        bestSim = simAtZ;
        bestZIdx = vz;
      }
    }

    // TODO: consider filtering out the values with a too bad score like (bestSim > 200.0f)
    //       to reduce the storage volume of the depth maps
    if (bestZIdx == -1)
    {
        out_bestDepthSimPtr->x = -1.0f; // invalid depth
        out_bestDepthSimPtr->y =  1.0f; // worst similarity value
        return;
    }

#ifdef ALICEVISION_DEPTHMAP_RETRIEVE_BEST_Z_INTERPOLATION
    // with depth/sim interpolation
    // NOTE: disable by default
    const int bestZIdx_m1 = max(0, bestZIdx - 1);
    const int bestZIdx_p1 = min(volDimZ-1, bestZIdx + 1);

    float3 depths;
    depths.x = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx_m1, 0);
    depths.y = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx, 0);
    depths.z = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx_p1, 0);

    float3 sims;
    sims.x = *get3DBufferAt(in_simVolume, in_simVolume_s, in_simVolume_p, vx, vy, bestZIdx_m1);
    sims.y = bestSim;
    sims.z = *get3DBufferAt(in_simVolume, in_simVolume_s, in_simVolume_p, vx, vy, bestZIdx_p1);

    // convert sims from (0, 255) to (-1, +1)
    sims.x = (sims.x / 255.0f) * 2.0f - 1.0f;
    sims.y = (sims.y / 255.0f) * 2.0f - 1.0f;
    sims.z = (sims.z / 255.0f) * 2.0f - 1.0f;

    // interpolation between the 3 depth planes candidates
    const float refinedDepthPlane = refineDepthSubPixel(depths, sims);

    out_bestDepthSimPtr->x = depthPlaneToDepth(rcDeviceCamId, pix, refinedDepthPlane);
    out_bestDepthSimPtr->y = sims.y;
#else
    // without depth interpolation
    const float bestDepthPlane = *get2DBufferAt(in_depths_d, in_depths_p, bestZIdx, 0);
    out_bestDepthSimPtr->x = depthPlaneToDepth(rcDeviceCamId, pix, bestDepthPlane);
    out_bestDepthSimPtr->y = (bestSim / 255.0f) * 2.0f - 1.0f; // convert from (0, 255) to (-1, +1)
    return;
#endif
}


__global__ void volume_refineBestZ_kernel(float2* out_bestDepthSimMap_d, int out_bestDepthSimMap_p,
                                          const float2* in_midDepthSimMap_d, int in_midDepthSimMap_p, 
                                          const TSimRefine* in_volSim_d, int in_volSim_s, int in_volSim_p, 
                                          int volDimZ, 
                                          int rcDeviceCamId, 
                                          int scaleStep,
                                          float samplesPerPixSize, 
                                          float twoTimesSigmaPowerTwo, 
                                          float nbSamplesHalf, 
                                          const ROI roi)
{
    const int roiX = blockIdx.x * blockDim.x + threadIdx.x;
    const int roiY = blockIdx.y * blockDim.y + threadIdx.y;

    if(roiX >= roi.width() || roiY >= roi.height())
        return;

    // corresponding volume / depth sim map coordinates
    const int vx = roiX;
    const int vy = roiY;

    // corresponding device image coordinates
    const int x = (roi.x.begin + vx) * scaleStep;
    const int y = (roi.y.begin + vy) * scaleStep;

    // corresponding original plane depth
    const float originalDepth = get2DBufferAt(in_midDepthSimMap_d, in_midDepthSimMap_p, vx, vy)->x; // input original middle depth

    // corresponding output depth/sim pointer
    float2* out_bestDepthSimPtr = get2DBufferAt(out_bestDepthSimMap_d, out_bestDepthSimMap_p, vx, vy);

    if(originalDepth < 0.0f) // original depth invalid or masked
    {
        out_bestDepthSimPtr->x = originalDepth;  // -1 (invalid) or -2 (masked)
        out_bestDepthSimPtr->y = 1.0f;           // similarity between (-1, +1)
        return;
    }

    // find best z sample per pixel
    float bestSampleSim = 99999.f;
    int bestSampleOffsetIndex = 0;

    // sliding gaussian window
    for(int sample = -nbSamplesHalf; sample <= nbSamplesHalf; ++sample)
    {
        float sampleSim = 0.f; 

        for(int vz = 0; vz < volDimZ; ++vz)
        {
            const int rz = (vz - ((volDimZ - 1) / 2)); // relative depth index offset
            const int zs = rz * samplesPerPixSize;     // relative sample offset

            // get the inversed similarity sum value
            // best value is the HIGHEST
            const float invSimSum = *get3DBufferAt(in_volSim_d, in_volSim_s, in_volSim_p, vx, vy, vz);

            // reverse the inversed similarity sum value
            // best similarity value is the LOWEST
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

    // get rc 3d point at original depth (z center)
    const float3 p = get3DPointForPixelAndDepthFromRC(rcDeviceCamId, make_int2(x, y), originalDepth);
    const float sampleSize = computePixSize(rcDeviceCamId, p) / samplesPerPixSize;
    const float sampleSizeOffset = bestSampleOffsetIndex * sampleSize;
    const float bestDepth = originalDepth + sampleSizeOffset;

    out_bestDepthSimPtr->x = bestDepth;
    out_bestDepthSimPtr->y = bestSampleSim;
}

template <typename T>
__global__ void volume_initVolumeYSlice_kernel(T* volume, int volume_s, int volume_p, const int3 volDim, const int3 axisT, int y, T cst)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if ((x >= 0) && (x < (&volDim.x)[axisT.x]) && (z >= 0) && (z < (&volDim.x)[axisT.z]))
    {
        T* volume_zyx = get3DBufferAt(volume, volume_s, volume_p, v.x, v.y, v.z);
        *volume_zyx = cst;
    }
}

template <typename T1, typename T2>
__global__ void volume_getVolumeXZSlice_kernel(T1* slice, int slice_p,
                                               const T2* volume, int volume_s, int volume_p,
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

    const T2* volume_xyz = get3DBufferAt(volume, volume_s, volume_p, v);
    T1* slice_xz = get2DBufferAt(slice, slice_p, x, z);
    *slice_xz = (T1)(*volume_xyz);
}

__global__ void volume_computeBestZInSlice_kernel(TSimAcc* xzSlice, int xzSlice_p, TSimAcc* ySliceBestInColCst, int volDimX, int volDimZ)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x >= volDimX)
        return;

    TSimAcc bestCst = *get2DBufferAt(xzSlice, xzSlice_p, x, 0);

    for(int z = 1; z < volDimZ; ++z)
    {
        const TSimAcc cst = *get2DBufferAt(xzSlice, xzSlice_p, x, z);
        bestCst = cst < bestCst ? cst : bestCst;  // min(cst, bestCst);
    }
    ySliceBestInColCst[x] = bestCst;
}

/**
 * @param[inout] xySliceForZ input similarity plane
 * @param[in] xySliceForZM1
 * @param[in] xSliceBestInColCst
 * @param[out] volSimT output similarity volume
 */
__global__ void volume_agregateCostVolumeAtXinSlices_kernel(
            cudaTextureObject_t rcTex,
            TSimAcc* xzSliceForY, int xzSliceForY_p,
            const TSimAcc* xzSliceForYm1, int xzSliceForYm1_p,
            const TSimAcc* bestSimInYm1,
            TSim* volAgr, int volAgr_s, int volAgr_p,
            const int3 volDim,
            const int3 axisT,
            float step,
            int y, float _P1, float _P2,
            int ySign, int filteringIndex,
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

    TSimAcc* sim_xz = get2DBufferAt(xzSliceForY, xzSliceForY_p, x, z);
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

          const float4 gcr0 = tex2D_float4(rcTex, float(imX0) + 0.5f, float(imY0) + 0.5f);
          const float4 gcr1 = tex2D_float4(rcTex, float(imX1) + 0.5f, float(imY1) + 0.5f);
          const float deltaC = Euclidean3(gcr0, gcr1);

          // sigmoid f(x) = i + (a - i) * (1 / ( 1 + e^(10 * (x - P2) / w)))
          // see: https://www.desmos.com/calculator/1qvampwbyx
          // best values found from tests: i = 80, a = 255, w = 80, P2 = 100
          // historical values: i = 15, a = 255, w = 80, P2 = 20
          P2 = sigmoid(80.f, 255.f, 80.f, _P2, deltaC);
        }

        const TSimAcc bestCostInColM1 = bestSimInYm1[x];
        const TSimAcc pathCostMDM1 = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z - 1); // M1: minus 1 over depths
        const TSimAcc pathCostMD   = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z);
        const TSimAcc pathCostMDP1 = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z + 1); // P1: plus 1 over depths
        const float minCost = multi_fminf(pathCostMD, pathCostMDM1 + _P1, pathCostMDP1 + _P1, bestCostInColM1 + P2);

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
    TSim* volume_xyz = get3DBufferAt(volAgr, volAgr_s, volAgr_p, v.x, v.y, v.z);
    const float val = (float(*volume_xyz) * float(filteringIndex) + pathCost) / float(filteringIndex + 1);
    *volume_xyz = TSim(val);
}

} // namespace depthMap
} // namespace aliceVision
