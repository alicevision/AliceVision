// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/deviceCommon/device_matrix.cu>


namespace aliceVision {
namespace depthMap {

#ifdef TSIM_USE_FLOAT
using TSim = float;
using TSimAcc = float;
#else
using TSim = unsigned char;
using TSimAcc = unsigned int; // TSimAcc is the similarity accumulation type
#endif


inline __device__ void volume_computePatch( int rc_cam_cache_idx,
                                            int tc_cam_cache_idx,
                                            Patch& ptch,
                                            const float fpPlaneDepth, const int2& pix )
{
    ptch.p = get3DPointForPixelAndFrontoParellePlaneRC(rc_cam_cache_idx, pix, fpPlaneDepth); // no texture use
    ptch.d = computePixSize(rc_cam_cache_idx, ptch.p); // no texture use
    computeRotCSEpip(rc_cam_cache_idx, tc_cam_cache_idx, ptch); // no texture use
}

__global__ void volume_init_kernel(TSim* volume, int volume_s, int volume_p,
                                    int volDimX, int volDimY )
{
    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z; // * blockDim.z + threadIdx.z;

    if(vx >= volDimX || vy >= volDimY)
        return;

    *get3DBufferAt(volume, volume_s, volume_p, vx, vy, vz) = 255.0f;
}

__global__ void volume_slice_kernel(
                                    cudaTextureObject_t rc_tex,
                                    cudaTextureObject_t tc_tex,
                                    int rc_cam_cache_idx,
                                    int tc_cam_cache_idx,
                                    const float* depths_d,
                                    const int startDepthIndex,
                                    const int nbDepthsToSearch,
                                    int rcWidth, int rcHeight,
                                    int tcWidth, int tcHeight,
                                    int wsh,
                                    const float gammaC, const float gammaP,
                                    TSim* volume_1st, int volume1st_s, int volume1st_p,
                                    TSim* volume_2nd, int volume2nd_s, int volume2nd_p,
                                    int volStepXY,
                                    int volDimX, int volDimY)
{
    /*
     * Note !
     * volDimX == width  / volStepXY
     * volDimY == height / volStepXY
     * width and height are needed to compute transformations,
     * volDimX and volDimY may be the number of samples, reducing memory or computation
     */

    const int vx = blockIdx.x * blockDim.x + threadIdx.x;
    const int vy = blockIdx.y * blockDim.y + threadIdx.y;
    const int vz = blockIdx.z; // * blockDim.z + threadIdx.z;

    if( vx >= volDimX || vy >= volDimY ) // || vz >= volDimZ
        return;
    // if (vz >= nbDepthsToSearch)
    //  return;
    assert(vz < nbDepthsToSearch);

    const int x = vx * volStepXY;
    const int y = vy * volStepXY;

    // if(x >= rcWidth || y >= rcHeight)
    //     return;

    const int zIndex = startDepthIndex + vz;
    const float fpPlaneDepth = depths_d[zIndex];

    Patch ptcho;
    volume_computePatch( rc_cam_cache_idx,
                         tc_cam_cache_idx,
                         ptcho, fpPlaneDepth, make_int2(x, y)); // no texture use

    float fsim = compNCCby3DptsYK(rc_tex, tc_tex,
                                  rc_cam_cache_idx, tc_cam_cache_idx,
                                  ptcho, wsh,
                                  rcWidth, rcHeight,
                                  tcWidth, tcHeight,
                                  gammaC, gammaP);

    constexpr const float fminVal = -1.0f;
    constexpr const float fmaxVal = 1.0f;
    constexpr const float fmultiplier = 1.0f / (fmaxVal - fminVal);

    if(fsim == CUDART_INF_F) // invalid similarity
    {
      fsim = 255.0f;
    }
    else // valid similarity
    {
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

    TSim* fsim_1st = get3DBufferAt(volume_1st, volume1st_s, volume1st_p, vx, vy, zIndex);
    TSim* fsim_2nd = get3DBufferAt(volume_2nd, volume2nd_s, volume2nd_p, vx, vy, zIndex);

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

__device__ float depthPlaneToDepth(
    int cam_cache_idx,
    const float2& pix,
    float fpPlaneDepth)
{
    const CameraStructBase& cam = camsBasesDev[cam_cache_idx];
    float3 planen = M3x3mulV3(cam.iR, make_float3(0.0f, 0.0f, 1.0f));
    normalize(planen);
    float3 planep = cam.C + planen * fpPlaneDepth;
    float3 v = M3x3mulV2(cam.iP, pix);
    normalize(v);
    float3 p = linePlaneIntersect(cam.C, v, planep, planen);
    float depth = size(cam.C - p);
    return depth;
}


__global__ void volume_retrieveBestZ_kernel(
  int rcamCacheId,
  float* bestDepthM, int bestDepthM_s,
  float* bestSimM, int bestSimM_s,
  const TSim* simVolume, int simVolume_s, int simVolume_p,
  int volDimX, int volDimY, int volDimZ,
  const float* depths_d,
  int scaleStep, bool interpolate)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  
  if(x >= volDimX || y >= volDimY)
    return;

  float bestSim = 255.0f;
  int bestZIdx = -1;
  for (int z = 0; z < volDimZ; ++z)
  {
    const float simAtZ = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, z);
    if (simAtZ < bestSim)
    {
      bestSim = simAtZ;
      bestZIdx = z;
    }
  }

  // TODO: consider filtering out the values with a too bad score like (bestSim > 200.0f)
  //       to reduce the storage volume of the depth maps
  if (bestZIdx == -1)
  {
      *get2DBufferAt(bestDepthM, bestDepthM_s, x, y) = -1.0f;
      *get2DBufferAt(bestSimM, bestSimM_s, x, y) = 1.0f;
      return;
  }

  const float2 pix{float(x * scaleStep), float(y * scaleStep)};
  // Without depth interpolation (for debug purpose only)
  if(!interpolate)
  {
    *get2DBufferAt(bestDepthM, bestDepthM_s, x, y) = depthPlaneToDepth(rcamCacheId, pix, depths_d[bestZIdx]);
    *get2DBufferAt(bestSimM, bestSimM_s, x, y) = (bestSim / 255.0f) * 2.0f - 1.0f; // convert from (0, 255) to (-1, +1)
    return;
  }

  // With depth/sim interpolation
  const int bestZIdx_m1 = max(0, bestZIdx - 1);
  const int bestZIdx_p1 = min(volDimZ-1, bestZIdx + 1);

  float3 depths;
  depths.x = depths_d[bestZIdx_m1];
  depths.y = depths_d[bestZIdx];
  depths.z = depths_d[bestZIdx_p1];

  float3 sims;
  sims.x = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, bestZIdx_m1);
  sims.y = bestSim;
  sims.z = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, bestZIdx_p1);

  // Convert sims from (0, 255) to (-1, +1)
  sims.x = (sims.x / 255.0f) * 2.0f - 1.0f;
  sims.y = (sims.y / 255.0f) * 2.0f - 1.0f;
  sims.z = (sims.z / 255.0f) * 2.0f - 1.0f;

  // Interpolation between the 3 depth planes candidates
  const float refinedDepth = refineDepthSubPixel(depths, sims);

  *get2DBufferAt(bestDepthM, bestDepthM_s, x, y) = depthPlaneToDepth(rcamCacheId, pix, refinedDepth);
  *get2DBufferAt(bestSimM, bestSimM_s, x, y) = sims.y;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
            cudaTextureObject_t rc_tex,
            TSimAcc* xzSliceForY, int xzSliceForY_p,
            const TSimAcc* xzSliceForYm1, int xzSliceForYm1_p,
            const TSimAcc* bestSimInYm1,
            TSim* volAgr, int volAgr_s, int volAgr_p,
            const int3 volDim,
            const int3 axisT,
            float step,
            int y, float _P1, float _P2,
            int ySign, int filteringIndex)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if (x >= (&volDim.x)[axisT.x] || z >= volDim.z)
        return;

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
          const int imX0 = v.x * step; // current
          const int imY0 = v.y * step;

          const int imX1 = imX0 - ySign * step * (axisT.y == 0); // M1
          const int imY1 = imY0 - ySign * step * (axisT.y == 1);

          const float4 gcr0 = tex2D_float4(rc_tex, float(imX0) + 0.5f, float(imY0) + 0.5f);
          const float4 gcr1 = tex2D_float4(rc_tex, float(imX1) + 0.5f, float(imY1) + 0.5f);
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
