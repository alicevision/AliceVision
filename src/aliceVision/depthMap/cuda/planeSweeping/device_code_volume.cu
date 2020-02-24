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
#else
using TSim = unsigned char;
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
    fsim = (fsim - fminVal) * fmultiplier;
#ifdef TSIM_USE_FLOAT
    // no clamp
#else
    fsim = fminf(1.0f, fmaxf(0.0f, fsim));
#endif
    fsim *= 255.0f; // needed to store in the volume in uchar

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
  int cam_cache_idx,
  float* bestDepthM, int bestDepthM_s,
  float* bestSimM, int bestSimM_s,
  const float* depths_d,
  const TSim* simVolume, int simVolume_s, int simVolume_p,
  int volDimX, int volDimY, int volDimZ,
  int scaleStep,
  bool interpolate)
{
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;
  
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

  float2 pix{float(x * scaleStep), float(y * scaleStep)};
  // Without depth interpolation (for debug purpose only)
  if(!interpolate)
  {
    *get2DBufferAt(bestDepthM, bestDepthM_s, x, y) = depthPlaneToDepth(cam_cache_idx, pix, depths_d[bestZIdx]);
    *get2DBufferAt(bestSimM, bestSimM_s, x, y) = bestSim / 255.0f;
    return;
  }

  // With depth/sim interpolation
  int bestZIdx_m1 = max(0, bestZIdx - 1);
  int bestZIdx_p1 = min(volDimZ-1, bestZIdx + 1);
  float3 depths;
  depths.x = depths_d[bestZIdx_m1];
  depths.y = depths_d[bestZIdx];
  depths.z = depths_d[bestZIdx_p1];
  float3 sims;
  sims.x = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, bestZIdx_m1) / 255.0f;
  sims.y = bestSim / 255.0f;
  sims.z = *get3DBufferAt(simVolume, simVolume_s, simVolume_p, x, y, bestZIdx_p1) / 255.0f;

  // Interpolation between the 3 depth planes candidates
  const float refinedDepth = refineDepthSubPixel(depths, sims);

  *get2DBufferAt(bestDepthM, bestDepthM_s, x, y) = depthPlaneToDepth(cam_cache_idx, pix, refinedDepth);
  *get2DBufferAt(bestSimM, bestSimM_s, x, y) = sims.y;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
__global__ void volume_initVolumeYSlice_kernel(T* volume, int volume_s, int volume_p, const int3 volDim, const int3 axisT, int y, T cst)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int z = blockIdx.y * blockDim.y + threadIdx.y;

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
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int z = blockIdx.y * blockDim.y + threadIdx.y;

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

__global__ void volume_computeBestZInSlice_kernel(TSim* xzSlice, int xzSlice_p, TSim* ySliceBestInColCst, int volDimX, int volDimZ)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;

    if(x >= volDimX)
        return;

    float bestCst = *get2DBufferAt(xzSlice, xzSlice_p, x, 0);

    for(int z = 1; z < volDimZ; ++z)
    {
        float cst = *get2DBufferAt(xzSlice, xzSlice_p, x, z);
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
            TSim* xzSliceForY, int xzSliceForY_p,
            const TSim* xzSliceForYm1, int xzSliceForYm1_p,
            const TSim* bestSimInYm1,
            TSim* volAgr, int volAgr_s, int volAgr_p,
            const int3 volDim,
            const int3 axisT,
            int y, float _P1, float _P2,
            int ySign, int filteringIndex)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int z = blockIdx.y * blockDim.y + threadIdx.y;

    int3 v;
    (&v.x)[axisT.x] = x;
    (&v.x)[axisT.y] = y;
    (&v.x)[axisT.z] = z;

    if (x >= (&volDim.x)[axisT.x] || z >= volDim.z)
        return;

    TSim* sim_xz = get2DBufferAt(xzSliceForY, xzSliceForY_p, x, z);
    float pathCost = 255.0f;

    if((z >= 1) && (z < volDim.z - 1))
    {
        const int imX0 = v.x; // current
        const int imY0 = v.y;

        const int imX1 = imX0 - ySign * (axisT.y == 0); // M1
        const int imY1 = imY0 - ySign * (axisT.y == 1);

        const float4 gcr0 = tex2D_float4(rc_tex, float(imX0) + 0.5f, float(imY0) + 0.5f);
        const float4 gcr1 = tex2D_float4(rc_tex, float(imX1) + 0.5f, float(imY1) + 0.5f);

        const float deltaC = Euclidean3(gcr0, gcr1);
        // unsigned int P1 = (unsigned int)sigmoid(5.0f,20.0f,60.0f,10.0f,deltaC);
        float P1 = _P1;
        // 15.0 + (255.0 - 15.0) * (1.0 / (1.0 + exp(10.0 * ((x - 20.) / 80.))))
        float P2 = 0;
        // _P2 convention: use negative value to skip the use of deltaC
        if(_P2 >= 0)
            P2 = sigmoid(15.0f, 255.0f, 80.0f, _P2, deltaC);
        else
            P2 = std::abs(_P2);

        const float bestCostInColM1 = bestSimInYm1[x];
        const float pathCostMDM1 = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z - 1); // M1: minus 1 over depths
        const float pathCostMD   = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z);
        const float pathCostMDP1 = *get2DBufferAt(xzSliceForYm1, xzSliceForYm1_p, x, z + 1); // P1: plus 1 over depths
        const float minCost = multi_fminf(pathCostMD, pathCostMDM1 + P1, pathCostMDP1 + P1, bestCostInColM1 + P2);

        // if 'pathCostMD' is the minimal value of the depth
        pathCost = (*sim_xz) + minCost - bestCostInColM1;
    }

#ifndef TSIM_USE_FLOAT
    // clamp if we use uchar
    pathCost = fminf(255.0f, fmaxf(0.0f, pathCost));
#endif

    // fill the current slice with the new similarity score
    *sim_xz = pathCost;

    // aggregate into the final output
    TSim* volume_xyz = get3DBufferAt(volAgr, volAgr_s, volAgr_p, v.x, v.y, v.z);
    const float val = (float(*volume_xyz) * float(filteringIndex) + pathCost) / (float)(filteringIndex + 1);
    *volume_xyz = (unsigned char)(val);
}

} // namespace depthMap
} // namespace aliceVision
