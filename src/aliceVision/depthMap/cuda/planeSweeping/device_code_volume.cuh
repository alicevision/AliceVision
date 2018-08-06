// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_utils.cuh"

namespace aliceVision {
namespace depthMap {


__global__ void volume_slice_kernel(
    NormLinearTex<uchar4> r4tex,
    NormLinearTex<uchar4> t4tex,
    cudaTextureObject_t depthsTex,
    cudaTextureObject_t volPixs_x_Tex,
    cudaTextureObject_t volPixs_y_Tex,
    cudaTextureObject_t volPixs_z_Tex,
    unsigned char* slice, int slice_p,
    // float3* slicePts, int slicePts_p,
    int nsearchdepths, int ndepths, int slicesAtTime, int width, int height, int wsh,
    int t, int npixs, const float gammaC, const float gammaP, const float epipShift );

__global__ void volume_saveSliceToVolume_kernel(
    cudaTextureObject_t volPixsTex_x,
    cudaTextureObject_t volPixsTex_y,
    cudaTextureObject_t volPixsTex_z,
    unsigned char* volume, int volume_s, int volume_p, unsigned char* slice,
    int slice_p, int nsearchdepths, int ndepths, int slicesAtTime,
    int width, int height, int t, int npixs, int volStepXY, int volDimX,
    int volDimY, int volDimZ, int volLUX, int volLUY, int volLUZ );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void volume_transposeAddAvgVolume_kernel(unsigned char* volumeT, int volumeT_s, int volumeT_p,
                                                    const unsigned char* volume, int volume_s, int volume_p, int volDimX,
                                                    int volDimY, int volDimZ, int dimTrnX, int dimTrnY, int dimTrnZ,
                                                    int z, int lastN);

template <typename T>
__global__ void volume_transposeVolume_kernel(T* volumeT, int volumeT_s, int volumeT_p, 
                                              const T* volume, int volume_s, int volume_p, 
                                              int volDimX, int volDimY, int volDimZ, 
                                              int dimTrnX, int dimTrnY, int dimTrnZ, 
                                              int z)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    int vz = z;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;
    if( vz >= volDimZ ) return;

    int v[3];
    v[0] = vx;
    v[1] = vy;
    v[2] = vz;

#if 0
    int dimsTrn[3];
    dimsTrn[0] = dimTrnX;
    dimsTrn[1] = dimTrnY;
    dimsTrn[2] = dimTrnZ;

    int vTx = v[dimsTrn[0]];
    int vTy = v[dimsTrn[1]];
    int vTz = v[dimsTrn[2]];
#else
    int vTx = v[dimTrnX];
    int vTy = v[dimTrnY];
    int vTz = v[dimTrnZ];
#endif

    T newVal = Block<const T>(volume, volume_s, volume_p).get(vx, vy, vz);
    Block<T>(volumeT, volumeT_s, volumeT_p).set( vTx, vTy, vTz, newVal );
}

template <typename T>
__global__ void volume_shiftZVolumeTempl_kernel(T* volume, int volume_s, int volume_p, int volDimX, int volDimY,
                                                int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;
    if( vz >= volDimZ ) return;

    Block<T> block( volume, volume_s, volume_p );
    T v1 = block.get( vx, vy, vz );
    T v2 = block.get( vx, vy, volDimZ - 1 - vz );
    block.set( vx, vy, vz,               v2 );
    block.set( vx, vy, volDimZ - 1 - vz, v1 );
}

template <typename T>
__global__ void volume_initVolume_kernel(T* volume, int volume_s, int volume_p, int volDimX, int volDimY, int volDimZ,
                                         int vz, T cst)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;
    if( vz >= volDimZ ) return;

    Block<T>(volume, volume_s, volume_p).set( vx, vy, vz, cst );
}

template <typename T>
__global__ void volume_initFullVolume_kernel(T* volume, int volume_s, int volume_p, int volDimX, int volDimY, int volDimZ,
    T cst)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;
    int vz = blockIdx.z * blockDim.z + threadIdx.z;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;
    if( vz >= volDimZ ) return;

    Block<T>(volume, volume_s, volume_p).set( vx, vy, vz, cst );
}

__global__ void volume_updateMinXSlice_kernel(unsigned char* volume, int volume_s, int volume_p,
                                              unsigned char* xySliceBestSim, int xySliceBestSim_p, int* xySliceBestZ,
                                              int xySliceBestZ_p, int volDimX, int volDimY, int volDimZ, int vz);

template <typename T1, typename T2>
__global__ void volume_getVolumeXYSliceAtZ_kernel(T1* xySlice, int xySlice_p, T2* volume, int volume_s, int volume_p,
                                                  int volDimX, int volDimY, int volDimZ, int vz)
{
    int vx = blockIdx.x * blockDim.x + threadIdx.x;
    int vy = blockIdx.y * blockDim.y + threadIdx.y;

    if( vx >= volDimX ) return;
    if( vy >= volDimY ) return;
    if( vz >= volDimZ ) return;

    T2 volume_zyx = Block<T2>(volume, volume_s, volume_p).get( vx, vy, vz );

    Plane<T1>(xySlice, xySlice_p).set(vx, vy, (T1)volume_zyx );
}

#if 0
// __global__ void volume_computeBestXSlice_kernel( cudaTextureObject_t sliceTexUChar,
//                                                  unsigned char* xsliceBestInColCst,
//                                                  int volDimX, int volDimY );
// 
// __global__ void volume_agregateCostVolumeAtZ_kernel(unsigned char* volume, int volume_s, int volume_p,
//                                                     unsigned char* xsliceBestInColCst, int volDimX, int volDimY,
//                                                     int volDimZ, int vz, unsigned char P1, unsigned char P2,
//                                                     bool transfer);
#endif

__global__ void volume_computeBestXSliceUInt_kernel(
    cudaTextureObject_t sliceTexUInt,
    unsigned int* xsliceBestInColCst, int volDimX, int volDimY );

/**
 * @param[inout] xySliceForZ input similarity plane
 * @param[in] xySliceForZM1
 * @param[in] xSliceBestInColCst
 * @param[out] volSimT output similarity volume
 */
__global__ void volume_agregateCostVolumeAtZinSlices_kernel(
                    NormLinearTex<uchar4> r4tex,
                    unsigned int* xySliceForZ, int xySliceForZ_p,
                    const unsigned int* xySliceForZM1, int xySliceForZM1_p,
                    const unsigned int* xSliceBestInColSimForZM1,
                    unsigned char* volSimT, int volSimT_s, int volSimT_p, 
                    int volDimX, int volDimY, int volDimZ, 
                    int vz, unsigned int _P1, unsigned int _P2,
                    bool transfer, int volLUX, int volLUY,
                    int dimTrnX, bool doInvZ);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_updateRcVolumeForTcDepthMap_kernel(
//     cudaTextureObject_t sliceTexFloat2,
//     unsigned int* volume, int volume_s, int volume_p,
//     const int volDimX, const int volDimY, const int volDimZ,
//     const int vz, const int volStepXY, const int tcDepthMapStep,
//     const int width, const int height, const float fpPlaneDepth,
//     const float stepInDepth, const int zPart,
//     const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
//     const bool considerNegativeDepthAsInfinity,
//     const float2 tcMinMaxFpDepth );
#endif

#if 0
// __global__ void volume_updateRcVolumeForTcDepthMap2_kernel(
//     cudaTextureObject_t sliceTexFloat2,
//     unsigned int* volume, int volume_s, int volume_p,
//     const int volDimX, const int volDimY, const int volDimZ,
//     const int vz, const int volStepXY, const int tcDepthMapStep,
//     const int width, const int height, const float fpPlaneDepth,
//     const float stepInDepth, const int zPart,
//     const int vilDimZGlob, const float maxTcRcPixSizeInVoxRatio,
//     const bool considerNegativeDepthAsInfinity,
//     const float2 tcMinMaxFpDepth, const bool useSimilarity );
#endif

#if 0
// __global__ void volume_update_nModalsMap_kernel_id0(
//     unsigned short* nModalsMap, int nModalsMap_p,
//     int volDimX, int volDimY );
#endif

#if 0
// __global__ void volume_update_nModalsMap_kernel(
//     cudaTextureObject_t depthsTex,
//     cudaTextureObject_t sliceTex,
//     unsigned short* nModalsMap, int nModalsMap_p,
//     unsigned short* rcIdDepthMap, int rcIdDepthMap_p, int volDimX,
//     int volDimY, int volDimZ, int volStepXY, int tcDepthMapStep, int width,
//     int height, int distLimit, int id );
#endif

#if 0
// __global__ void volume_filterRcIdDepthMapByTcDepthMap_kernel(
//     cudaTextureObject_t depthsTex,
//     cudaTextureObject_t sliceTex,
//     unsigned short* rcIdDepthMap, int rcIdDepthMap_p,
//     int volDimX, int volDimY, int volDimZ, int volStepXY,
//     int tcDepthMapStep, int width, int height, int distLimit );
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

__global__ void update_GC_volumeXYSliceAtZInt4_kernel(int* xySlice_x, int xySlice_x_p,
	                                              int* xySlice_y, int xySlice_y_p,
						      int* xySlice_z, int xySlice_z_p,
						      int* xySlice_w, int xySlice_w_p,
						      unsigned int* volume, int volume_s,
                                                      int volume_p, int volDimX, int volDimY, int volDimZ, int vz);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel(unsigned int* oxySlice, int oxySlice_p,
//                                                                   int* ixySlice_z, int ixySlice_z_p,
// 								  int volDimX, int volDimY);
#endif

#if 0
// __global__ void volume_GC_K_initXYSliceInt4_kernel(int* xySlice_x, int xySlice_x_p,
//                                                    int* xySlice_y, int xySlice_y_p,
//                                                    int* xySlice_z, int xySlice_z_p,
//                                                    int* xySlice_w, int xySlice_w_p,
// 						   unsigned int* volume, int volume_s,
//                                                    int volume_p, int volDimX, int volDimY, int vz);
#endif

#if 0
// __global__ void update_GC_K_volumeXYSliceAtZInt4_kernel(int* xySlice_x, int xySlice_x_p,
//                                                         int* xySlice_y, int xySlice_y_p,
//                                                         int* xySlice_z, int xySlice_z_p,
//                                                         int* xySlice_w, int xySlice_w_p,
//                                                         unsigned int* volume,
//                                                         int volume_s, int volume_p, int volDimX, int volDimY,
//                                                         int volDimZ, int vz, int K);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_compute_rDP1_kernel(int2* xySlice, int xySlice_p, int* ovolume, int ovolume_s, int ovolume_p,
//                                            unsigned int* volume, int volume_s, int volume_p, int volDimX, int volDimY,
//                                            int volDimZ, int vz);
#endif

__global__ void volume_compute_DP1_kernel(int2* xySlice, int xySlice_p, int* ovolume, int ovolume_s, int ovolume_p,
                                          unsigned int* volume, int volume_s, int volume_p, int volDimX, int volDimY,
                                          int volDimZ, int vz);

#if 0
// __global__ void volume_compute_rDP1_volume_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume, int volume_s,
//                                                             int volume_p, int volDimX, int volDimY, int volDimZ, int z,
//                                                             int zPart, int volDimZGlob);
#endif

#if 0
// __global__ void volume_normalize_rDP1_volume_by_minMaxMap_kernel(int2* xySlice, int xySlice_p, int* volume,
//                                                                  int volume_s, int volume_p, int volDimX, int volDimY,
//                                                                  int volDimZ, int z, int zPart, int volDimZGlob);
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_filter_VisTVolume_kernel(
//     cudaTextureObject_t sliceTexUInt,
//     unsigned int* ovolume, int ovolume_s, int ovolume_p,
//     int volDimX, int volDimY, int volDimZ,
//     int vz, int K );
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
// __global__ void volume_filter_enforceTWeightInVolume_kernel(
//     cudaTextureObject_t sliceTexUInt,
//     unsigned int* ovolume, int ovolume_s, int ovolume_p,
//     int volDimX, int volDimY, int volDimZ,
//     int vz, int K );
#endif

} // namespace depthMap
} // namespace aliceVision
