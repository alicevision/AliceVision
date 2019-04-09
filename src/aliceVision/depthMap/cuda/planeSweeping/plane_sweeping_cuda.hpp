// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/tcinfo.hpp>

namespace aliceVision {
namespace depthMap {

#ifdef TSIM_USE_FLOAT
    using TSim = float;
#else
    using TSim = unsigned char;
#endif

void ps_initCameraMatrix( CameraStructBase& base );

void pr_printfDeviceMemoryInfo();

void ps_initSimilarityVolume(
  CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
  CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
  int volDimX, int volDimY, int volDimZ);

void ps_initColorVolumeFromCamera(
    CudaDeviceMemoryPitched<float4, 3>& volColor_dmp,
    const CameraStruct& rcam,
    const CameraStruct& tcam,
    cudaTextureObject_t tcam_tex,
    const int tcWidth, const int tcHeight,
    const int lowestUsedDepth,
    const CudaDeviceMemory<float>& depths_d,
    int volDimX, int volDimY, int volDimZ, int volStepXY);


void ps_computeSimilarityVolume_precomputedColors(
    cudaTextureObject_t rc_tex,
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
    cudaTextureObject_t volTcamColors_tex3D,
#else
    const CudaDeviceMemoryPitched<float4, 3>& volTcamColors_dmp,
#endif
    CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
    CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
    const CameraStruct& rcam, int rcWidth, int rcHeight,
    int volStepXY, int volDimX, int volDimY,
    const OneTC& cell,
    int wsh, int kernelSizeHalf,
    int scale,
    bool verbose,
    float gammaC, float gammaP);

void ps_computeSimilarityVolume(
  Pyramids& ps_texs_arr,
  CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
  CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
  const CameraStruct& rcam, int rcWidth, int rcHeight,
  const CameraStruct& tcams, int tcWidth, int tcHeight,
  int volStepXY, int volDimX, int volDimY,
  const CudaDeviceMemory<float>& depths_dev,
  const std::vector<OneTC>&  cells,
  int wsh, int kernelSizeHalf,
  int scale,
  bool verbose,
  float gammaC, float gammaP);

void ps_SGMoptimizeSimVolume(
    Pyramids& ps_texs_arr,
    const CameraStruct& rccam,
    CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    int volDimX, int volDimY, int volDimZ,
    bool verbose,
    unsigned char P1, unsigned char P2,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated);

  void ps_SGMretrieveBestDepth(CudaDeviceMemoryPitched<float2, 2>& bestDepth_dmp, CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    int volDimX, int volDimY, int volDimZ, int zBorder);

int ps_listCUDADevices(bool verbose);

void ps_deviceAllocate(
    Pyramids& ps_texs_arr,
    int ncams,
    int width, int height,
    int scales,
    int deviceId);

void ps_testCUDAdeviceNo(int CUDAdeviceNo);

void ps_device_updateCam(
    Pyramids& ps_texs_arr,
    const CameraStruct& cam,
    int camId,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int w, int h);

void ps_deviceDeallocate(
    Pyramids& ps_texs_arr,
    int CUDAdeviceNo, int ncams, int scales);

void ps_refineRcDepthMap(
    Pyramids& ps_texs_arr,
    float* out_osimMap_hmh,
    float* inout_rcDepthMap_hmh,
    int ntcsteps,
    const std::vector<CameraStruct>& cams,
    int width, int height,
    int rcWidth, int rcHeight,
    int tcWidth, int tcHeight,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    bool verbose,
    int wsh,
    float gammaC, float gammaP,
    bool moveByTcOrRc,
    int xFrom);

void ps_fuseDepthSimMapsGaussianKernelVoting(
    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
    std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh,
    int ndepthSimMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    float sigma,
    int width, int height,
    bool verbose);

void ps_optimizeDepthSimMapGradientDescent(
    Pyramids& ps_texs_arr,
    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
    std::vector<CudaHostMemoryHeap<float2, 2>*>& dataMaps_hmh,
    int ndataMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    int nIters,
    float sigma,
    const std::vector<CameraStruct>& cams,
    int ncams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    bool verbose,
    int yFrom);

void ps_getSilhoueteMap(
    Pyramids& ps_texs_arr,
    CudaHostMemoryHeap<bool, 2>* omap_hmh,
    int width, int height,
    int scale,
    int step,
    int camId,
    uchar4 maskColorRgb,
    bool verbose);

void ps_computeNormalMap(
    Pyramids& ps_texs_arr,
    CudaHostMemoryHeap<float3, 2>& normalMap_hmh,
    CudaHostMemoryHeap<float, 2>& depthMap_hmh,
    const CameraStruct& camera, int width, int height,
    int scale, int ncamsAllocated, int scales, int wsh, bool verbose,
    float gammaC, float gammaP);

} // namespace depthMap
} // namespace aliceVision

