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
    const CameraStruct& rccam,
    const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    CudaDeviceMemoryPitched<TSim, 3>& volSimFiltered_dmp,
    int volDimX, int volDimY, int volDimZ,
    const std::string& filteringAxes,
    bool verbose,
    float P1, float P2,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated);

  void ps_SGMretrieveBestDepth(
    CudaDeviceMemoryPitched<float, 2>& bestDepth_dmp, CudaDeviceMemoryPitched<float, 2>& bestSim_dmp,
    const CameraStruct& rccam,
    const CudaDeviceMemory<float>& depths_d,
    CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    int volDimX, int volDimY, int volDimZ, int scaleStep, bool interpolate);

int ps_listCUDADevices(bool verbose);

void ps_deviceAllocate(
    Pyramids& ps_texs_arr,
    int ncams,
    int width, int height,
    int scales,
    int deviceId);

void ps_testCUDAdeviceNo(int CUDAdeviceNo);

void ps_device_updateCam(
    const CameraStruct& cam,
    int CUDAdeviceNo,
    int scales,
    int w, int h);

void ps_deviceDeallocate(
    Pyramids& ps_texs_arr,
    int CUDAdeviceNo, int ncams, int scales);

void ps_refineRcDepthMap(
    float* out_osimMap_hmh,
    float* inout_rcDepthMap_hmh,
    int ntcsteps,
    CameraStruct& rc_cam,
    CameraStruct& tc_cam,
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
    CudaHostMemoryHeap<float2, 2>& out_depthSimMap_hmh,
    const CudaHostMemoryHeap<float2, 2>& sgmDepthSimMap_hmh,
    const CudaHostMemoryHeap<float2, 2>& refinedDepthSimMap_hmh,
    int nSamplesHalf, int nDepthsToRefine, int nIters, float sigma,
    CameraStruct& rc_cam,
    int width, int height, int scale,
    int CUDAdeviceNo, int ncamsAllocated, bool verbose, int yFrom);

void ps_getSilhoueteMap(
    CudaHostMemoryHeap<bool, 2>* omap_hmh,
    int width, int height,
    int scale,
    int step,
    CameraStruct& cam,
    uchar4 maskColorRgb,
    bool verbose);

void ps_computeNormalMap(
    CudaHostMemoryHeap<float3, 2>& normalMap_hmh,
    CudaHostMemoryHeap<float, 2>& depthMap_hmh,
    const CameraStruct& camera, int width, int height,
    int scale, int ncamsAllocated, int scales, int wsh, bool verbose,
    float gammaC, float gammaP);

void ps_loadCameraStructs( CameraStructBase*       dev,
                           const CameraStructBase* hst,
                           int                     offset );

} // namespace depthMap
} // namespace aliceVision

