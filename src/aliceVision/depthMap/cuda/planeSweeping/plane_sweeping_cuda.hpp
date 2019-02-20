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

void ps_initCameraMatrix( cameraStructBase& base );

void pr_printfDeviceMemoryInfo();

void ps_computeSimilarityVolume(
  Pyramid& ps_texs_arr,
  std::vector<CudaDeviceMemoryPitched<float, 3>*> vol_dmp,
  const cameraStruct& rcam,
  const std::vector<cameraStruct>& tcams,
  int stepLessWidth, int stepLessHeight,
  int volStepXY,
  int volDimX, int volDimY,
  const int zDimsAtATime,
  CudaDeviceMemory<float>& depths_dev,
  std::vector<OneTC>&  tcs,
  int wsh, int kernelSizeHalf,
  int scale,
  int scales, bool verbose, bool doUsePixelsDepths,
  int nbest,
  float gammaC, float gammaP, bool subPixel,
  float epipShift);

void ps_SGMoptimizeSimVolume(
    Pyramid& ps_texs_arr,
    const cameraStruct& rccam,
    unsigned char* iovol_hmh,
    int volDimX, int volDimY, int volDimZ,
    int volStepXY,
    bool verbose,
    unsigned char P1, unsigned char P2,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales);

int ps_listCUDADevices(bool verbose);

void ps_deviceAllocate(
    Pyramid& ps_texs_arr,
    int ncams,
    int width, int height,
    int scales,
    int deviceId);

void ps_testCUDAdeviceNo(int CUDAdeviceNo);

void ps_deviceUpdateCam(
    Pyramid& ps_texs_arr,
    const cameraStruct& cam,
    int camId,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int w, int h,
    int varianceWsh);

void ps_deviceDeallocate(
    Pyramid& ps_texs_arr,
    int CUDAdeviceNo, int ncams, int scales);

void ps_refineRcDepthMap(
    Pyramid& ps_texs_arr,
    float* osimMap_hmh,
    float* rcDepthMap_hmh,
    int ntcsteps,
    const std::vector<cameraStruct>& cams,
    int width, int height,
    int imWidth, int imHeight,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    bool verbose,
    int wsh,
    float gammaC, float gammaP,
    float epipShift,
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
    Pyramid& ps_texs_arr,
    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
    std::vector<CudaHostMemoryHeap<float2, 2>*>& dataMaps_hmh,
    int ndataMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    int nIters,
    float sigma,
    const std::vector<cameraStruct>& cams,
    int ncams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    bool verbose,
    int yFrom);

void ps_getSilhoueteMap(
    Pyramid& ps_texs_arr,
    CudaHostMemoryHeap<bool, 2>* omap_hmh,
    int width, int height,
    int scale,
    int step,
    int camId,
    uchar4 maskColorRgb,
    bool verbose);

} // namespace depthMap
} // namespace aliceVision

