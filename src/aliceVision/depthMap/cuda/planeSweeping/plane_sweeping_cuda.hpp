// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

float3 ps_getDeviceMemoryInfo();

float ps_planeSweepingGPUPixelsVolume(
    CudaArray<uchar4, 2>** ps_texs_arr,
    unsigned char* ovol_hmh,
    cameraStruct** cams,
    int ncams,
    int width, int height,
    int volStepXY, int volDimX, int volDimY,
    int volDimZ,
    int volLUX,
    int volLUY,
    int volLUZ,
    CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
    CudaHostMemoryHeap<float, 2>& depths_hmh,
    int nDepthsToSearch,
    int slicesAtTime,
    int ntimes,
    int npixs,
    int wsh,
    int kernelSizeHalf,
    int nPlanes,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    bool verbose,
    bool doUsePixelsDepths,
    int nbest,
    bool useTcOrRcPixSize,
    float gammaC, float gammaP,
    bool subPixel,
    float epipShift);

void ps_SGMoptimizeSimVolume(
    CudaArray<uchar4, 2>** ps_texs_arr,
    cameraStruct* rccam,
    unsigned char* iovol_hmh,
    int volDimX, int volDimY, int volDimZ,
    int volStepXY,
    int volLUX,
    int volLUY,
    bool verbose,
    unsigned char P1, unsigned char P2,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales);



int ps_listCUDADevices(bool verbose);

void ps_deviceAllocate(
    CudaArray<uchar4, 2>*** ps_texs_arr,
    int ncams,
    int width, int height,
    int scales,
    int deviceId);

void ps_deviceUpdateCam(
    CudaArray<uchar4, 2>** ps_texs_arr,
    cameraStruct* cam,
    int camId,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int w, int h,
    int varianceWsh);

void ps_deviceDeallocate(
    CudaArray<uchar4, 2>*** ps_texs_arr,
    int CUDAdeviceNo,
    int ncams,
    int scales);

void ps_refineRcDepthMap(
    CudaArray<uchar4, 2>** ps_texs_arr,
    float* osimMap_hmh,
    float* rcDepthMap_hmh,
    int ntcsteps,
    cameraStruct** cams,
    int ncams,
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
    CudaHostMemoryHeap<float2, 2>** depthSimMaps_hmh,
    int ndepthSimMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    float sigma,
    int width, int height,
    bool verbose);

void ps_optimizeDepthSimMapGradientDescent(
    CudaArray<uchar4, 2>** ps_texs_arr,
    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
    CudaHostMemoryHeap<float2, 2>** dataMaps_hmh,
    int ndataMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    int nIters,
    float sigma,
    cameraStruct** cams,
    int ncams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    bool verbose,
    int yFrom);

void ps_getSilhoueteMap(
    CudaArray<uchar4, 2>** ps_texs_arr,
    CudaHostMemoryHeap<bool, 2>* omap_hmh,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int step,
    int camId,
    uchar4 maskColorRgb,
    bool verbose);

void ps_computeNormalMap(
  CudaArray<uchar4, 2>** ps_texs_arr,
  CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
  CudaHostMemoryHeap<float, 2>* depthMap_hmh,
  const cameraStruct& camera,
  int width, int height,
  int scale,
  int CUDAdeviceNo,
  int ncamsAllocated,
  int scales,
  int wsh,
  bool verbose,
  float gammaC, float gammaP);

#if 0
void ps_smoothDepthMap(
    CudaArray<uchar4, 2>** ps_texs_arr,
    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
    cameraStruct** cams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int wsh,
    bool verbose,
    float gammaC, float gammaP);

void ps_filterDepthMap(
    CudaArray<uchar4, 2>** ps_texs_arr,
    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
    cameraStruct** cams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int wsh,
    bool verbose,
    float gammaC,
    float minCostThr);

void ps_alignSourceDepthMapToTarget(
    CudaArray<uchar4, 2>** ps_texs_arr,
    CudaHostMemoryHeap<float, 2>* sourceDepthMap_hmh,
    CudaHostMemoryHeap<float, 2>* targetDepthMap_hmh,
    cameraStruct** cams,
    int width, int height,
    int scale,
    int CUDAdeviceNo,
    int ncamsAllocated,
    int scales,
    int wsh,
    bool verbose,
    float gammaC,
    float maxPixelSizeDist);

#endif

} // namespace depthMap
} // namespace aliceVision

