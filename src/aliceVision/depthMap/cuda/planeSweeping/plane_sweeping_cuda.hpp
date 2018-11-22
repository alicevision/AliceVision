// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/cuda/commonStructures.hpp>

namespace aliceVision {
namespace depthMap {

void ps_initCameraMatrix( cameraStructBase& base );

float3 ps_getDeviceMemoryInfo();

void ps_planeSweepingGPUPixelsVolume(
    Pyramid& ps_texs_arr,
    const int max_ct,
    float* volume_out,
    const int volume_offset,
    std::vector<CudaDeviceMemoryPitched<float, 3>*>& volSim_dmp,
    const cameraStruct& rcam,
    const std::vector<cameraStruct>& tcams,
    int width, int height,
    int volStepXY, int volDimX, int volDimY,
    const int zDimsAtATime,
    std::vector<CudaDeviceMemory<float>*> depths_dev,
    const std::vector<int>& depths_to_search,
    int wsh,
    int kernelSizeHalf,
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

float3 ps_getDeviceMemoryInfo();

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
    CudaHostMemoryHeap<float2, 2>** depthSimMaps_hmh,
    int ndepthSimMaps,
    int nSamplesHalf,
    int nDepthsToRefine,
    float sigma,
    int width, int height,
    bool verbose);

void ps_optimizeDepthSimMapGradientDescent(
    Pyramid& ps_texs_arr,
    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
    CudaHostMemoryHeap<float2, 2>** dataMaps_hmh,
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

