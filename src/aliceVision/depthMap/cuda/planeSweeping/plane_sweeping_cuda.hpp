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


namespace ps
{
class SimilarityVolume
{
public:
    SimilarityVolume( int volDimX, int volDimY, int volDimZ,
                      int volStepXY,
                      int scale,
                      const std::vector<float>& depths_h,
                      bool verbose );
    ~SimilarityVolume( );

    void initOutputVolumes(
        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
        const int streamIndex );

    void compute(
          CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
          CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
          const CameraStruct& rcam, int rcWidth, int rcHeight,
          const CameraStruct& tcams, int tcWidth, int tcHeight,
          const OneTC&  cell,
          int wsh, float gammaC, float gammaP,
          int streamIndex );

    inline int DimX()      const { return _dimX; }
    inline int DimY()      const { return _dimY; }
    inline int DimZ()      const { return _dimZ; }
    inline int StepXY()    const { return _stepXY; }
    inline int Scale()     const { return _scale; }
    inline int PrevScale() const { return _scale-1; }

    cudaStream_t SweepStream( int offset );
    void WaitSweepStream( int offset );

private:
    const int  _dimX;
    const int  _dimY;
    const int  _dimZ;
    const int  _stepXY;
    const int  _scale;

    const bool _verbose;

    const CudaDeviceMemory<float> _depths_d;

    const int                 _stream_max;
    std::vector<cudaStream_t> _sweep_stream;

    /* CUDA can help us to find good block sizes for a kernel, depending
     * on architecture. Call configure_* functions and use *_block
     * afterwards.
     */
    static bool _configured;
    static dim3 _block;

    static void configureGrid( );
};
}; // namespace ps

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
    int rc_cam_cache_id,
    const CudaDeviceMemory<float>& depths_d,
    CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    int volDimX, int volDimY, int volDimZ, int scaleStep, bool interpolate);

int ps_listCUDADevices(bool verbose);

int ps_deviceAllocate(
    Pyramid& pyramid,
    int width,
    int height,
    int scales );

void ps_deviceDeallocate(
    Pyramid& pyramid,
    int scales );

void ps_testCUDAdeviceNo(int CUDAdeviceNo);

void ps_device_fillPyramidFromHostFrame(
    Pyramid& pyramid,
    CudaHostMemoryHeap<CudaRGBA, 2>* host_frame,
    int scales, int w, int h,
    cudaStream_t stream );

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

void ps_loadCameraStructs( const CameraStructBase* hst,
                           const CamCacheIdx&      offset,
                           cudaStream_t            stream );

} // namespace depthMap
} // namespace aliceVision

