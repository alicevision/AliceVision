// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/depthMap/SgmParams.hpp>
#include <aliceVision/depthMap/RefineParams.hpp>
#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/OneTC.hpp>

namespace aliceVision {
namespace depthMap {

#ifdef TSIM_USE_FLOAT
    using TSim = float;
    using TSimAcc = float;
#else
    using TSim = unsigned char;
    using TSimAcc = unsigned int; // TSimAcc is the similarity accumulation type
#endif


void ps_initCameraMatrix( CameraStructBase& base );

void pr_printfDeviceMemoryInfo();


namespace ps
{
class SimilarityVolume
{
public:
    SimilarityVolume( const CudaSize<3>& volDim,
                      int volStepXY,
                      int scale,
                      const std::vector<float>& depths_h);
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
          const SgmParams& sgmParams,
          int streamIndex );

    inline int dimX()      const { return _dimX; }
    inline int dimY()      const { return _dimY; }
    inline int dimZ()      const { return _dimZ; }
    inline int stepXY()    const { return _stepXY; }
    inline int scale()     const { return _scale; }
    inline int prevScale() const { return _scale-1; }

    cudaStream_t SweepStream( int offset );
    void WaitSweepStream( int offset );

private:
    const int  _dimX;
    const int  _dimY;
    const int  _dimZ;
    const int  _stepXY;
    const int  _scale;

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

void ps_aggregatePathVolume(CudaDeviceMemoryPitched<TSim, 3>& d_volAgr,
                            const CudaDeviceMemoryPitched<TSim, 3>& d_volSim, 
                            const CudaSize<3>& volDim,
                            const CudaSize<3>& axisT, cudaTextureObject_t rc_tex, 
                            const SgmParams& sgmParams,
                            bool invY, int filteringIndex);

void ps_SGMretrieveBestDepth(int rcamCacheId, 
                            CudaDeviceMemoryPitched<float, 2>& bestDepth_dmp,
                            CudaDeviceMemoryPitched<float, 2>& bestSim_dmp,
                            const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp, 
                            const CudaSize<3>& volDim, 
                            const CudaDeviceMemory<float>& depths_d, 
                            int scaleStep, bool interpolate);

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

void ps_refineRcDepthMap(const CameraStruct& rcam, 
                         const CameraStruct& tcam, 
                         float* inout_depthMap_hmh,
                         float* out_simMap_hmh, 
                         int rcWidth, int rcHeight, 
                         int tcWidth, int tcHeight,
                         const RefineParams& refineParams, 
                         int xFrom, int wPart, int CUDAdeviceNo);

void ps_fuseDepthSimMapsGaussianKernelVoting(int width, int height,
                                            CudaHostMemoryHeap<float2, 2>* out_depthSimMap_hmh,
                                            std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh,
                                            int ndepthSimMaps, 
                                            const RefineParams& refineParams);

void ps_optimizeDepthSimMapGradientDescent(const CameraStruct& rcam,
                                           CudaHostMemoryHeap<float2, 2>& out_optimizedDepthSimMap_hmh,
                                           const CudaHostMemoryHeap<float2, 2>& sgmDepthPixSizeMap_hmh,
                                           const CudaHostMemoryHeap<float2, 2>& refinedDepthSimMap_hmh,
                                           const CudaSize<2>& depthSimMapPartDim, 
                                           const RefineParams& refineParams,
                                           int CUDAdeviceNo, int nbCamsAllocated, int yFrom);

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

