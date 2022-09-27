// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_color.cu>
#include <aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cu>
// #include <aliceVision/depthMap/cuda/deviceCommon/device_eig33.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/device_code.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/device_code_refine.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/device_code_volume.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/device_code_fuse.cu>
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/images/gauss_filter.hpp>

#include <math_constants.h>

#include <iostream>
#include <algorithm>
#include <map>
#include <array>

namespace aliceVision {
namespace depthMap {

// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                    \
    if(cudaError_t err = cudaGetLastError())                                  \
    {                                                                         \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));      \
        fprintf(stderr, "  file:       %s\n", __FILE__);                      \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                  \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                    \
        std::stringstream s;                                                  \
        s << "\n  CUDA Error: " << cudaGetErrorString(err)                    \
          << "\n  file:       " << __FILE__                                   \
          << "\n  function:   " << __FUNCTION__                               \
          << "\n  line:       " << __LINE__ << "\n";                          \
        throw std::runtime_error(s.str());                                    \
    }

#define ALICEVISION_CU_PRINT_DEBUG(a) \
    std::cerr << a << std::endl;

#define ALICEVISION_CU_PRINT_ERROR(a) \
    std::cerr << a << std::endl;

__host__ float3 ps_M3x3mulV3(const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

__host__ void ps_normalize(float3& a)
{
    float d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

void pr_printfDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;
    cudaMemGetInfo(&iavail, &itotal);
    size_t iused = itotal - iavail;

    double avail = double(iavail) / (1024.0 * 1024.0);
    double total = double(itotal) / (1024.0 * 1024.0);
    double used = double(iused) / (1024.0 * 1024.0);

    int CUDAdeviceNo;
    cudaGetDevice(&CUDAdeviceNo);

    printf("Device %i memory - used: %f, free: %f, total: %f\n", CUDAdeviceNo, used, avail, total);
}

__host__ void ps_initCameraMatrix( CameraStructBase& base )
{
    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    base.ZVect = ps_M3x3mulV3(base.iR, z);
    ps_normalize(base.ZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    base.YVect = ps_M3x3mulV3(base.iR, y);
    ps_normalize(base.YVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    base.XVect = ps_M3x3mulV3(base.iR, x);
    ps_normalize(base.XVect);
}

int ps_listCUDADevices(bool verbose)
{
    int num_gpus = 0; // number of CUDA GPUs

    // determine the number of CUDA capable GPUs
    cudaError_t err = cudaGetDeviceCount(&num_gpus);
    CHECK_CUDA_ERROR();
    if(err != cudaSuccess)
    {
        printf("Error getting cuda device count");
        return 0;
    }

    if(num_gpus < 1)
    {
        printf("ERROR: no CUDA capable devices detected");
        return 0;
    }

    if(verbose == true)
    {
        // display CPU and GPU configuration
        printf("number of CUDA devices:\t%d\n", num_gpus);
        for(int i = 0; i < num_gpus; i++)
        {
            cudaDeviceProp dprop;
            cudaGetDeviceProperties(&dprop, i);
            printf("   %d: %s\n", i, dprop.name);
        }
    }

    return num_gpus;
}

int ps_deviceAllocate(Pyramid& pyramid, int width, int height, int scales )
{
    int bytesAllocated = 0;

    pyramid.resize(scales);

    for(int s = 0; s < scales; s++)
    {
        int w = width / (s + 1);
        int h = height / (s + 1);
        // printf("ps_deviceAllocate: CudaDeviceMemoryPitched: [c%i][s%i] %ix%i\n", c, s, w, h);
        pyramid[s].arr = new CudaDeviceMemoryPitched<CudaRGBA, 2>(CudaSize<2>(w, h));
        bytesAllocated += pyramid[s].arr->getBytesPadded();

        cudaTextureDesc  tex_desc;
        memset(&tex_desc, 0, sizeof(cudaTextureDesc));
        tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
        tex_desc.addressMode[0]   = cudaAddressModeClamp;
        tex_desc.addressMode[1]   = cudaAddressModeClamp;
        tex_desc.addressMode[2]   = cudaAddressModeClamp;
#if defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR) && defined(ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION)
        tex_desc.readMode = cudaReadModeNormalizedFloat; // uchar to float [0:1], see tex2d_float4 function
#else
        tex_desc.readMode = cudaReadModeElementType;
#endif
#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION
        // with subpixel interpolation (can have a large performance impact on some graphic cards)
        // but could be critical for quality during SGM in small resolution
        tex_desc.filterMode = cudaFilterModeLinear;
#else
        // without interpolation
        tex_desc.filterMode = cudaFilterModePoint;
#endif

        cudaResourceDesc res_desc;
        res_desc.resType = cudaResourceTypePitch2D;
        res_desc.res.pitch2D.desc = cudaCreateChannelDesc<CudaRGBA>();
        res_desc.res.pitch2D.devPtr       = pyramid[s].arr->getBuffer();
        res_desc.res.pitch2D.width        = pyramid[s].arr->getSize()[0];
        res_desc.res.pitch2D.height       = pyramid[s].arr->getSize()[1];
        res_desc.res.pitch2D.pitchInBytes = pyramid[s].arr->getPitch();

        cudaError_t err = cudaCreateTextureObject( &pyramid[s].tex, &res_desc, &tex_desc, 0 );
        THROW_ON_CUDA_ERROR( err, "Failed to bind texture object to cam array" );
    }

    return bytesAllocated;
}

void ps_deviceDeallocate( Pyramid& pyramid, int scales )
{
    for( TexturedArray& entry : pyramid )
    {
        delete entry.arr;
        cudaDestroyTextureObject( entry.tex );
    }
    pyramid.clear();
}

void ps_testCUDAdeviceNo(int CUDAdeviceNo)
{
    int myCUDAdeviceNo;
    cudaGetDevice(&myCUDAdeviceNo);
    if(myCUDAdeviceNo != CUDAdeviceNo)
    {
        printf("WARNING different device %i %i\n", myCUDAdeviceNo, CUDAdeviceNo);
    }
}

// void ps_device_updateCam( const CameraStruct& cam, int CUDAdeviceNo,
//                           int scales, int w, int h)
void ps_device_fillPyramidFromHostFrame( Pyramid& pyramid,
                          CudaHostMemoryHeap<CudaRGBA, 2>* host_frame,
                          int scales, int w, int h,
                          cudaStream_t stream )
{
    ALICEVISION_CU_PRINT_DEBUG(std::endl
              << "Calling " << __FUNCTION__ << std::endl
              << "    for " << scales << " scales"
              << ", w: " << w << ", h: " << h
              << std::endl);

    {
        /* copy texture's data from host to device */
        pyramid[0].arr->copyFrom( *host_frame, stream );

        const dim3 block(32, 2, 1);
        const dim3 grid(divUp(w, block.x), divUp(h, block.y), 1);
        ALICEVISION_CU_PRINT_DEBUG("rgb2lab_kernel: block=(" << block.x << ", " << block.y << ", " << block.z << "), grid=(" << grid.x << ", " << grid.y << ", " << grid.z << ")");

        /* in-place color conversion into CIELAB */
        rgb2lab_kernel<<<grid, block, 0, stream>>>(
            pyramid[0].arr->getBuffer(), pyramid[0].arr->getPitch(),
            w, h);
        CHECK_CUDA_ERROR();
    }

    /* For each scale, create a Gaussian-filtered and scaled version of the
     * initial texture */
    for(int scale = 1; scale < scales; ++scale)
    {
        const int radius = scale + 1;
        // const int sWidth = w / (scale + 1);
        // const int sHeight = h / (scale + 1);
        // ALICEVISION_CU_PRINT_DEBUG("Create downscaled image for camera id " << camId << " at scale " << scale << ": " << sWidth << "x" << sHeight);

        // const dim3 block(32, 2, 1);
        // const dim3 grid(divUp(sWidth, block.x), divUp(sHeight, block.y), 1);
        // ALICEVISION_CU_PRINT_DEBUG("ps_downscale_gauss: block=(" << block.x << ", " << block.y << ", " << block.z << "), grid=(" << grid.x << ", " << grid.y << ", " << grid.z << ")");

        ps_downscale_gauss(pyramid, scale, w, h, radius, stream);
        CHECK_CUDA_ERROR();
    }

    CHECK_CUDA_ERROR();
}


/**
 * @param[inout] d_volSimT similarity volume
 */
void ps_aggregatePathVolume(
    CudaDeviceMemoryPitched<TSim, 3>& d_volAgr,
    const CudaDeviceMemoryPitched<TSim, 3>& d_volSim,
    const CudaSize<3>& volDim,
    const CudaSize<3>& axisT,
    cudaTextureObject_t rc_tex, 
    const SgmParams& sgmParams,
    bool invY, int filteringIndex)
{
    const size_t volDimX = volDim[axisT[0]];
    const size_t volDimY = volDim[axisT[1]];
    const size_t volDimZ = volDim[axisT[2]];

    const int3 volDim_ = make_int3(volDim[0], volDim[1], volDim[2]);
    const int3 axisT_ = make_int3(axisT[0], axisT[1], axisT[2]);
    const int ySign = (invY ? -1 : 1);

    // setup block and grid
    const int blockSize = 8;
    const dim3 blockVolXZ(blockSize, blockSize, 1);
    const dim3 gridVolXZ(divUp(volDimX, blockVolXZ.x), divUp(volDimZ, blockVolXZ.y), 1);

    const int blockSizeL = 64;
    const dim3 blockColZ(blockSizeL, 1, 1);
    const dim3 gridColZ(divUp(volDimX, blockColZ.x), 1, 1);

    const dim3 blockVolSlide(blockSizeL, 1, 1);
    const dim3 gridVolSlide(divUp(volDimX, blockVolSlide.x), volDimZ, 1);

    CudaDeviceMemoryPitched<TSimAcc, 2> d_sliceBufferA(CudaSize<2>(volDimX, volDimZ));
    CudaDeviceMemoryPitched<TSimAcc, 2> d_sliceBufferB(CudaSize<2>(volDimX, volDimZ));

    CudaDeviceMemoryPitched<TSimAcc, 2>* d_xzSliceForY = &d_sliceBufferA; // Y slice
    CudaDeviceMemoryPitched<TSimAcc, 2>* d_xzSliceForYm1 = &d_sliceBufferB; // Y-1 slice

    CudaDeviceMemoryPitched<TSimAcc, 2> d_bestSimInYm1(CudaSize<2>(volDimX, 1)); // best sim score along the Y axis for each Z value

    // Copy the first XZ plane (at Y=0) from 'd_volSim' into 'd_xzSliceForYm1'
    volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ>>>(
        d_xzSliceForYm1->getBuffer(),
        d_xzSliceForYm1->getPitch(),
        d_volSim.getBuffer(),
        d_volSim.getBytesPaddedUpToDim(1),
        d_volSim.getBytesPaddedUpToDim(0),
        volDim_, axisT_, 0); // Y=0

    // Set the first Z plane from 'd_volAgr' to 255
    volume_initVolumeYSlice_kernel<TSim><<<gridVolXZ, blockVolXZ>>>(
        d_volAgr.getBuffer(),
        d_volAgr.getBytesPaddedUpToDim(1),
        d_volAgr.getBytesPaddedUpToDim(0),
        volDim_, axisT_, 0, 255);

    for(int iy = 1; iy < volDimY; ++iy)
    {
        const int y = invY ? volDimY - 1 - iy : iy;

        // For each column: compute the best score
        // Foreach x:
        //   d_zBestSimInYm1[x] = min(d_xzSliceForY[1:height])
        volume_computeBestZInSlice_kernel<<<gridColZ, blockColZ>>>(
            d_xzSliceForYm1->getBuffer(), d_xzSliceForYm1->getPitch(),
            d_bestSimInYm1.getBuffer(),
            volDimX, volDimZ);

        // Copy the 'z' plane from 'd_volSimT' into 'd_xzSliceForY'
        volume_getVolumeXZSlice_kernel<TSimAcc, TSim><<<gridVolXZ, blockVolXZ>>>(
            d_xzSliceForY->getBuffer(),
            d_xzSliceForY->getPitch(),
            d_volSim.getBuffer(),
            d_volSim.getBytesPaddedUpToDim(1),
            d_volSim.getBytesPaddedUpToDim(0),
            volDim_, axisT_, y);

        volume_agregateCostVolumeAtXinSlices_kernel<<<gridVolSlide, blockVolSlide>>>(
            rc_tex,
            d_xzSliceForY->getBuffer(), d_xzSliceForY->getPitch(),              // inout: xzSliceForY
            d_xzSliceForYm1->getBuffer(), d_xzSliceForYm1->getPitch(),          // in:    xzSliceForYm1
            d_bestSimInYm1.getBuffer(),                                         // in:    bestSimInYm1
            d_volAgr.getBuffer(), d_volAgr.getBytesPaddedUpToDim(1), d_volAgr.getBytesPaddedUpToDim(0), // out:   volAgr
            volDim_, axisT_, 
            sgmParams.stepXY, 
            y, 
            sgmParams.p1, 
            sgmParams.p2Weighting,
            ySign, filteringIndex);

        std::swap(d_xzSliceForYm1, d_xzSliceForY);
    }
    // CHECK_CUDA_ERROR();
}

void ps_SGMretrieveBestDepth(int rcamCacheId,
    CudaDeviceMemoryPitched<float, 2>& bestDepth_dmp,
    CudaDeviceMemoryPitched<float, 2>& bestSim_dmp, 
    const CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    const CudaSize<3>& volDim,
    const CudaDeviceMemory<float>& depths_d,
    int scaleStep, bool interpolate)
{
  const int block_size = 8;
  const dim3 block(block_size, block_size, 1);
  const dim3 grid(divUp(volDim.x(), block_size), divUp(volDim.y(), block_size), 1);

  volume_retrieveBestZ_kernel<<<grid, block>>>(
    rcamCacheId,
    bestDepth_dmp.getBuffer(),
    bestDepth_dmp.getBytesPaddedUpToDim(0),
    bestSim_dmp.getBuffer(),
    bestSim_dmp.getBytesPaddedUpToDim(0),
    volSim_dmp.getBuffer(),
    volSim_dmp.getBytesPaddedUpToDim(1), volSim_dmp.getBytesPaddedUpToDim(0), 
    int(volDim.x()), 
    int(volDim.y()), 
    int(volDim.z()), 
    depths_d.getBuffer(),
    scaleStep,
    interpolate);
}



namespace ps
{
/*
 * static private variables in this class
 */
bool SimilarityVolume::_configured = false;
dim3 SimilarityVolume::_block( 32, 1, 1 ); // minimal default settings

SimilarityVolume::SimilarityVolume( const CudaSize<3>& volDim,
                                    int volStepXY,
                                    int scale,
                                    const std::vector<float>& depths_h)
    : _dimX(int(volDim.x()))
    , _dimY(int(volDim.y()))
    , _dimZ(int(volDim.z()))
    , _stepXY(volStepXY)
    , _scale(scale)
    , _depths_d(depths_h.data(), depths_h.size())
    , _stream_max( 2 )
{
    configureGrid();

    _sweep_stream.resize(_stream_max);
    for( cudaStream_t& stream : _sweep_stream )
    {
        cudaError_t err;
        err = cudaStreamCreate( &stream );
        if( err != cudaSuccess )
        {
            ALICEVISION_CU_PRINT_DEBUG("Failed to create a CUDA stream object for SimilarityVolume");
            stream = 0;
        }
    }
}

SimilarityVolume::~SimilarityVolume( )
{
    for( cudaStream_t& stream : _sweep_stream )
    {
        cudaStreamSynchronize( stream );
        if( stream != 0 ) cudaStreamDestroy( stream );
    }
}

void SimilarityVolume::initOutputVolumes(
    CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
    CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
    const int streamIndex )
{
  const dim3 block(32, 4, 1);
  const dim3 grid(divUp(_dimX, block.x), divUp(_dimY, block.y), _dimZ);

  volume_init_kernel
    <<<grid, block, 0, SweepStream(streamIndex)>>>
    (volBestSim_dmp.getBuffer(),
      volBestSim_dmp.getBytesPaddedUpToDim(1),
      volBestSim_dmp.getBytesPaddedUpToDim(0),
      _dimX, _dimY);
  volume_init_kernel
    <<<grid, block, 0, SweepStream(streamIndex)>>>
    (volSecBestSim_dmp.getBuffer(),
      volSecBestSim_dmp.getBytesPaddedUpToDim(1),
      volSecBestSim_dmp.getBytesPaddedUpToDim(0),
      _dimX, _dimY);
}

void SimilarityVolume::compute(
                        CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                        CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                        const CameraStruct& rcam, int rcWidth, int rcHeight,
                        const CameraStruct& tcam, int tcWidth, int tcHeight,
                        const OneTC& cell,
                        const SgmParams& sgmParams,
                        const int streamIndex )
{
    TSim* gpu_volume_1st = volBestSim_dmp.getBuffer();
    TSim* gpu_volume_2nd = volSecBestSim_dmp.getBuffer();

    {
      const int startDepthIndex = cell.getDepthToStart();
      const int nbDepthsToSearch = cell.getDepthsToSearch();

      const dim3 grid(divUp(_dimX, _block.x), divUp(_dimY, _block.y), nbDepthsToSearch);

      ALICEVISION_CU_PRINT_DEBUG("====================");
      ALICEVISION_CU_PRINT_DEBUG("Volume slice kernel");
      ALICEVISION_CU_PRINT_DEBUG("RC: " << rcam.camId << ", TC: " << tcam.camId);
      ALICEVISION_CU_PRINT_DEBUG("Cell TC index: " << cell.getTCIndex());
      ALICEVISION_CU_PRINT_DEBUG("grid:  " << grid.x << ", " << grid.y << ", " << grid.z);
      ALICEVISION_CU_PRINT_DEBUG("block: " << _block.x << ", " << _block.y << ", " << _block.z);
      ALICEVISION_CU_PRINT_DEBUG("startDepthIndex: " << startDepthIndex);
      ALICEVISION_CU_PRINT_DEBUG("nbDepthsToSearch: " << nbDepthsToSearch);
      ALICEVISION_CU_PRINT_DEBUG("nb all depths: " << int(_depths_d.getUnitsTotal()));
      ALICEVISION_CU_PRINT_DEBUG("startDepthIndex+nbDepthsToSearch: " << startDepthIndex+nbDepthsToSearch);
      ALICEVISION_CU_PRINT_DEBUG("_dimX: " << _dimX << ", _dimY: " << _dimY);
      ALICEVISION_CU_PRINT_DEBUG("scale-1: " << prevScale() );
      ALICEVISION_CU_PRINT_DEBUG("rcWH / scale: " << rcWidth / _scale << "x" << rcHeight / _scale);
      ALICEVISION_CU_PRINT_DEBUG("tcWH / scale: " << tcWidth / _scale << "x" << tcHeight / _scale);
      ALICEVISION_CU_PRINT_DEBUG("====================");

      const Pyramid& rc_pyramid = *rcam.pyramid;
      const Pyramid& tc_pyramid = *tcam.pyramid;
      cudaTextureObject_t rc_tex = rc_pyramid[prevScale()].tex;
      cudaTextureObject_t tc_tex = tc_pyramid[prevScale()].tex;

      volume_slice_kernel
            <<<grid, _block, 0, SweepStream(streamIndex)>>>
            ( rc_tex,
              tc_tex,
              rcam.param_dev.i,
              tcam.param_dev.i,
              _depths_d.getBuffer(),
              startDepthIndex,
              nbDepthsToSearch,
              rcWidth / _scale, rcHeight / _scale,
              tcWidth / _scale, tcHeight / _scale,
              sgmParams.wsh,
              float(sgmParams.gammaC), 
              float(sgmParams.gammaP),
              gpu_volume_1st,
              volBestSim_dmp.getBytesPaddedUpToDim(1),
              volBestSim_dmp.getBytesPaddedUpToDim(0),
              gpu_volume_2nd,
              volSecBestSim_dmp.getBytesPaddedUpToDim(1),
              volSecBestSim_dmp.getBytesPaddedUpToDim(0),
              _stepXY,
              _dimX, _dimY);

        // cudaDeviceSynchronize();
        // CHECK_CUDA_ERROR();
    }

    // cudaDeviceSynchronize();
}

cudaStream_t SimilarityVolume::SweepStream( int streamIndex )
{
    streamIndex %= _stream_max;
    return _sweep_stream[streamIndex];
}

void SimilarityVolume::WaitSweepStream( const int streamIndex )
{
    cudaStreamSynchronize( SweepStream(streamIndex) );
}

void SimilarityVolume::configureGrid( )
{

    if( _configured ) return;
    _configured = true;

    int recommendedMinGridSize;
    int recommendedBlockSize;
    cudaError_t err;
    err = cudaOccupancyMaxPotentialBlockSize( &recommendedMinGridSize,
                                              &recommendedBlockSize,
                                              volume_slice_kernel,
                                              0, // dynamic shared mem size: none used
                                              0 ); // no block size limit, 1 thread OK
    if( err != cudaSuccess )
    {
        ALICEVISION_CU_PRINT_DEBUG( "cudaOccupancyMaxPotentialBlockSize failed for kernel volume_slice_kernel, using defaults" );
    }
    else
    {
        if( recommendedBlockSize > 32 )
        {
            _block.x = 32;
            _block.y = divUp( recommendedBlockSize, 32 );
        }
    }
}
}; // namespace ps

void ps_refineRcDepthMap(const CameraStruct& rcam, 
                         const CameraStruct& tcam,
                         float* inout_depthMap_hmh,
                         float* out_simMap_hmh,
                         int rcWidth, int rcHeight,
                         int tcWidth, int tcHeight,
                         const RefineParams& refineParams, 
                         int xFrom, int wPart, int CUDAdeviceNo)
{
    // setup block and grid
    const dim3 block(16, 16, 1);
    const dim3 grid(divUp(wPart, block.x), divUp(rcHeight, block.y), 1);

    const Pyramid& rcPyramid = *rcam.pyramid;
    const Pyramid& tcPyramid = *tcam.pyramid;
    const size_t pyramidScaleIndex = size_t(refineParams.scale) - 1;

    cudaTextureObject_t rc_tex = rcPyramid[pyramidScaleIndex].tex;
    cudaTextureObject_t tc_tex = tcPyramid[pyramidScaleIndex].tex;

    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(wPart, rcHeight));
    copy(rcDepthMap_dmp, inout_depthMap_hmh, wPart, rcHeight);

    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(wPart, rcHeight));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(wPart, rcHeight));

    const int halfNSteps = ((refineParams.nDepthsToRefine - 1) / 2) + 1; // Default ntcsteps = 31

    for(int i = 0; i < halfNSteps; ++i)
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcam.param_dev.i,
            tcam.param_dev.i,
            rc_tex, tc_tex,
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(), 
            wPart, rcHeight, 
            refineParams.wsh, 
            refineParams.gammaC, 
            refineParams.gammaP,
            float(i), 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcWidth, rcHeight,
            tcWidth, tcHeight);
    }

    for(int i = 1; i < halfNSteps; ++i)
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcam.param_dev.i, 
            tcam.param_dev.i, 
            rc_tex, tc_tex, 
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(), 
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(), 
            wPart, rcHeight, 
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            float(-i),
            refineParams.useTcOrRcPixSize, 
            xFrom, 
            rcWidth, rcHeight, 
            tcWidth, tcHeight);
    }

    /*
    // Filter intermediate refined images does not improve
    for (int i = 0; i < 5; ++i)
    {
        // Filter refined depth map
        CudaTexture<float> depthTex(bestDptMap_dmp);
        float euclideanDelta = 1.0;
        int radius = 3;
        ps_bilateralFilter<float>(
            depthTex.textureObj,
            bestDptMap_dmp,
            euclideanDelta,
            radius);
        ps_medianFilter<float>(
            depthTex.textureObj,
            bestDptMap_dmp,
            radius);
    }
    */

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap_dmp(CudaSize<2>(wPart, rcHeight));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(wPart, rcHeight));

    {
        // Set best sim map into lastThreeSimsMap_dmp.y
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(), 
            wPart, rcHeight, 1);
        /*
        // Compute NCC for depth-1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rc_cam.param_dev.i, 
            tc_cam.param_dev.i,
            rc_tex, tc_tex,
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            wPart, rcHeight,
            refineParams.wsh,
            refineParams.gammaC,
            refineParams.gammaP,
            0.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcWidth, rcHeight,
            tcWidth, tcHeight);

        // Set sim for depth-1 into lastThreeSimsMap_dmp.y
        refine_setLastThreeSimsMap_kernel <<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            wPart, rcHeight, 1);
        */
    }

    {
        // Compute NCC for depth-1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcam.param_dev.i,
            tcam.param_dev.i, 
            rc_tex, tc_tex,
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(), 
            wPart, rcHeight, 
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            -1.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcWidth, rcHeight,
            tcWidth, tcHeight);

        // Set sim for depth-1 into lastThreeSimsMap_dmp.x
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(),
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(), 
            wPart, rcHeight, 0);
    }

    {
        // Compute NCC for depth+1
        refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
            rcam.param_dev.i,
            tcam.param_dev.i,
            rc_tex, tc_tex,
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(), 
            wPart, rcHeight, 
            refineParams.wsh,
            refineParams.gammaC, 
            refineParams.gammaP,
            +1.0f, 
            refineParams.useTcOrRcPixSize, 
            xFrom,
            rcWidth, rcHeight,
            tcWidth, tcHeight);

        // Set sim for depth+1 into lastThreeSimsMap_dmp.z
        refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
            lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(), 
            simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
            wPart, rcHeight, 2);
    }

    // Interpolation from the lastThreeSimsMap_dmp
    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        rcam.param_dev.i,
        tcam.param_dev.i,
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        lastThreeSimsMap_dmp.getBuffer(), lastThreeSimsMap_dmp.getPitch(), 
        wPart, rcHeight,  
        refineParams.useTcOrRcPixSize, 
        xFrom);

    copy(out_simMap_hmh, wPart, rcHeight, bestSimMap_dmp);
    copy(inout_depthMap_hmh, wPart, rcHeight, bestDptMap_dmp);
}

/**
 * @brief ps_fuseDepthSimMapsGaussianKernelVoting
 * @param ndepthSimMaps: number of Tc cameras
 * @param nSamplesHalf (default value 150)
 * @param nDepthsToRefine (default value 31)
 */
void ps_fuseDepthSimMapsGaussianKernelVoting(int width, int height,
                                             CudaHostMemoryHeap<float2, 2>* out_depthSimMap_hmh,
                                             std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh, 
                                             int ndepthSimMaps, 
                                             const RefineParams& refineParams)
{
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));
    const float twoTimesSigmaPowerTwo = 2.0f * refineParams.sigma * refineParams.sigma;

    // setup block and grid
    const int block_size = 16;
    const dim3 block(block_size, block_size, 1);
    const dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    CudaDeviceMemoryPitched<float2, 2> bestDepthSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> bestGsvSampleMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> gsvSampleMap_dmp(CudaSize<2>(width, height));
    std::vector<CudaDeviceMemoryPitched<float2, 2>*> depthSimMaps_dmp(ndepthSimMaps);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        depthSimMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*depthSimMaps_dmp[i]), (*depthSimMaps_hmh[i]));
    }

    for(int s = -refineParams.nSamplesHalf; s <= refineParams.nSamplesHalf; s++) // (-150, 150)
    {
        for(int c = 1; c < ndepthSimMaps; c++) // number of T cameras
        {
            fuse_computeGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
                gsvSampleMap_dmp.getBuffer(), gsvSampleMap_dmp.getPitch(),
                depthSimMaps_dmp[c]->getBuffer(), depthSimMaps_dmp[c]->getPitch(),
                depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->getPitch(),
                width, height, (float)s, c - 1, samplesPerPixSize, twoTimesSigmaPowerTwo);
        }
        fuse_updateBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
            bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.getPitch(),
            gsvSampleMap_dmp.getBuffer(), gsvSampleMap_dmp.getPitch(), 
            width, height, (float)s, s + refineParams.nSamplesHalf);
    }

    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
        bestDepthSimMap_dmp.getBuffer(), bestDepthSimMap_dmp.getPitch(),
        bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.getPitch(),
        depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->getPitch(), 
        width, height, samplesPerPixSize);

    copy((*out_depthSimMap_hmh), bestDepthSimMap_dmp);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        delete depthSimMaps_dmp[i];
    }
}

void ps_optimizeDepthSimMapGradientDescent(const CameraStruct& rcam,
                                           CudaHostMemoryHeap<float2, 2>& out_optimizedDepthSimMap_hmh,
                                           const CudaHostMemoryHeap<float2, 2>& sgmDepthPixSizeMap_hmh,
                                           const CudaHostMemoryHeap<float2, 2>& refinedDepthSimMap_hmh,
                                           const CudaSize<2>& depthSimMapPartDim, 
                                           const RefineParams& refineParams,
                                           int CUDAdeviceNo, int nbCamsAllocated, int yFrom)
{
    const int partWidth = depthSimMapPartDim.x();
    const int partHeight = depthSimMapPartDim.y(); 
    const float samplesPerPixSize = float(refineParams.nSamplesHalf / ((refineParams.nDepthsToRefine - 1) / 2));

    // setup block and grid
    const int block_size = 16;
    const dim3 block(block_size, block_size, 1);
    const dim3 grid(divUp(partWidth, block_size), divUp(partHeight, block_size), 1);

    const CudaDeviceMemoryPitched<float2, 2> sgmDepthPixSizeMap_dmp(sgmDepthPixSizeMap_hmh);
    const CudaDeviceMemoryPitched<float2, 2> refinedDepthSimMap_dmp(refinedDepthSimMap_hmh);

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(depthSimMapPartDim);
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(depthSimMapPartDim);
    copy(optDepthSimMap_dmp, sgmDepthPixSizeMap_dmp);

    // get rc CUDA texture object
    const size_t pyramidScaleIndex = size_t(refineParams.scale) - 1;
    const Pyramid& rcPyramid = *rcam.pyramid;
    cudaTextureObject_t rc_tex = rcPyramid[pyramidScaleIndex].tex;

    CudaDeviceMemoryPitched<float, 2> imgVariance_dmp(depthSimMapPartDim);
    {
        const dim3 lblock(32, 2, 1);
        const dim3 lgrid(divUp(partWidth, lblock.x), divUp(partHeight, lblock.y), 1);

        compute_varLofLABtoW_kernel<<<lgrid, lblock>>>(rc_tex,
                                                       imgVariance_dmp.getBuffer(), 
                                                       imgVariance_dmp.getPitch(),
                                                       partWidth, partHeight, yFrom);
    }
    CudaTexture<float> imgVarianceTex(imgVariance_dmp);

    for(int iter = 0; iter < refineParams.nIters; ++iter) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOptDepthSimMap_kernel<<<grid, block>>>(optDepthMap_dmp.getBuffer(), optDepthMap_dmp.getPitch(),
                                                                     optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(), 
                                                                     partWidth, partHeight);

        CudaTexture<float> depthTex(optDepthMap_dmp);

        // Adjust depth/sim by using previously computed depths
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(rc_tex, 
                                                         rcam.param_dev.i,
                                                         imgVarianceTex.textureObj, 
                                                         depthTex.textureObj,
                                                         optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
                                                         sgmDepthPixSizeMap_dmp.getBuffer(), sgmDepthPixSizeMap_dmp.getPitch(),
                                                         refinedDepthSimMap_dmp.getBuffer(), refinedDepthSimMap_dmp.getPitch(), 
                                                         partWidth, partHeight, iter, samplesPerPixSize, yFrom);
    }

    copy(out_optimizedDepthSimMap_hmh, optDepthSimMap_dmp);
}

// uchar4 with 0..255 components => float3 with 0..1 components
inline __device__ __host__ float3 uchar4_to_float3(const uchar4 c)
{
    return make_float3(float(c.x) / 255.0f, float(c.y) / 255.0f, float(c.z) / 255.0f);
}

void ps_getSilhoueteMap(CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
                        int height, int scale,
                        int step,
                        CameraStruct& cam,
                        uchar4 maskColorRgb, bool verbose)
{
    clock_t tall = tic();

    uchar4 maskColorLab;
    float3 flab = xyz2lab(h_rgb2xyz(uchar4_to_float3(maskColorRgb)));
    maskColorLab.x = (unsigned char)(flab.x);
    maskColorLab.y = (unsigned char)(flab.y);
    maskColorLab.z = (unsigned char)(flab.z);
    maskColorLab.w = 0;

    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width / step, block_size), divUp(height / step, block_size), 1);

    Pyramid& pyramid = *cam.pyramid;

    CudaDeviceMemoryPitched<bool, 2> map_dmp(CudaSize<2>(width / step, height / step));
    getSilhoueteMap_kernel<<<grid, block>>>(
        pyramid[scale].tex,
        map_dmp.getBuffer(), map_dmp.getPitch(),
        step, width, height, maskColorLab);
    CHECK_CUDA_ERROR();

    copy((*omap_hmh), map_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}


void ps_loadCameraStructs( const CameraStructBase* hst,
                           const CamCacheIdx&      offset,
                           cudaStream_t            stream )
{
    cudaMemcpyKind kind = cudaMemcpyHostToDevice;
    cudaError_t err;
    if( stream == 0 )
    {
        err = cudaMemcpyToSymbol( camsBasesDev,
                                  &hst[offset.i],
                                  sizeof(CameraStructBase),
                                  offset.i*sizeof(CameraStructBase),
                                  kind );
    }
    else
    {
        err = cudaMemcpyToSymbolAsync( camsBasesDev,
                                       &hst[offset.i],
                                       sizeof(CameraStructBase),
                                       offset.i*sizeof(CameraStructBase),
                                       kind,
                                       stream );
    }
    THROW_ON_CUDA_ERROR( err, "Failed to copy CameraStructs from host to device in " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(err) );
}

} // namespace depthMap
} // namespace aliceVision
