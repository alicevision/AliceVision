// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/depthMap/cuda/commonStructures.hpp>
#include <aliceVision/depthMap/cuda/deviceCommon/device_color.cu>
#include <aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cu>
#include <aliceVision/depthMap/cuda/deviceCommon/device_eig33.cu>
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

/* CUDA can help us to find good block sizes for a kernel, depending
 * on architecture. Call configure_* functions and use *_block
 * afterwards.
 */
static bool volume_slice_kernel_block_set = false;
static dim3 volume_slice_kernel_block( 32, 1, 1 ); // minimal default settings

__host__ void configure_volume_slice_kernel( )
{
    if( volume_slice_kernel_block_set ) return;
    volume_slice_kernel_block_set = true;

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
            volume_slice_kernel_block.x = 32;
            volume_slice_kernel_block.y = divUp( recommendedBlockSize, 32 );
        }
    }
}

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

void ps_deviceAllocate(Pyramids& ps_texs_arr, int ncams, int width, int height, int scales,
                       int deviceId)
{
    int num_gpus = 0;
    cudaGetDeviceCount(&num_gpus);


    cudaError_t outval = cudaSetDevice(deviceId);
    if( outval != cudaSuccess )
    {
        ALICEVISION_CU_PRINT_ERROR( "Failed to set CUDA device " << deviceId << " for thread: " << cudaGetErrorString(outval) );
    }
    printf("Setting CUDA device to %i\n", deviceId);

    pr_printfDeviceMemoryInfo();

    // copy textures to the device
    int allBytes = 0;
    ps_texs_arr.resize(ncams);
    for(int c = 0; c < ncams; c++)
    {
        ps_texs_arr[c].resize(scales);
    }
    for(int c = 0; c < ncams; c++)
    {
        for(int s = 0; s < scales; s++)
        {
            int w = width / (s + 1);
            int h = height / (s + 1);
            printf("ps_deviceAllocate: CudaDeviceMemoryPitched: [c%i][s%i] %ix%i\n", c, s, w, h);
            ps_texs_arr[c][s].arr = new CudaDeviceMemoryPitched<float4, 2>(CudaSize<2>(w, h));
            allBytes += ps_texs_arr[c][s].arr->getBytesPadded();

            cudaTextureDesc  tex_desc;
            memset(&tex_desc, 0, sizeof(cudaTextureDesc));
            tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
            tex_desc.addressMode[0]   = cudaAddressModeClamp;
            tex_desc.addressMode[1]   = cudaAddressModeClamp;
            tex_desc.addressMode[2]   = cudaAddressModeClamp;
            tex_desc.readMode         = cudaReadModeElementType; // transform uchar to float
            tex_desc.filterMode       = cudaFilterModeLinear; // with interpolation

            cudaResourceDesc res_desc;
            res_desc.resType = cudaResourceTypePitch2D;
            res_desc.res.pitch2D.desc = cudaCreateChannelDesc<float4>();
            res_desc.res.pitch2D.devPtr       = ps_texs_arr[c][s].arr->getBuffer();
            res_desc.res.pitch2D.width        = ps_texs_arr[c][s].arr->getSize()[0];
            res_desc.res.pitch2D.height       = ps_texs_arr[c][s].arr->getSize()[1];
            res_desc.res.pitch2D.pitchInBytes = ps_texs_arr[c][s].arr->getPitch();

            cudaError_t err = cudaCreateTextureObject( &ps_texs_arr[c][s].tex, &res_desc, &tex_desc, 0 );
            THROW_ON_CUDA_ERROR( err, "Failed to bind texture object to cam array" );
        }
    }

    CHECK_CUDA_ERROR();

    // calcDCTCoefficients();
    // CHECK_CUDA_ERROR();

    printf("scales %i\n",scales);

    printf("total size of preallocated images in GPU memory: %f\n",double(allBytes)/(1024.0*1024.0));

    pr_printfDeviceMemoryInfo();
    // printf("ps_deviceAllocate - done\n");
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

void ps_device_updateCam( Pyramids& ps_texs_arr,
                         const CameraStruct& cam, int camId, int CUDAdeviceNo,
                         int ncamsAllocated, int scales, int w, int h)
{
    ALICEVISION_CU_PRINT_DEBUG(std::endl
              << "Calling " << __FUNCTION__ << std::endl
              << "    for camera id " << camId << " and " << scales << " scales"
              << ", w: " << w << ", h: " << h << ", ncamsAllocated: " << ncamsAllocated
              << std::endl);

    {
        copy(*ps_texs_arr[camId][0].arr, (*cam.tex_rgba_hmh));

        const dim3 block(32, 2, 1);
        const dim3 grid(divUp(w, block.x), divUp(h, block.y), 1);
        ALICEVISION_CU_PRINT_DEBUG("rgb2lab_kernel: block=(" << block.x << ", " << block.y << ", " << block.z << "), grid=(" << grid.x << ", " << grid.y << ", " << grid.z << ")");

        rgb2lab_kernel<<<grid, block>>>(
            ps_texs_arr[camId][0].arr->getBuffer(), ps_texs_arr[camId][0].arr->getPitch(),
            w, h);
        CHECK_CUDA_ERROR();
    }

    ps_create_gaussian_arr( CUDAdeviceNo, scales );
    CHECK_CUDA_ERROR();

    // for each scale
    for(int scale = 1; scale < scales; ++scale)
    {
        const int radius = scale + 1;
        const int sWidth = w / (scale + 1);
        const int sHeight = h / (scale + 1);
        ALICEVISION_CU_PRINT_DEBUG("Create downscaled image for camera id " << camId << " at scale " << scale
                  << ": " << sWidth << "x" << sHeight);

        const dim3 block(32, 2, 1);
        const dim3 grid(divUp(sWidth, block.x), divUp(sHeight, block.y), 1);
        ALICEVISION_CU_PRINT_DEBUG("ps_downscale_gauss: block=(" << block.x << ", " << block.y << ", " << block.z << "), grid=(" << grid.x << ", " << grid.y << ", " << grid.z << ")");

        ps_downscale_gauss(ps_texs_arr, camId, scale, w, h, radius);
        CHECK_CUDA_ERROR();
    }

    CHECK_CUDA_ERROR();
}

void ps_deviceDeallocate(Pyramids& ps_texs_arr, int CUDAdeviceNo, int ncams, int scales)
{
    for(int c = 0; c < ncams; c++)
    {
        for(int s = 0; s < scales; s++)
        {
            delete ps_texs_arr[c][s].arr;
            cudaDestroyTextureObject( ps_texs_arr[c][s].tex );
        }
    }
}

/**
 * @param[inout] d_volSimT similarity volume with some transposition applied
 */
void ps_aggregatePathVolume(CudaDeviceMemoryPitched<TSim, 3>& d_volSimT,
                            int volDimX, int volDimY, int volDimZ,
                            cudaTextureObject_t rc_tex,
                            float P1, float P2,
                            int dimTrnX, bool doInvZ, bool verbose)
{
    if(verbose)
        printf("ps_aggregatePathVolume\n");

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    int block_sizenmxs = 64;
    dim3 blockvolrow(block_sizenmxs, 1, 1);
    dim3 gridvolrow(divUp(volDimX, block_sizenmxs), 1, 1);
    dim3 gridvolrowAllCols(divUp(volDimX, block_sizenmxs), volDimY, 1);

    CudaDeviceMemoryPitched<TSim, 2> d_xySliceForZ(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<TSim, 2> d_xySliceForZM1(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<TSim, 2> d_xSliceBestInColSimForZM1(CudaSize<2>(volDimX, 1));

    int zStart = doInvZ ? volDimZ - 1 : 0;

    // Copy the first Z plane from 'd_volSimT' into 'xysliceForZ_dmp'
    volume_getVolumeXYSliceAtZ_kernel<TSim, TSim><<<gridvol, blockvol>>>(
        d_xySliceForZ.getBuffer(),
        d_xySliceForZ.getPitch(),
        d_volSimT.getBuffer(),
        d_volSimT.getBytesPaddedUpToDim(1),
        d_volSimT.getBytesPaddedUpToDim(0),
        volDimX, volDimY, volDimZ, zStart); // Z=0
    CHECK_CUDA_ERROR();

    // Set the first Z plane from 'd_volSimT' to 255
    volume_initVolume_kernel<TSim><<<gridvol, blockvol>>>(
        d_volSimT.getBuffer(),
        d_volSimT.getBytesPaddedUpToDim(1),
        d_volSimT.getBytesPaddedUpToDim(0),
        volDimX, volDimY, volDimZ, 0, TSim(255.0f));
    CHECK_CUDA_ERROR();

    for(int iz = 1; iz < volDimZ; iz++)
    {
        int z = doInvZ ? volDimZ - 1 - iz : iz;

        copy(d_xySliceForZM1, d_xySliceForZ);

        // For each column: compute the best score
        // Foreach x:
        //   d_xSliceBestInColSimForZM1[x] = min(d_xySliceForZ[1:height])
        volume_computeBestXSlice_kernel<<<gridvolrow, blockvolrow>>>(
            d_xySliceForZM1.getBuffer(), d_xySliceForZM1.getPitch(),
            d_xSliceBestInColSimForZM1.getBuffer(),
            volDimX, volDimY);
        CHECK_CUDA_ERROR();

        // Copy the 'z' plane from 'd_volSimT' into 'd_xySliceForZ'
        volume_getVolumeXYSliceAtZ_kernel<TSim, TSim><<<gridvol, blockvol>>>(
            d_xySliceForZ.getBuffer(),
            d_xySliceForZ.getPitch(),
            d_volSimT.getBuffer(),
            d_volSimT.getBytesPaddedUpToDim(1),
            d_volSimT.getBytesPaddedUpToDim(0),
            volDimX, volDimY, volDimZ, z);
        CHECK_CUDA_ERROR();

        volume_agregateCostVolumeAtZinSlices_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            rc_tex,
            d_xySliceForZ.getBuffer(), d_xySliceForZ.getPitch(),              // inout: xySliceForZ
            d_xySliceForZM1.getBuffer(), d_xySliceForZM1.getPitch(),          // in:    xySliceForZM1
            d_xSliceBestInColSimForZM1.getBuffer(),                          // in:    xSliceBestInColSimForZM1
            d_volSimT.getBuffer(), d_volSimT.getBytesPaddedUpToDim(1), d_volSimT.getBytesPaddedUpToDim(0), // out:   volSimT
            volDimX, volDimY, volDimZ,
            z, P1, P2,
            dimTrnX, doInvZ);
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();
    }

    if(verbose)
        printf("ps_aggregatePathVolume done\n");
}

/**
 * @param[out] volAgr_dmp output volume where we will aggregate the best XXX
 * @param[in] d_volSim input similarity volume
 */
void ps_updateAggrVolume(CudaDeviceMemoryPitched<TSim, 3>& volAgr_dmp,
                         const CudaDeviceMemoryPitched<TSim, 3>& d_volSim,
                         int volDimX, int volDimY, int volDimZ,
                         int dimTrnX, int dimTrnY, int dimTrnZ,
                         cudaTextureObject_t rc_tex,
                         unsigned char P1, unsigned char P2, 
                         bool verbose, bool doInvZ, int lastN)
{
    if(verbose)
        printf("ps_updateAggrVolume\n");

    int dimsTrn[3];
    dimsTrn[0] = dimTrnX;
    dimsTrn[1] = dimTrnY;
    dimsTrn[2] = dimTrnZ;
    int volDims[3];
    volDims[0] = volDimX;
    volDims[1] = volDimY;
    volDims[2] = volDimZ;

    int dimsTri[3];
    dimsTri[dimTrnX] = 0;
    dimsTri[dimTrnY] = 1;
    dimsTri[dimTrnZ] = 2;

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    dim3 blockT(block_size, block_size, 1);
    dim3 gridT(divUp(volDims[dimsTrn[0]], block_size), divUp(volDims[dimsTrn[1]], block_size), 1);

    //--------------------------------------------------------------------------------------------------
    // aggregate similarity volume
    // clock_t tall = tic();
    CudaDeviceMemoryPitched<TSim, 3> d_volSimT(
        CudaSize<3>(volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]]));
    for(int z = 0; z < volDimZ; z++)
    {
        volume_transposeVolume_kernel<<<grid, block>>>(
            d_volSimT.getBuffer(),
            d_volSimT.getBytesPaddedUpToDim(1),
            d_volSimT.getBytesPaddedUpToDim(0), // output
            d_volSim.getBuffer(),
            d_volSim.getBytesPaddedUpToDim(1),
            d_volSim.getBytesPaddedUpToDim(0), // input
            volDimX, volDimY, volDimZ,
            dimTrnX, dimTrnY, dimTrnZ,
            z);
    }
    CHECK_CUDA_ERROR();
    // if (verbose) printf("transpose volume gpu elapsed time: %f ms \n", toc(tall));
    // pr_printfDeviceMemoryInfo();

    // clock_t tall = tic();
    ps_aggregatePathVolume(
        d_volSimT, volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]],
        rc_tex,
        P1, P2,
        dimTrnX, doInvZ, verbose);
    // if (verbose) printf("aggregate volume gpu elapsed time: %f ms \n", toc(tall));
    // pr_printfDeviceMemoryInfo();

    // clock_t tall = tic();
    for(int zT = 0; zT < volDims[dimsTrn[2]]; zT++)
    {
        volume_transposeAddAvgVolume_kernel<<<gridT, blockT>>>(
            volAgr_dmp.getBuffer(),
            volAgr_dmp.getBytesPaddedUpToDim(1),
            volAgr_dmp.getBytesPaddedUpToDim(0),
            d_volSimT.getBuffer(),
            d_volSimT.getBytesPaddedUpToDim(1),
            d_volSimT.getBytesPaddedUpToDim(0),
            volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]],
            dimsTri[0], dimsTri[1], dimsTri[2], zT, lastN);
    }
    CHECK_CUDA_ERROR();
    // if (verbose) printf("transpose volume gpu elapsed time: %f ms \n", toc(tall));
    // pr_printfDeviceMemoryInfo();

    if(verbose)
        printf("ps_updateAggrVolume done\n");
}


/**
* @param[in] ps_texs_arr table of image (in Lab colorspace) for all scales
* @param[in] rccam RC camera
* @param[inout] iovol_hmh input similarity volume (after Z reduction)
*/
void ps_SGMoptimizeSimVolume(Pyramids& ps_texs_arr,
                             const CameraStruct& rccam,
                             CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
                             int volDimX, int volDimY, int volDimZ,
                             const std::string& filteringAxes,
                             bool verbose, unsigned char P1, unsigned char P2,
                             int scale, int CUDAdeviceNo, int ncamsAllocated)
{
    clock_t tall = tic();

    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    if (verbose)
        printf("ps_SGMoptimizeSimVolume: Total size of volume map in GPU memory: %f\n", double(volSim_dmp.getBytesPadded())/(1024.0*1024.0));

    // Don't need to initialize this buffer
    // ps_updateAggrVolume multiplies the initial value by npaths, which is 0 at first call
    CudaDeviceMemoryPitched<TSim, 3> volAgr_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));

    // update aggregation volume
    int npaths = 0;
    cudaTextureObject_t rc_tex = ps_texs_arr[rccam.camId][scale - 1].tex;

    const auto updateAggrVolume = [&](const std::array<int, 3>& dimTrn, bool invZ) 
                                  {
                                      ps_updateAggrVolume(volAgr_dmp,
                                                          volSim_dmp,
                                                          volDimX, volDimY, volDimZ,
                                                          dimTrn[0], dimTrn[1], dimTrn[2],
                                                          rc_tex,
                                                          P1, P2, verbose,
                                                          invZ,
                                                          npaths);
                                      npaths++;
                                  };
    // Filtering is done on the last axis
    const std::map<char, std::array<int, 3>> mapAxes = {
        {'Y', {0, 2, 1}}, // XYZ -> XZY
        {'X', {1, 2, 0}}, // XYZ -> YZX
        {'Z', {0, 1, 2}}, // XYZ
    };

    for (char axis : filteringAxes)
    {
        updateAggrVolume(mapAxes.at(axis), false); // without transpose
        updateAggrVolume(mapAxes.at(axis), true); // with transpose of the last axis
    }

    if(verbose)
        printf("SGM volume gpu elapsed time: %f ms \n", toc(tall));

    volSim_dmp.copyFrom(volAgr_dmp);

    if(verbose)
        printf("ps_SGMoptimizeSimVolume done\n");
}

void ps_SGMretrieveBestDepth(CudaDeviceMemoryPitched<float2, 2>& bestDepth_dmp, CudaDeviceMemoryPitched<TSim, 3>& volSim_dmp,
    int volDimX, int volDimY, int volDimZ, int zBorder)
{
  int block_size = 8;
  dim3 block(block_size, block_size, 1);
  dim3 grid(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

  volume_retrieveBestZ_kernel<<<grid, block>>>(
    bestDepth_dmp.getBuffer(),
    bestDepth_dmp.getBytesPaddedUpToDim(0),
    volSim_dmp.getBuffer(),
    volSim_dmp.getBytesPaddedUpToDim(1),
    volSim_dmp.getBytesPaddedUpToDim(0),
    volDimX, volDimY, volDimZ, zBorder);
}


void ps_transposeVolume(CudaHostMemoryHeap<unsigned char, 3>* ovol_hmh,
                        CudaHostMemoryHeap<unsigned char, 3>* ivol_hmh, int volDimX, int volDimY, int volDimZ,
                        int dimTrnX, int dimTrnY, int dimTrnZ, bool verbose)
{
    clock_t tall = tic();

    CudaDeviceMemoryPitched<unsigned char, 3> volSim_dmp(*ivol_hmh);

    int dimsTrn[3];
    dimsTrn[0] = dimTrnX;
    dimsTrn[1] = dimTrnY;
    dimsTrn[2] = dimTrnZ;
    int volDims[3];
    volDims[0] = volDimX;
    volDims[1] = volDimY;
    volDims[2] = volDimZ;

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<unsigned char, 3> volTra_dmp(
        CudaSize<3>(volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]]));
    for(int z = 0; z < volDimZ; z++)
    {
        volume_transposeVolume_kernel<<<grid, block>>>(volTra_dmp.getBuffer(),
                                                       volTra_dmp.getBytesPaddedUpToDim(1),
                                                       volTra_dmp.getBytesPaddedUpToDim(0),
                                                       volSim_dmp.getBuffer(),
                                                       volSim_dmp.getBytesPaddedUpToDim(1),
                                                       volSim_dmp.getBytesPaddedUpToDim(0),
                                                       volDimX, volDimY, volDimZ,
                                                       dimTrnX, dimTrnY, dimTrnZ,
                                                       z);
        CHECK_CUDA_ERROR();
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));

    copy((*ovol_hmh), volTra_dmp);
}


void ps_initSimilarityVolume(
    CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
    CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
    int volDimX, int volDimY, int volDimZ)
{
  dim3 block(32, 4, 1);

  dim3 grid(divUp(volDimX, block.x),
    divUp(volDimY, block.y),
    volDimZ);

  volume_init_kernel
    <<<grid, block >>>
    (volBestSim_dmp.getBuffer(),
      volBestSim_dmp.getBytesPaddedUpToDim(1),
      volBestSim_dmp.getBytesPaddedUpToDim(0),
      volDimX, volDimY);
  volume_init_kernel
    <<<grid, block >>>
    (volSecBestSim_dmp.getBuffer(),
      volSecBestSim_dmp.getBytesPaddedUpToDim(1),
      volSecBestSim_dmp.getBytesPaddedUpToDim(0),
      volDimX, volDimY);
}

void ps_initColorVolumeFromCamera(
    CudaDeviceMemoryPitched<float4, 3>& volColor_dmp,
    const CameraStruct& rcam,
    const CameraStruct& tcam,
    cudaTextureObject_t tcam_tex,
    const int tcWidth, const int tcHeight,
    const int depthToStart,
    const CudaDeviceMemory<float>& depths_d,
    int volDimX, int volDimY, int usedVolDimZ, int volStepXY)
{
    dim3 block(32, 4, 1);

    dim3 grid(divUp(volDimX, block.x),
        divUp(volDimY, block.y),
        usedVolDimZ);

    ALICEVISION_CU_PRINT_DEBUG("ps_initColorVolumeFromCamera:" << std::endl
        << "\t- tcam.camId: " << tcam.camId << std::endl
        << "\t- block: " << block.x << ", " << block.y << ", " << block.z << std::endl
        << "\t- grid: " << grid.x << ", " << grid.y << ", " << grid.z << std::endl
        << "\t- volDimX: " << volDimX << ", volDimY: " << volDimY << ", volStepXY: " << volStepXY << std::endl
        << "\t- usedVolDimZ: " << usedVolDimZ << std::endl
        << "\t- depthToStart: " << depthToStart << std::endl
        << "\t- depths_d.getUnitsInDim(0): " << depths_d.getUnitsInDim(0) << std::endl
        << "\t- volColor_dmp.getSize(): " << volColor_dmp.getSize()[0] << ", " << volColor_dmp.getSize()[1] << ", " << volColor_dmp.getSize()[2] << std::endl
        << "\t- volColor_dmp.getBytesPaddedUpToDim(1): " << volColor_dmp.getBytesPaddedUpToDim(1) << std::endl
        << "\t- volColor_dmp.getBytesPaddedUpToDim(0): " << volColor_dmp.getBytesPaddedUpToDim(0) << std::endl
        );

    cudaDeviceSynchronize();
    CHECK_CUDA_ERROR();

    volume_initCameraColor_kernel
        <<<grid, block>>>
        (volColor_dmp.getBuffer(),
         volColor_dmp.getBytesPaddedUpToDim(1),
         volColor_dmp.getBytesPaddedUpToDim(0),
         *rcam.param_dev,
         *tcam.param_dev,
         tcam_tex,
         tcWidth, tcHeight,
         depthToStart,
         depths_d.getBuffer(),
         volDimX, volDimY, usedVolDimZ, volStepXY);

    cudaDeviceSynchronize();
    CHECK_CUDA_ERROR();
}


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
    float gammaC, float gammaP)
{

    configure_volume_slice_kernel();

    printf("Start depth: %i, nb depths to search: %i \n", cell.getLowestUsedDepth(), cell.getDepthsToSearch());

    dim3 volume_slice_kernel_grid(
        divUp(volDimX, volume_slice_kernel_block.x),
        divUp(volDimY, volume_slice_kernel_block.y),
        cell.getDepthsToSearch());

    ALICEVISION_CU_PRINT_DEBUG("====================");
    ALICEVISION_CU_PRINT_DEBUG("RC: " << rcam.camId);
    ALICEVISION_CU_PRINT_DEBUG("volume_slice_kernel_grid: " << volume_slice_kernel_grid.x << ", " << volume_slice_kernel_grid.y << ", " << volume_slice_kernel_grid.z);
    ALICEVISION_CU_PRINT_DEBUG("volume_slice_kernel_block: " << volume_slice_kernel_block.x << ", " << volume_slice_kernel_block.y << ", " << volume_slice_kernel_block.z);
    ALICEVISION_CU_PRINT_DEBUG("startDepthIndex: " << cell.getDepthToStart());
    ALICEVISION_CU_PRINT_DEBUG("nbDepthsToSearch: " << cell.getDepthsToSearch());
    ALICEVISION_CU_PRINT_DEBUG("startDepthIndex+nbDepthsToSearch: " << cell.getDepthToStart() + cell.getDepthsToSearch());
    ALICEVISION_CU_PRINT_DEBUG("volDimX: " << volDimX << ", volDimY: " << volDimY << ", volStepXY: " << volStepXY);
    ALICEVISION_CU_PRINT_DEBUG("scale: " << scale);
    ALICEVISION_CU_PRINT_DEBUG("rcWidth / scale: " << rcWidth / scale << "x" << rcHeight / scale);
    ALICEVISION_CU_PRINT_DEBUG("====================");

    cudaDeviceSynchronize();
    CHECK_CUDA_ERROR();

    volume_estimateSim_twoViews_kernel
        <<<volume_slice_kernel_grid, volume_slice_kernel_block>>>
        (rc_tex,
#ifdef PLANE_SWEEPING_PRECOMPUTED_COLORS_TEXTURE
         volTcamColors_tex3D,
#else
         volTcamColors_dmp.getBuffer(),
         volTcamColors_dmp.getBytesPaddedUpToDim(1),
         volTcamColors_dmp.getBytesPaddedUpToDim(0),
#endif
         cell.getDepthToStart(),
         cell.getDepthsToSearch(),
         rcWidth / scale, rcHeight / scale,
         wsh, gammaC, gammaP,
         volBestSim_dmp.getBuffer(),
         volBestSim_dmp.getBytesPaddedUpToDim(1),
         volBestSim_dmp.getBytesPaddedUpToDim(0),
         volSecBestSim_dmp.getBuffer(),
         volSecBestSim_dmp.getBytesPaddedUpToDim(1),
         volSecBestSim_dmp.getBytesPaddedUpToDim(0),
         scale, volStepXY,
         volDimX, volDimY);

    cudaDeviceSynchronize();
    CHECK_CUDA_ERROR();

    cudaDeviceSynchronize();
}


void ps_computeSimilarityVolume(Pyramids& ps_texs_arr,
                                CudaDeviceMemoryPitched<TSim, 3>& volBestSim_dmp,
                                CudaDeviceMemoryPitched<TSim, 3>& volSecBestSim_dmp,
                                const CameraStruct& rcam, int rcWidth, int rcHeight,
                                const CameraStruct& tcam, int tcWidth, int tcHeight,
                                int volStepXY, int volDimX, int volDimY,
                                const CudaDeviceMemory<float>& depths_d,
                                const std::vector<OneTC>& cells,
                                int wsh, int kernelSizeHalf,
                                int scale,
                                bool verbose,
                                float gammaC, float gammaP)
{
    CHECK_CUDA_ERROR();
    cudaDeviceSynchronize();

    configure_volume_slice_kernel();

    int s = scale -1;

    const int max_cells = cells.size();

    const int baseDepth = cells[0].getLowestUsedDepth();  // min of all cells
    const int stopDepth = cells[0].getHighestUsedDepth(); // max of all cells

    TSim* gpu_volume_1st = volBestSim_dmp.getBuffer();
    TSim* gpu_volume_2nd = volSecBestSim_dmp.getBuffer();

    if(verbose)
    {
        for(int ci=0; ci<max_cells; ++ci)
        {
            printf("Nb all depths: %i, start depth: %i, nb depths to search: %i \n",
                    (int)depths_d.getUnitsTotal(),
              cells[ci].getLowestUsedDepth(),
              cells[ci].getDepthsToSearch() );
        }
    }

    for(int ci=0; ci<max_cells; ci++)
    {
      const int startDepthIndex = cells[ci].getDepthToStart();
      const int nbDepthsToSearch = cells[ci].getDepthsToSearch();

      dim3 volume_slice_kernel_grid(
        divUp(volDimX, volume_slice_kernel_block.x),
        divUp(volDimY, volume_slice_kernel_block.y),
        nbDepthsToSearch);

      ALICEVISION_CU_PRINT_DEBUG("====================");
      ALICEVISION_CU_PRINT_DEBUG("RC: " << rcam.camId << ", TC: " << tcam.camId);
      ALICEVISION_CU_PRINT_DEBUG("volume_slice_kernel_grid: " << volume_slice_kernel_grid.x << ", " << volume_slice_kernel_grid.y << ", " << volume_slice_kernel_grid.z);
      ALICEVISION_CU_PRINT_DEBUG("volume_slice_kernel_block: " << volume_slice_kernel_block.x << ", " << volume_slice_kernel_block.y << ", " << volume_slice_kernel_block.z);
      ALICEVISION_CU_PRINT_DEBUG("startDepthIndex: " << startDepthIndex);
      ALICEVISION_CU_PRINT_DEBUG("nbDepthsToSearch: " << nbDepthsToSearch);
      ALICEVISION_CU_PRINT_DEBUG("startDepthIndex+nbDepthsToSearch: " << startDepthIndex+nbDepthsToSearch);
      ALICEVISION_CU_PRINT_DEBUG("volDimX: " << volDimX << ", volDimY: " << volDimY);
      ALICEVISION_CU_PRINT_DEBUG("scale-1: " << scale - 1);
      ALICEVISION_CU_PRINT_DEBUG("s: " << s);
      ALICEVISION_CU_PRINT_DEBUG("rcWidth / scale: " << rcWidth / scale << "x" << rcHeight / scale);
      ALICEVISION_CU_PRINT_DEBUG("tcWidth / scale: " << tcWidth / scale << "x" << tcHeight / scale);
      ALICEVISION_CU_PRINT_DEBUG("====================");
      volume_slice_kernel
            <<<volume_slice_kernel_grid, volume_slice_kernel_block>>>
            ( ps_texs_arr[rcam.camId][s].tex,
              ps_texs_arr[tcam.camId][s].tex,
              *rcam.param_dev,
              *tcam.param_dev,
              depths_d.getBuffer(),
              startDepthIndex,
              nbDepthsToSearch,
              rcWidth / scale, rcHeight / scale,
              tcWidth / scale, tcHeight / scale,
              wsh,
              gammaC, gammaP,
              gpu_volume_1st,
              volBestSim_dmp.getBytesPaddedUpToDim(1),
              volBestSim_dmp.getBytesPaddedUpToDim(0),
              gpu_volume_2nd,
              volSecBestSim_dmp.getBytesPaddedUpToDim(1),
              volSecBestSim_dmp.getBytesPaddedUpToDim(0),
              volStepXY,
              volDimX, volDimY);

        cudaDeviceSynchronize();
        CHECK_CUDA_ERROR();
    }

    cudaDeviceSynchronize();
}

void ps_refineRcDepthMap(Pyramids& ps_texs_arr, float* out_osimMap_hmh,
                         float* inout_rcDepthMap_hmh, int ntcsteps,
                         const std::vector<CameraStruct>& cams,
                         int width, int height,
                         int rcWidth, int rcHeight,
                         int tcWidth, int tcHeight,
                         int scale, int CUDAdeviceNo, int ncamsAllocated,
                         bool verbose, int wsh, float gammaC, float gammaP,
                         bool moveByTcOrRc, int xFrom)
{
    // setup block and grid
    dim3 block(16, 16, 1);
    dim3 grid(divUp(width, block.x), divUp(height, block.y), 1);

    cudaTextureObject_t rc_tex = ps_texs_arr[cams[0].camId][scale].tex;
    int t = 1;
    cudaTextureObject_t tc_tex = ps_texs_arr[cams[t].camId][scale].tex;

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(width, height));
    copy(rcDepthMap_dmp, inout_rcDepthMap_hmh, width, height);
    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(width, height));

    clock_t tall = tic();

    for(int i = 0; i < ntcsteps; ++i) // Default ntcsteps = 31
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            *cams[0].param_dev, *cams[t].param_dev,
            rc_tex, tc_tex,
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(),
            width, height, wsh, gammaC, gammaP,
            (float)(i - (ntcsteps - 1) / 2), i, moveByTcOrRc, xFrom,
            rcWidth, rcHeight,
            tcWidth, tcHeight);
    }

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        width, height, 1);

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        *cams[0].param_dev, *cams[t].param_dev, 
        rc_tex, tc_tex,
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        width,
        height, wsh, gammaC, gammaP, -1.0f, moveByTcOrRc, xFrom,
        rcWidth, rcHeight,
        tcWidth, tcHeight);

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        width, height, 0);

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        *cams[0].param_dev, *cams[t].param_dev,
        rc_tex, tc_tex,
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        width,
        height, wsh, gammaC, gammaP, +1.0f, moveByTcOrRc, xFrom,
        rcWidth, rcHeight,
        tcWidth, tcHeight);

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        width, height, 2);

    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        *cams[0].param_dev, *cams[t].param_dev,
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        width, height, moveByTcOrRc, xFrom);

    copy(out_osimMap_hmh, width, height, bestSimMap_dmp);
    copy(inout_rcDepthMap_hmh, width, height, bestDptMap_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

/**
 * @brief ps_fuseDepthSimMapsGaussianKernelVoting
 * @param odepthSimMap_hmh
 * @param depthSimMaps_hmh
 * @param ndepthSimMaps: number of Tc cameras
 * @param nSamplesHalf (default value 150)
 * @param nDepthsToRefine (default value 31)
 * @param sigma
 * @param width
 * @param height
 * @param verbose
 */
void ps_fuseDepthSimMapsGaussianKernelVoting(CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                                             std::vector<CudaHostMemoryHeap<float2, 2>*>& depthSimMaps_hmh, int ndepthSimMaps,
                                             int nSamplesHalf, int nDepthsToRefine, float sigma, int width, int height,
                                             bool verbose)
{
    clock_t tall = tic();

    float samplesPerPixSize = (float)(nSamplesHalf / ((nDepthsToRefine - 1) / 2));
    float twoTimesSigmaPowerTwo = 2.0f * sigma * sigma;

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    CudaDeviceMemoryPitched<float2, 2> bestDepthSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> bestGsvSampleMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> gsvSampleMap_dmp(CudaSize<2>(width, height));
    std::vector<CudaDeviceMemoryPitched<float2, 2>*> depthSimMaps_dmp(ndepthSimMaps);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        depthSimMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*depthSimMaps_dmp[i]), (*depthSimMaps_hmh[i]));
    }

    for(int s = -nSamplesHalf; s <= nSamplesHalf; s++) // (-150, 150)
    {
        for(int c = 1; c < ndepthSimMaps; c++) // number of Tc cameras
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
            width, height, (float)s, s + nSamplesHalf);
    }

    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
        bestDepthSimMap_dmp.getBuffer(), bestDepthSimMap_dmp.getPitch(),
        bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.getPitch(),
        depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->getPitch(),
        width, height, samplesPerPixSize);

    copy((*odepthSimMap_hmh), bestDepthSimMap_dmp);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        delete depthSimMaps_dmp[i];
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_optimizeDepthSimMapGradientDescent(Pyramids& ps_texs_arr,
                                           CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                                           std::vector<CudaHostMemoryHeap<float2, 2>*>& dataMaps_hmh, int ndataMaps,
                                           int nSamplesHalf, int nDepthsToRefine, int nIters, float sigma,
                                           const std::vector<CameraStruct>& cams,
                                           int ncams, int width, int height, int scale,
                                           int CUDAdeviceNo, int ncamsAllocated, bool verbose, int yFrom)
{
    clock_t tall = tic();

    float samplesPerPixSize = (float)(nSamplesHalf / ((nDepthsToRefine - 1) / 2));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    std::vector<CudaDeviceMemoryPitched<float2, 2>*> dataMaps_dmp(ndataMaps);
    for(int i = 0; i < ndataMaps; i++)
    {
        dataMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*dataMaps_dmp[i]), (*dataMaps_hmh[i]));
    }

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(CudaSize<2>(width, height));
    CudaArray<float, 2> optDepthMap_arr(CudaSize<2>(width, height));
    copy(optDepthSimMap_dmp, (*dataMaps_dmp[0]));

    cudaTextureObject_t rc_tex = ps_texs_arr[cams[0].camId][scale].tex;

    CudaDeviceMemoryPitched<float, 2> variance_dmp(CudaSize<2>(width, height));
    cudaTextureObject_t varianceTex = 0;

    {
        const dim3 lblock(32, 2, 1);
        const dim3 lgrid(divUp(width, block.x), divUp(height, block.y), 1);

        compute_varLofLABtoW_kernel<<<lgrid, lblock>>>
            (rc_tex,
            variance_dmp.getBuffer(), variance_dmp.getPitch(),
            width, height);

        cudaTextureDesc  tex_desc;
        memset(&tex_desc, 0, sizeof(cudaTextureDesc));
        tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
        tex_desc.addressMode[0] = cudaAddressModeClamp;
        tex_desc.addressMode[1] = cudaAddressModeClamp;
        tex_desc.addressMode[2] = cudaAddressModeClamp;
        tex_desc.readMode = cudaReadModeElementType;
        tex_desc.filterMode = cudaFilterModeLinear; // with interpolation

        cudaResourceDesc res_desc;
        res_desc.resType = cudaResourceTypePitch2D;
        res_desc.res.pitch2D.desc = cudaCreateChannelDesc<float>();
        res_desc.res.pitch2D.devPtr = variance_dmp.getBuffer();
        res_desc.res.pitch2D.width = variance_dmp.getSize()[0];
        res_desc.res.pitch2D.height = variance_dmp.getSize()[1];
        res_desc.res.pitch2D.pitchInBytes = variance_dmp.getPitch();

        // create texture object: we only have to do this once!
        cudaCreateTextureObject(&varianceTex, &res_desc, &tex_desc, NULL);
    }

    for(int iter = 0; iter < nIters; iter++) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOPtDepthSimMap_kernel<<<grid, block>>>(
            optDepthMap_dmp.getBuffer(), optDepthMap_dmp.getPitch(),
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
            width, height);

        cudaTextureObject_t depthTex = 0;
        {
            cudaTextureDesc  tex_desc;
            memset(&tex_desc, 0, sizeof(cudaTextureDesc));
            tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
            tex_desc.addressMode[0] = cudaAddressModeClamp;
            tex_desc.addressMode[1] = cudaAddressModeClamp;
            tex_desc.addressMode[2] = cudaAddressModeClamp;
            tex_desc.readMode = cudaReadModeElementType;
            tex_desc.filterMode = cudaFilterModePoint;

            cudaResourceDesc res_desc;
            res_desc.resType = cudaResourceTypePitch2D;
            res_desc.res.pitch2D.desc = cudaCreateChannelDesc<float>();
            res_desc.res.pitch2D.devPtr = optDepthMap_dmp.getBuffer();
            res_desc.res.pitch2D.width = optDepthMap_dmp.getSize()[0];
            res_desc.res.pitch2D.height = optDepthMap_dmp.getSize()[1];
            res_desc.res.pitch2D.pitchInBytes = optDepthMap_dmp.getPitch();

            // create texture object: we only have to do this once!
            cudaCreateTextureObject(&depthTex, &res_desc, &tex_desc, NULL);
        }

        // Adjust depth/sim by using previously computed depths
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(
            rc_tex, *cams[0].param_dev,
            varianceTex, depthTex,
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
            dataMaps_dmp[0]->getBuffer(), dataMaps_dmp[0]->getPitch(),
            dataMaps_dmp[1]->getBuffer(), dataMaps_dmp[1]->getPitch(),
            width, height, iter, samplesPerPixSize, yFrom);

        cudaDestroyTextureObject(depthTex);
    }

    copy((*odepthSimMap_hmh), optDepthSimMap_dmp);

    cudaDestroyTextureObject(varianceTex);

    for(int i = 0; i < ndataMaps; i++)
    {
        delete dataMaps_dmp[i];
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// uchar4 with 0..255 components => float3 with 0..1 components
inline __device__ __host__ float3 uchar4_to_float3(const uchar4 c)
{
    return make_float3(float(c.x) / 255.0f, float(c.y) / 255.0f, float(c.z) / 255.0f);
}

void ps_getSilhoueteMap(Pyramids& ps_texs_arr, CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
                        int height, int scale,
                        int step, int camId,
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

    CudaDeviceMemoryPitched<bool, 2> map_dmp(CudaSize<2>(width / step, height / step));
    getSilhoueteMap_kernel<<<grid, block>>>(
        ps_texs_arr[camId][scale].tex,
        map_dmp.getBuffer(), map_dmp.getPitch(),
        step, width, height, maskColorLab);
    CHECK_CUDA_ERROR();

    copy((*omap_hmh), map_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}


void ps_computeNormalMap(
    Pyramids& ps_texs_arr,
    CudaHostMemoryHeap<float3, 2>& normalMap_hmh,
    CudaHostMemoryHeap<float, 2>& depthMap_hmh,
    const CameraStruct& camera, int width, int height,
    int scale, int ncamsAllocated, int scales, int wsh, bool verbose,
    float gammaC, float gammaP)
{
  clock_t tall = tic();

  CudaDeviceMemoryPitched<float, 2> depthMap_dmp(depthMap_hmh);
  CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(normalMap_hmh);

  int block_size = 8;
  dim3 block(block_size, block_size, 1);
  dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

  if (verbose)
    printf("computeNormalMap_kernel\n");

  // compute normal map
  computeNormalMap_kernel<<<grid, block>>>(
    *camera.param_dev,
    depthMap_dmp.getBuffer(),
    depthMap_dmp.getPitch(),
    normalMap_dmp.getBuffer(),
    normalMap_dmp.getPitch(),
    width, height, wsh,
    gammaC, gammaP);

  cudaThreadSynchronize();
  CHECK_CUDA_ERROR();

  if (verbose)
    printf("copy normal map to host\n");

  copy(normalMap_hmh, normalMap_dmp);
  CHECK_CUDA_ERROR();

  if (verbose)
    printf("gpu elapsed time: %f ms \n", toc(tall));
}

} // namespace depthMap
} // namespace aliceVision
