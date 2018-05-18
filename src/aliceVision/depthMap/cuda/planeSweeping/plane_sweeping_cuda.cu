// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp"

#include "aliceVision/depthMap/cuda/commonStructures.hpp"

#include "aliceVision/depthMap/cuda/planeSweeping/cuda_global_data.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_code.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_code_refine.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_code_volume.cuh"
#include "aliceVision/depthMap/cuda/planeSweeping/device_code_fuse.cuh"

#include "aliceVision/depthMap/cuda/deviceCommon/device_color.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_patch_es.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_eig33.cuh"
#include "aliceVision/depthMap/cuda/deviceCommon/device_global.cuh"

#include <math_constants.h>
#include <iostream>
#include <map>

#include <algorithm>

namespace aliceVision {
namespace depthMap {

__host__ float3 ps_M3x3mulV3(float* M3x3, const float3& V)
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

    float avail = (float)iavail / (1024.0f * 1024.0f);
    float total = (float)itotal / (1024.0f * 1024.0f);
    float used = (float)iused / (1024.0f * 1024.0f);

    int CUDAdeviceNo;
    cudaGetDevice(&CUDAdeviceNo);

    printf("Device %i memory - used: %f, free: %f, total: %f\n", CUDAdeviceNo, used, avail, total);
}

float3 ps_getDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;
    cudaMemGetInfo(&iavail, &itotal);
    size_t iused = itotal - iavail;

    float avail = (float)iavail / (1024.0f * 1024.0f);
    float total = (float)itotal / (1024.0f * 1024.0f);
    float used = (float)iused / (1024.0f * 1024.0f);

    return make_float3(avail, total, used);
}

__host__ void ps_init_reference_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                                float* _C)
{
    cudaMemcpyToSymbol(sg_s_rP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(sg_s_riP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_riR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_riK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_rC, _C, sizeof(float) * 3);

    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    float3 _rZVect = ps_M3x3mulV3(_iR, z);
    ps_normalize(_rZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    float3 _rYVect = ps_M3x3mulV3(_iR, y);
    ps_normalize(_rYVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    float3 _rXVect = ps_M3x3mulV3(_iR, x);
    ps_normalize(_rXVect);

    cudaMemcpyToSymbol(sg_s_rXVect, &_rXVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_rYVect, &_rYVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_rZVect, &_rZVect, sizeof(float) * 3);
}

__host__ void ps_init_target_camera_matrices(float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                                             float* _C)
{
    cudaMemcpyToSymbol(sg_s_tP, _P, sizeof(float) * 3 * 4);
    cudaMemcpyToSymbol(sg_s_tiP, _iP, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tR, _R, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tiR, _iR, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tK, _K, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tiK, _iK, sizeof(float) * 3 * 3);
    cudaMemcpyToSymbol(sg_s_tC, _C, sizeof(float) * 3);

    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    float3 _tZVect = ps_M3x3mulV3(_iR, z);
    ps_normalize(_tZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    float3 _tYVect = ps_M3x3mulV3(_iR, y);
    ps_normalize(_tYVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    float3 _tXVect = ps_M3x3mulV3(_iR, x);
    ps_normalize(_tXVect);

    cudaMemcpyToSymbol(sg_s_tXVect, &_tXVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_tYVect, &_tYVect, sizeof(float) * 3);
    cudaMemcpyToSymbol(sg_s_tZVect, &_tZVect, sizeof(float) * 3);
}

__host__ GaussianArray* ps_create_gaussian_arr(float delta, int radius)
{
    std::cerr << "Fetching Gaussian table for radius " << radius << " and delta " << delta << std::endl;
    return global_data.getGaussianArray( delta, radius );
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

void ps_deviceAllocate( int ncams, int width, int height, int scales,
                        int deviceId)
{
    static int counter = 1;
    std::cerr << "    INFO " << __FUNCTION__ << " Call " << counter++ << " to ps_deviceAllocate" << std::endl;
    int num_gpus = 0;
    cudaGetDeviceCount(&num_gpus);


    int outval = cudaSetDevice(deviceId);
    printf("CUDA device no %i for %i\n", outval, deviceId);

    // printf("ps_deviceAllocate\n");
    // pr_printfDeviceMemoryInfo();

    pr_printfDeviceMemoryInfo();

    ///////////////////////////////////////////////////////////////////////////////
    // copy textures to the device
    global_data.allocScaledPictureArrays( scales, ncams, width, height );

    // int allBytes = 0;
    // (*ps_texs_arr) = new CudaArray<uchar4, 2>*[scales * ncams];
    // for(int c = 0; c < ncams; c++)
    // {
    //     for(int s = 0; s < scales; s++)
    //     {
    //         int w = width / (s + 1);
    //         int h = height / (s + 1);
    //         (*ps_texs_arr)[c * scales + s] = new CudaArray<uchar4, 2>(CudaSize<2>(w, h));
    //         allBytes += (*ps_texs_arr)[c * scales + s]->getBytes();
    //     }
    // }
    // CHECK_CUDA_ERROR();

    // calcDCTCoefficients();
    // CHECK_CUDA_ERROR();

    // printf("scales %i\n",scales);
    // cudaThreadSynchronize();

    // printf("total size of preallocated images in GPU memory: %f\n",(float)allBytes/(1024.0f*1024.0f));

    pr_printfDeviceMemoryInfo();
    // printf("ps_deviceAllocate - done\n");
}

void testCUDAdeviceNo(int CUDAdeviceNo)
{
    int myCUDAdeviceNo;
    cudaGetDevice(&myCUDAdeviceNo);
    if(myCUDAdeviceNo != CUDAdeviceNo)
    {
        printf("WARNING different device %i %i\n", myCUDAdeviceNo, CUDAdeviceNo);
    }
}

void ps_deviceUpdateCam( cameraStruct* cam, int camId, int CUDAdeviceNo,
                         int ncamsAllocated, int scales, int w, int h, int varianceWsh)
{
    std::cerr << "    INFO " << __FUNCTION__ << " Calling with scales=" << scales << std::endl;
    testCUDAdeviceNo(CUDAdeviceNo);

    CudaArray<uchar4,2>& array0 = global_data.getScaledPictureArray( 0, camId );
    cudaTextureObject_t r4tex;

    // compute gradient
    {
        CudaDeviceMemoryPitched<uchar4, 2> tex_lab_dmp(CudaSize<2>(w, h));
        std::cerr << "    INFO " << __FUNCTION__ << " Allocating pitch memory with " << w << "X" << h << " entries" << std::endl;
        copy(tex_lab_dmp, (*cam->tex_rgba_hmh)); // copy host to device

        int block_size = 8;
        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(w, block_size), divUp(h, block_size), 1);
        rgb2lab_kernel<<<grid, block>>>(tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0], w, h);

        // copy((*ps_texs_arr[camId * scales + 0]), tex_lab_dmp);
        copy( array0, tex_lab_dmp );

        if(varianceWsh > 0)
        {
            r4tex = global_data.getScaledPictureTex( 0, camId );
            compute_varLofLABtoW_kernel<<<grid, block>>>(
                    r4tex,
                    tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0], w, h,
                    varianceWsh);
            copy( array0, tex_lab_dmp );
        }
    }

    r4tex = global_data.getScaledPictureTex( 0, camId );

    // for each scale
    for(int scale = 1; scale < scales; scale++)
    {
        int radius = scale + 1;
        GaussianArray* gaussian_arr = ps_create_gaussian_arr(1.0f, radius);

        int block_size = 8;
        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(w / (scale + 1), block_size), divUp(h / (scale + 1), block_size), 1);

        CudaDeviceMemoryPitched<uchar4, 2> tex_lab_dmp(CudaSize<2>(w / (scale + 1), h / (scale + 1)));
        std::cerr << "    INFO " << __FUNCTION__ << " Allocating pitch memory with " << w / (scale + 1) << "X" << h / (scale + 1) << " entries" << std::endl;
        // downscale_bilateral_smooth_lab_kernel<<<grid, block>>>
        // downscale_mean_smooth_lab_kernel<<<grid, block>>>
        downscale_gauss_smooth_lab_kernel<<<grid, block>>>(
            gaussian_arr->tex,
            r4tex,
            tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0], w / (scale + 1), h / (scale + 1), scale + 1,
            radius //, 15.5f
            );

        CudaArray<uchar4,2>& array = global_data.getScaledPictureArray( scale, camId );

        // copy((*ps_texs_arr[camId * scales + scale]), tex_lab_dmp);
        copy( array, tex_lab_dmp);

        if(varianceWsh > 0)
        {
            r4tex = global_data.getScaledPictureTex( scale, camId );
            compute_varLofLABtoW_kernel<<<grid, block>>>(
                r4tex,
                tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0],
                w / (scale + 1), h / (scale + 1), varianceWsh);
            // copy((*ps_texs_arr[camId * scales + scale]), tex_lab_dmp);
            copy( array, tex_lab_dmp );
            r4tex = global_data.getScaledPictureTex( 0, camId );
        }
    }

    CHECK_CUDA_ERROR();
}

void ps_deviceDeallocate( int CUDAdeviceNo, int ncams, int scales)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    global_data.freeScaledPictureArrays();
}

void ps_aggregatePathVolume2(CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp, int volDimX, int volDimY,
                             int volDimZ, float P1, float P2, bool transfer)
{

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    int block_sizenmxs = 64;
    dim3 blockvolrow(block_sizenmxs, 1, 1);
    dim3 gridvolrow(divUp(volDimX, block_sizenmxs), 1, 1);
    dim3 gridvolrowAllCols(divUp(volDimX, block_sizenmxs), volDimY, 1);

    auto xyslice_dmp = global_data.pitched_mem_uchar_point_tex_cache.get( volDimX, volDimY );

    CudaDeviceMemoryPitched<unsigned char, 2> xsliceBestInColSim_dmp(CudaSize<2>(volDimX, 1));

    for(int z = 0; z < volDimZ; z++)
    {
        volume_agregateCostVolumeAtZ_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0], xsliceBestInColSim_dmp.getBuffer(), volDimX,
            volDimY, volDimZ, z, P1, P2, transfer);
        CHECK_CUDA_ERROR();

        volume_getVolumeXYSliceAtZ_kernel<unsigned char, unsigned char><<<gridvol, blockvol>>>(
            xyslice_dmp->mem->getBuffer(), xyslice_dmp->mem->stride()[0], vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, z);
        CHECK_CUDA_ERROR();

        volume_computeBestXSlice_kernel<<<gridvolrow, blockvolrow>>>(
            xyslice_dmp->tex,
            xsliceBestInColSim_dmp.getBuffer(),
            volDimX, volDimY);
        CHECK_CUDA_ERROR();
    }

    // pr_printfDeviceMemoryInfo();
    global_data.pitched_mem_uchar_point_tex_cache.put( xyslice_dmp );
}

/**
 * @param[inout] d_volSimT similarity volume with some transposition applied
 */
void ps_aggregatePathVolume(
    cudaTextureObject_t r4tex,
    CudaDeviceMemoryPitched<unsigned char, 3>& d_volSimT,
    int volDimX, int volDimY, int volDimZ,
    float P1, float P2, bool transfer,
    int volLUX, int volLUY,
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

    CudaDeviceMemoryPitched<unsigned int, 2> d_xySliceForZ(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<unsigned int, 2> d_xySliceForZM1(CudaSize<2>(volDimX, volDimY));
    // CudaArray<unsigned int, 2> xySliceForZM1_arr(CudaSize<2>(volDimX, volDimY));
    auto xySliceForZM1_arr = global_data.pitched_mem_uint_point_tex_cache.get( volDimX, volDimY );
    cudaTextureObject_t sliceTexUInt = xySliceForZM1_arr->tex;
    CudaDeviceMemoryPitched<unsigned int, 2> d_xSliceBestInColSimForZM1(CudaSize<2>(volDimX, 1));

    // Copy the first Z plane from 'd_volSimT' into 'xysliceForZ_dmp'
    volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned char><<<gridvol, blockvol>>>(
        d_xySliceForZ.getBuffer(), d_xySliceForZ.stride()[0], d_volSimT.getBuffer(), d_volSimT.stride()[1],
        d_volSimT.stride()[0], volDimX, volDimY, volDimZ, 0); // Z=0
    CHECK_CUDA_ERROR();

    // Set the first Z plane from 'd_volSimT' to 255
    volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
        d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], volDimX, volDimY, volDimZ, 0, 255);
    CHECK_CUDA_ERROR();

    for(int z = 1; z < volDimZ; z++)
    {
        copy(d_xySliceForZM1, d_xySliceForZ);
        copy( *xySliceForZM1_arr->mem, d_xySliceForZM1 );
        // For each column: compute the best score
        // Foreach x:
        //   d_xSliceBestInColSimForZM1[x] = min(d_xySliceForZ[1:height])
        volume_computeBestXSliceUInt_kernel<<<gridvolrow, blockvolrow>>>(
            sliceTexUInt,
            d_xSliceBestInColSimForZM1.getBuffer(),
            volDimX, volDimY);
        CHECK_CUDA_ERROR();

        // Copy the 'z' plane from 'd_volSimT' into 'd_xySliceForZ'
        volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned char><<<gridvol, blockvol>>>(
            d_xySliceForZ.getBuffer(), d_xySliceForZ.stride()[0],
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0],
            volDimX, volDimY, volDimZ, z);
        CHECK_CUDA_ERROR();

        volume_agregateCostVolumeAtZinSlices_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            r4tex,
            d_xySliceForZ.getBuffer(), d_xySliceForZ.stride()[0],              // inout: xySliceForZ
            d_xySliceForZM1.getBuffer(), d_xySliceForZM1.stride()[0],          // in:    xySliceForZM1
            d_xSliceBestInColSimForZM1.getBuffer(),                          // in:    xSliceBestInColSimForZM1
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], // out:   volSimT
            volDimX, volDimY, volDimZ,
            z, P1, P2, transfer, volLUX,
            volLUY, dimTrnX, doInvZ);
        CHECK_CUDA_ERROR();
    }

    global_data.pitched_mem_uint_point_tex_cache.put( xySliceForZM1_arr );

    if(verbose)
        printf("ps_aggregatePathVolume done\n");
}

/**
 * @param[out] volAgr_dmp output volume where we will aggregate the best XXX
 * @param[in] d_volSim input similarity volume
 */
void ps_updateAggrVolume(
    cudaTextureObject_t r4tex,
    CudaDeviceMemoryPitched<unsigned char, 3>& volAgr_dmp,
    const CudaDeviceMemoryPitched<unsigned char, 3>& d_volSim,
    int volDimX, int volDimY, int volDimZ,
    int volStepXY, int volLUX, int volLUY,
    int dimTrnX, int dimTrnY, int dimTrnZ,
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
    CudaDeviceMemoryPitched<unsigned char, 3> d_volSimT(
        CudaSize<3>(volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]]));
    for(int z = 0; z < volDimZ; z++)
    {
        volume_transposeVolume_kernel<<<grid, block>>>(
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], // output
            d_volSim.getBuffer(), d_volSim.stride()[1], d_volSim.stride()[0], // input
            volDimX, volDimY, volDimZ,
            dimTrnX, dimTrnY, dimTrnZ,
            z);
    }
    CHECK_CUDA_ERROR();
    // if (verbose) printf("transpose volume gpu elapsed time: %f ms \n", toc(tall));
    // pr_printfDeviceMemoryInfo();

    if(doInvZ == true)
    {
        // clock_t tall = tic();
        for(int z = 0; z < volDims[dimsTrn[2]] / 2; z++)
        {
            volume_shiftZVolumeTempl_kernel<unsigned char><<<gridT, blockT>>>(
                d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0],
                volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]],
                z);
        }
        CHECK_CUDA_ERROR();
        // if (verbose) printf("shift z volume gpu elapsed time: %f ms \n", toc(tall));
    }

    // clock_t tall = tic();
    ps_aggregatePathVolume(
        r4tex,
        d_volSimT, volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]],
        P1, P2, false,
        volLUX, volLUY,
        dimTrnX, doInvZ, verbose);
    // if (verbose) printf("aggregate volume gpu elapsed time: %f ms \n", toc(tall));
    // pr_printfDeviceMemoryInfo();

    if(doInvZ == true)
    {
        // clock_t tall = tic();
        for(int z = 0; z < volDims[dimsTrn[2]] / 2; z++)
        {
            volume_shiftZVolumeTempl_kernel<unsigned char><<<gridT, blockT>>>(
                d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], volDims[dimsTrn[0]],
                volDims[dimsTrn[1]], volDims[dimsTrn[2]], z);
        }
        CHECK_CUDA_ERROR();
        // if (verbose) printf("shift z volume gpu elapsed time: %f ms \n", toc(tall));
    }

    // clock_t tall = tic();
    for(int zT = 0; zT < volDims[dimsTrn[2]]; zT++)
    {
        volume_transposeAddAvgVolume_kernel<<<gridT, blockT>>>(
            volAgr_dmp.getBuffer(), volAgr_dmp.stride()[1], volAgr_dmp.stride()[0],
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0],
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
void ps_SGMoptimizeSimVolume(
                             cameraStruct* rccam,
                             unsigned char* iovol_hmh,
                             int volDimX, int volDimY, int volDimZ,
                             int volStepXY, int volLUX, int volLUY,
                             bool verbose, unsigned char P1, unsigned char P2,
                             int scale, int CUDAdeviceNo, int ncamsAllocated, int scales)
{
    testCUDAdeviceNo(CUDAdeviceNo);
    if(verbose)
        printf("ps_SGMoptimizeSimVolume\n");

    ps_init_reference_camera_matrices(rccam->P, rccam->iP, rccam->R, rccam->iR, rccam->K, rccam->iK, rccam->C);

    // bind 'r4tex' from the image in Lab colorspace at the scale used
    CudaArray<uchar4,2>& array = global_data.getScaledPictureArray( scale, rccam->camId );
    cudaTextureObject_t  r4tex = global_data.getScaledPictureTex( scale, rccam->camId );

    CudaDeviceMemoryPitched<unsigned char, 3> volSim_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));
    copy(volSim_dmp, iovol_hmh, volDimX, volDimY, volDimZ);

    clock_t tall = tic();

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    // printf("total size of volume map in GPU memory: %f\n",(float)d_volSim.getBytes()/(1024.0f*1024.0f));

    // Don't need to initialize this buffer
    // ps_updateAggrVolume multiplies the initial value by npaths, which is 0 at first call
    CudaDeviceMemoryPitched<unsigned char, 3> volAgr_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));
    
    // update aggregation volume
    int npaths = 0;

    const auto updateAggrVolume = [&](int dimTrnX, int dimTrnY, int dimTrnZ, bool invZ) 
                                  {
                                      ps_updateAggrVolume(r4tex,
                                                          volAgr_dmp,
                                                          volSim_dmp,
                                                          volDimX, volDimY, volDimZ,
                                                          volStepXY, volLUX, volLUY,
                                                          dimTrnX, dimTrnY, dimTrnZ,
                                                          P1, P2, verbose,
                                                          invZ,
                                                          npaths);
                                      npaths++;
                                  };

    // XYZ -> XZY
    updateAggrVolume(0, 2, 1, false);
    // XYZ -> XZ'Y
    updateAggrVolume(0, 2, 1, true);
    // XYZ -> YZX
    updateAggrVolume(1, 2, 0, false);
    // XYZ -> YZ'X
    updateAggrVolume(1, 2, 0, true);


    if(verbose)
        printf("SGM volume gpu elapsed time: %f ms \n", toc(tall));

    //--------------------------------------------------------------------------------------------------
    // copy to host
    //copy((*iovol_hmh), volAgr_dmp);
    copy(iovol_hmh, volDimX, volDimY, volDimZ, volAgr_dmp);

    // cudaUnbindTexture(r4tex);

    if(verbose)
        printf("ps_SGMoptimizeSimVolume done\n");
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
        volume_transposeVolume_kernel<<<grid, block>>>(volTra_dmp.getBuffer(), volTra_dmp.stride()[1],
                                                       volTra_dmp.stride()[0], volSim_dmp.getBuffer(),
                                                       volSim_dmp.stride()[1], volSim_dmp.stride()[0], volDimX,
                                                       volDimY, volDimZ, dimTrnX, dimTrnY, dimTrnZ, z);
        CHECK_CUDA_ERROR();
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));

    copy((*ovol_hmh), volTra_dmp);
}

void ps_computeSimilarityVolume(
    CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp, cameraStruct** cams, int ncams,
    int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ, int volLUX,
    int volLUY, int volLUZ, CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
    CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
    int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
    int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, bool doUsePixelsDepths,
    int nbest, bool useTcOrRcPixSize, float gammaC, float gammaP, bool subPixel,
    float epipShift )
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    if(verbose)
        printf("nDepths %i, nDepthsToSearch %i \n", nDepths, nDepthsToSearch);

    auto volPixs_arr = global_data.pitched_mem_int4_point_tex_cache.get(
        volPixs_hmh.getSize()[0],
        volPixs_hmh.getSize()[1] );
    copy( *volPixs_arr->mem, volPixs_hmh );

    auto depths_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depths_hmh.getSize()[0],
        depths_hmh.getSize()[1] );
    copy( *depths_arr->mem, depths_hmh );

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(nDepthsToSearch, block_size), divUp(slicesAtTime, block_size), 1);
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    //--------------------------------------------------------------------------------------------------
    // init similarity volume
    for(int z = 0; z < volDimZ; z++)
    {
        volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
            vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, z, 255 );
        CHECK_CUDA_ERROR();
    }

    //--------------------------------------------------------------------------------------------------
    // compute similarity volume
    CudaDeviceMemoryPitched<unsigned char, 2> slice_dmp(CudaSize<2>(nDepthsToSearch, slicesAtTime));
    for(int t = 0; t < ntimes; t++)
    {
        volume_slice_kernel<<<grid, block>>>(
            r4tex,
            t4tex,
            depths_arr->tex,
            volPixs_arr->tex,
            slice_dmp.getBuffer(), slice_dmp.stride()[0],
            nDepthsToSearch, nDepths,
            slicesAtTime, width, height, wsh, t, npixs, gammaC, gammaP, epipShift);

        volume_saveSliceToVolume_kernel<<<grid, block>>>(
            volPixs_arr->tex,
            vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            slice_dmp.getBuffer(), slice_dmp.stride()[0],
            nDepthsToSearch, nDepths,
            slicesAtTime, width, height, t, npixs, volStepXY,
            volDimX, volDimY, volDimZ, volLUX, volLUY, volLUZ);
        CHECK_CUDA_ERROR();
    }

    global_data.pitched_mem_float_point_tex_cache.put( depths_arr );
    global_data.pitched_mem_int4_point_tex_cache .put( volPixs_arr );

    if(verbose)
        printf("ps_computeSimilarityVolume elapsed time: %f ms \n", toc(tall));
}

float ps_planeSweepingGPUPixelsVolume(
                                      unsigned char* ovol_hmh, cameraStruct** cams, int ncams,
                                      int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ,
                                      int volLUX, int volLUY, int volLUZ, CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
                                      CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
                                      int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
                                      int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                                      bool doUsePixelsDepths, int nbest, bool useTcOrRcPixSize, float gammaC,
                                      float gammaP, bool subPixel, float epipShift)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    CudaDeviceMemoryPitched<unsigned char, 3> volSim_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));

    if(verbose)
        pr_printfDeviceMemoryInfo();
    if(verbose)
        printf("total size of volume map in GPU memory: %f\n", (float)volSim_dmp.getBytes() / (1024.0f * 1024.0f));

    //--------------------------------------------------------------------------------------------------
    // compute similarity volume
    ps_computeSimilarityVolume(
        volSim_dmp, cams, ncams, width, height, volStepXY, volDimX, volDimY,
        volDimZ, volLUX, volLUY, volLUZ, volPixs_hmh, depths_hmh, nDepthsToSearch, slicesAtTime,
        ntimes, npixs, wsh, kernelSizeHalf, nDepths, scale, CUDAdeviceNo, ncamsAllocated, scales,
        verbose, doUsePixelsDepths, nbest, useTcOrRcPixSize, gammaC, gammaP, subPixel, epipShift );

    //--------------------------------------------------------------------------------------------------
    // copy to host
    //copy((*ovol_hmh), volSim_dmp);
    copy(ovol_hmh, volDimX, volDimY, volDimZ, volSim_dmp);

    // pr_printfDeviceMemoryInfo();
    // printf("total size of volume map in GPU memory: %f\n",(float)d_volSim.getBytes()/(1024.0f*1024.0f));

    return (float)volSim_dmp.getBytes() / (1024.0f * 1024.0f);
}

void ps_filterVisTVolume(CudaHostMemoryHeap<unsigned int, 3>* iovol_hmh, int volDimX, int volDimY, int volDimZ,
                         bool verbose)
{
    clock_t tall = tic();
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<int2, 2> xySlice_dmp(CudaSize<2>(volDimX, volDimY));

    CudaDeviceMemoryPitched<unsigned int, 3> ivol_dmp(*iovol_hmh);
    CudaDeviceMemoryPitched<unsigned int, 3> ovol_dmp(ivol_dmp);

    CudaDeviceMemoryPitched<unsigned int, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));

    auto xyslice_arr = global_data.pitched_mem_uint_point_tex_cache.get( volDimX, volDimY );
    cudaTextureObject_t sliceTexUInt = xyslice_arr->tex;

    /*
    //--------------------------------------------------------------------------------------------------
    //init similarity volume
    for (int z=0;z<volDimZ;z++)	{
            volume_initVolume_kernel<unsigned int><<<gridvol, blockvol>>>(
                    ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
                    volDimX, volDimY, volDimZ, z, 0);
            cudaThreadSynchronize();
    }
    */

    for(int K = -3; K <= 3; K++)
    {
        for(int z = 0; z < volDimZ; z++)
        {
            if((z + K >= 0) && (z + K < volDimZ))
            {
                volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned int><<<gridvol, blockvol>>>(
                    xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0],
                    ivol_dmp.getBuffer(), ivol_dmp.stride()[1], ivol_dmp.stride()[0],
                    volDimX, volDimY, volDimZ, z + K );

                copy( *xyslice_arr->mem, xyslice_dmp );
                volume_filter_VisTVolume_kernel<<<gridvol, blockvol>>>(
                    sliceTexUInt,
                    ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
                    volDimX, volDimY, volDimZ, z, K );
                cudaThreadSynchronize();
                CHECK_CUDA_ERROR();
            }
        }
    }

    global_data.pitched_mem_uint_point_tex_cache.put( xyslice_arr );
    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*iovol_hmh), ovol_dmp);

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

void ps_enforceTweigthInVolumeInternal(CudaDeviceMemoryPitched<unsigned int, 3>& ivol_dmp,
                                       CudaDeviceMemoryPitched<unsigned int, 3>& ovol_dmp, int volDimX, int volDimY,
                                       int volDimZ, bool verbose)
{
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<unsigned int, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));
    auto xyslice_arr = global_data.pitched_mem_uint_point_tex_cache.get( volDimX, volDimY );
    cudaTextureObject_t sliceTexUInt = xyslice_arr->tex;

    for(int z = 0; z < volDimZ; z++)
    {
        volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned int><<<gridvol, blockvol>>>(
            xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0],
            ivol_dmp.getBuffer(), ivol_dmp.stride()[1], ivol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, z );

        copy( *xyslice_arr->mem, xyslice_dmp );
        volume_filter_enforceTWeightInVolume_kernel<<<gridvol, blockvol>>>(
            sliceTexUInt,
            ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, z, 3 );
        CHECK_CUDA_ERROR();
    }

    global_data.pitched_mem_uint_point_tex_cache.put( xyslice_arr );
}

void ps_enforceTweigthInVolume(CudaHostMemoryHeap<unsigned int, 3>* iovol_hmh, int volDimX, int volDimY, int volDimZ,
                               bool verbose)
{
    clock_t tall = tic();

    CudaDeviceMemoryPitched<unsigned int, 3> ivol_dmp(*iovol_hmh);

    int dimTrnX = 2;
    int dimTrnY = 1;
    int dimTrnZ = 0;

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
    // transpose volume
    CudaDeviceMemoryPitched<unsigned int, 3> tvol_dmp(
        CudaSize<3>(volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]]));
    for(int z = 0; z < volDimZ; z++)
    {
        volume_transposeVolume_kernel<unsigned int><<<grid, block>>>(
            tvol_dmp.getBuffer(), tvol_dmp.stride()[1], tvol_dmp.stride()[0], ivol_dmp.getBuffer(), ivol_dmp.stride()[1],
            ivol_dmp.stride()[0], volDimX, volDimY, volDimZ, dimTrnX, dimTrnY, dimTrnZ, z);
    }
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    CudaDeviceMemoryPitched<unsigned int, 3> otvol_dmp(tvol_dmp);
    ps_enforceTweigthInVolumeInternal(tvol_dmp, otvol_dmp, volDims[dimsTrn[0]], volDims[dimsTrn[1]],
                                      volDims[dimsTrn[2]], verbose);

    for(int zT = 0; zT < volDims[dimsTrn[2]]; zT++)
    {
        volume_transposeVolume_kernel<unsigned int><<<gridT, blockT>>>(
            ivol_dmp.getBuffer(), ivol_dmp.stride()[1], ivol_dmp.stride()[0], otvol_dmp.getBuffer(), otvol_dmp.stride()[1],
            otvol_dmp.stride()[0], volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]], dimsTri[0], dimsTri[1],
            dimsTri[2], zT);
    }
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*iovol_hmh), ivol_dmp);

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

void ps_computeDP1Volume(CudaHostMemoryHeap<int, 3>* ovol_hmh, CudaHostMemoryHeap<unsigned int, 3>* ivol_hmh,
                         int volDimX, int volDimY, int volDimZ, bool verbose)
{
    clock_t tall = tic();
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<int2, 2> xySlice_dmp(CudaSize<2>(volDimX, volDimY));

    CudaDeviceMemoryPitched<unsigned int, 3> ivol_dmp(*ivol_hmh);
    CudaDeviceMemoryPitched<int, 3> ovol_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));

    for(int z = volDimZ - 1; z >= 0; z--)
    {
        // printf("zglob %i voldimz %i\n",zPart*volDimZpart+z,volDimZ);
        volume_compute_rDP1_kernel<<<gridvol, blockvol>>>(xySlice_dmp.getBuffer(), xySlice_dmp.stride()[0],
                                                          ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
                                                          ivol_dmp.getBuffer(), ivol_dmp.stride()[1], ivol_dmp.stride()[0],
                                                          volDimX, volDimY, volDimZ, z);
        cudaThreadSynchronize();
    }

    /*
    for (int z=0;z<volDimZ;z++)
    {
            volume_compute_DP1_kernel<<<gridvol, blockvol>>>(
                    xySlice_dmp.getBuffer(), xySlice_dmp.stride()[0],
                    ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
                    ivol_dmp.getBuffer(), ivol_dmp.stride()[1], ivol_dmp.stride()[0],
                    volDimX, volDimY, volDimZ, z);
            cudaThreadSynchronize();
    }
    */

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*ovol_hmh), ovol_dmp);

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

void ps_normalizeDP1Volume(CudaHostMemoryHeap<int, 3>** iovols_hmh, int nZparts, int volDimZpart, int volDimX,
                           int volDimY, int volDimZ, bool verbose)
{
    clock_t tall = tic();
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<int2, 2> xySlice_dmp(CudaSize<2>(volDimX, volDimY));

    for(int zPart = 0; zPart < nZparts; zPart++)
    {
        CudaDeviceMemoryPitched<int, 3> iovol_dmp(*iovols_hmh[zPart]);

        //--------------------------------------------------------------------------------------------------
        // update xySlice_dmp
        for(int z = 0; z < volDimZpart; z++)
        {
            volume_compute_rDP1_volume_minMaxMap_kernel<<<gridvol, blockvol>>>(
                xySlice_dmp.getBuffer(), xySlice_dmp.stride()[0], iovol_dmp.getBuffer(), iovol_dmp.stride()[1],
                iovol_dmp.stride()[0], volDimX, volDimY, volDimZpart, z, zPart, volDimZ);
            cudaThreadSynchronize();
        }

    } // for zPart

    for(int zPart = 0; zPart < nZparts; zPart++)
    {
        CudaDeviceMemoryPitched<int, 3> iovol_dmp(*iovols_hmh[zPart]);

        //--------------------------------------------------------------------------------------------------
        // update volume
        for(int z = volDimZpart - 1; z >= 0; z--)
        {
            volume_normalize_rDP1_volume_by_minMaxMap_kernel<<<gridvol, blockvol>>>(
                xySlice_dmp.getBuffer(), xySlice_dmp.stride()[0], iovol_dmp.getBuffer(), iovol_dmp.stride()[1],
                iovol_dmp.stride()[0], volDimX, volDimY, volDimZpart, z, zPart, volDimZ);
            cudaThreadSynchronize();
        }

        //--------------------------------------------------------------------------------------------------
        // copy to host
        copy((*iovols_hmh[zPart]), iovol_dmp);

    } // for zPart

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

void ps_computeRcVolumeForTcDepthSimMaps(CudaHostMemoryHeap<unsigned int, 3>** ovols_hmh, int nZparts,
                                         int volDimZpart, cameraStruct** cams, int ncams, float2* camsMinMaxFpDepths,
                                         int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ,
                                         CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                                         int CUDAdeviceNo, int ncamsAllocated, int scales,
                                         CudaHostMemoryHeap<float2, 2>** rcTcsDepthSimMaps_hmh, bool verbose,
                                         float maxTcRcPixSizeInVoxRatio, bool considerNegativeDepthAsInfinity)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);
    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    for(int zPart = 0; zPart < nZparts; zPart++)
    {
        CudaDeviceMemoryPitched<unsigned int, 3> vol_dmp(CudaSize<3>(volDimX, volDimY, volDimZpart));

        //--------------------------------------------------------------------------------------------------
        // init volume
        for(int z = 0; z < volDimZpart; z++)
        {
            volume_initVolume_kernel<unsigned int><<<gridvol, blockvol>>>(
                vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0], volDimX, volDimY, volDimZ, z, 0);
        }
        cudaThreadSynchronize();

        clock_t t1 = tic();

        // indexing from 0 is right ... because we want include RC depth map as well
        for(int c = 0; c < ncams; c++)
        {
            ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                           cams[c]->C);

            auto tcDepthSimMap_dmp = global_data.pitched_mem_float2_point_tex_cache.get(
                rcTcsDepthSimMaps_hmh[c]->getSize()[0],
                rcTcsDepthSimMaps_hmh[c]->getSize()[1] );
            copy( *tcDepthSimMap_dmp->mem, *rcTcsDepthSimMaps_hmh[c] );
            cudaTextureObject_t sliceTexFloat2 = tcDepthSimMap_dmp->tex;

            //--------------------------------------------------------------------------------------------------
            // update volume
            int zFrom = (zPart == 0) ? 1 : 0;
            for(int z = zFrom; z < volDimZpart; z++)
            {
                float fpPlaneDepth = depths_hmh.getBuffer()[zPart * volDimZpart + z];      // tex2D(depthsTex,vz,0);
                float fpPlaneDepthP = depths_hmh.getBuffer()[zPart * volDimZpart + z - 1]; // tex2D(depthsTex,vz-1,0);
                // float stepInDepth = fabs(fpPlaneDepthP-fpPlaneDepth)*1.5f;
                float stepInDepth = fabs(fpPlaneDepthP - fpPlaneDepth);

                float2 tcMinMaxFpDepth = camsMinMaxFpDepths[c];

                volume_updateRcVolumeForTcDepthMap_kernel<<<gridvol, blockvol>>>(
                    sliceTexFloat2,
                    vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
                    volDimX, volDimY, volDimZpart, z,
                    volStepXY, volStepXY, width, height, fpPlaneDepth, stepInDepth, zPart, volDimZ,
                    maxTcRcPixSizeInVoxRatio, considerNegativeDepthAsInfinity, tcMinMaxFpDepth );
            }

            global_data.pitched_mem_float2_point_tex_cache.put( tcDepthSimMap_dmp );
        }

        if(verbose)
            printf("elapsed time: %f ms, ncams %i \n", toc(t1), ncams);

        //--------------------------------------------------------------------------------------------------
        // copy to host
        copy((*ovols_hmh[zPart]), vol_dmp);

    } // for zPart

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

/*
void ps_computeRcVolumeForTcDepthMap(
                CudaHostMemoryHeap<unsigned char, 3>* ovol_hmh,
                cameraStruct **cams, int ncams,
                int width, int height,
                int volStepXY, int volDimX, int volDimY, int volDimZ,
                CudaHostMemoryHeap<float, 2> &depths_hmh,
                int nDepths,
                int scale,
                int scales,
                CudaHostMemoryHeap<float, 2> &tcDepthMap_hmh,
                bool verbose, const float maxTcRcPixSizeInVoxRatio
        )
{
        clock_t tall = tic();

        CudaDeviceMemoryPitched<unsigned char,3>  vol_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));

        if (verbose) printf("nDepths %i \n", nDepths);

        CudaArray<float,2> tcDepthMap_arr(tcDepthMap_hmh);
        cudaBindTextureToArray(sliceTex, tcDepthMap_arr.getArray(), cudaCreateChannelDesc<float>());

        //CudaArray<float,2> depths_arr(depths_hmh);
        //cudaBindTextureToArray(depthsTex, depths_arr.getArray(), cudaCreateChannelDesc<float>());

        int block_size = 8;
        dim3 blockvol(block_size,block_size,1);
        dim3 gridvol(divUp(volDimX, block_size),divUp(volDimY,block_size),1);

        //setup cameras matrices to the constant memory
        ps_init_reference_camera_matrices(cams[0]->P,cams[0]->iP,cams[0]->R,cams[0]->iR,cams[0]->K,cams[0]->iK,cams[0]->C);
        ps_init_target_camera_matrices
(cams[1]->P,cams[1]->iP,cams[1]->R,cams[1]->iR,cams[1]->K,cams[1]->iK,cams[1]->C);

        if (verbose) printf("init gpu elapsed time: %f ms \n", toc(tall));

        clock_t tall = tic();

        //--------------------------------------------------------------------------------------------------
        //init volume
        for (int z=0;z<volDimZ;z++)	{
                volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
                        vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
                        volDimX, volDimY, volDimZ, z, 0);
        }
        cudaThreadSynchronize();

        if (verbose) printf("init volume gpu elapsed time: %f ms \n", toc(tall));

        clock_t tall = tic();

        //--------------------------------------------------------------------------------------------------
        //compute volume
        for (int z=1;z<volDimZ;z++)
        {
                float fpPlaneDepth  = depths_hmh.getBuffer()[z]; //tex2D(depthsTex,vz,0);
                float fpPlaneDepthP = depths_hmh.getBuffer()[z-1]; //tex2D(depthsTex,vz-1,0);
                float step = fabs(fpPlaneDepthP-fpPlaneDepth)*1.5f;

                volume_computeRcVolumeForTcDepthMap_kernel<<<gridvol, blockvol>>>(
                        vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
                        volDimX, volDimY, volDimZ, z, volStepXY, volStepXY, width, height, fpPlaneDepth, step,
maxTcRcPixSizeInVoxRatio);
        }
        cudaThreadSynchronize();


        if (verbose) printf("compute volume gpu elapsed time: %f ms \n", toc(tall));

        clock_t tall = tic();

        //--------------------------------------------------------------------------------------------------
        //copy to host
        copy((*ovol_hmh), vol_dmp);


        //cudaUnbindTexture(depthsTex);
        cudaUnbindTexture(sliceTex);

        if (verbose) printf("save to host gpu elapsed time: %f ms \n", toc(tall));
}
*/

void ps_filterRcIdDepthMapByTcDepthMap(CudaHostMemoryHeap<unsigned short, 2>* rcIdDepthMap_hmh, cameraStruct** cams,
                                       int ncams, int width, int height, int volStepXY, int volDimX, int volDimY,
                                       int volDimZ, CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                                       int CUDAdeviceNo, int ncamsAllocated, int scales,
                                       CudaHostMemoryHeap<float, 2>& tcDepthMap_hmh, bool verbose, int distLimit)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    if(verbose)
        printf("nDepths %i \n", nDepths);

    CudaDeviceMemoryPitched<unsigned short, 2> rcIdDepthMap_dmp(*rcIdDepthMap_hmh);

    auto tcDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        tcDepthMap_hmh.getSize()[0],
        tcDepthMap_hmh.getSize()[1] );

    auto depths_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depths_hmh.getSize()[0],
        depths_hmh.getSize()[1] );

    copy( *tcDepthMap_arr->mem, tcDepthMap_hmh );
    copy( *depths_arr->mem, depths_hmh );

    cudaTextureObject_t sliceTex  = tcDepthMap_arr->tex;
    cudaTextureObject_t depthsTex = depths_arr->tex;

    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    ps_init_target_camera_matrices(cams[1]->P, cams[1]->iP, cams[1]->R, cams[1]->iR, cams[1]->K, cams[1]->iK,
                                   cams[1]->C);

    //--------------------------------------------------------------------------------------------------
    // init volume
    volume_filterRcIdDepthMapByTcDepthMap_kernel<<<gridvol, blockvol>>>(
        depthsTex,
        sliceTex,
        rcIdDepthMap_dmp.getBuffer(), rcIdDepthMap_dmp.stride()[0], volDimX, volDimY, volDimZ, volStepXY, volStepXY,
        width, height, distLimit);

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*rcIdDepthMap_hmh), rcIdDepthMap_dmp);

    global_data.pitched_mem_float_point_tex_cache.put( tcDepthMap_arr );
    global_data.pitched_mem_float_point_tex_cache.put( depths_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_filterRcIdDepthMapByTcDepthMaps(CudaHostMemoryHeap<unsigned short, 2>* nModalsMap_hmh,
                                        CudaHostMemoryHeap<unsigned short, 2>* rcIdDepthMap_hmh, cameraStruct** cams,
                                        int ncams, int width, int height, int volStepXY, int volDimX, int volDimY,
                                        int volDimZ, CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                                        int CUDAdeviceNo, int ncamsAllocated, int scales,
                                        CudaHostMemoryHeap<float, 2>** tcDepthMaps_hmh, bool verbose, int distLimit)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    CudaDeviceMemoryPitched<unsigned short, 2> rcIdDepthMap_dmp(*rcIdDepthMap_hmh);
    CudaDeviceMemoryPitched<unsigned short, 2> nModalsMap_dmp(CudaSize<2>(volDimX, volDimY));
    // CudaArray<float,2>						tcDepthMap_arr(CudaSize<2>(volDimX, volDimY));

    auto depths_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depths_hmh.getSize()[0],
        depths_hmh.getSize()[1] );
    copy( *depths_arr->mem, depths_hmh );
    cudaTextureObject_t depthsTex = depths_arr->tex;

    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    volume_update_nModalsMap_kernel_id0<<<gridvol, blockvol>>>(
        nModalsMap_dmp.getBuffer(), nModalsMap_dmp.stride()[0],
        volDimX, volDimY );

    for(int c = 1; c < ncams; c++)
    {
        ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                       cams[c]->C);

        auto tcDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
            tcDepthMaps_hmh[c - 1]->getSize()[0],
            tcDepthMaps_hmh[c - 1]->getSize()[1] );
        copy( *tcDepthMap_arr->mem, *tcDepthMaps_hmh[c - 1] );
        cudaTextureObject_t sliceTex = tcDepthMap_arr->tex;

        volume_update_nModalsMap_kernel<<<gridvol, blockvol>>>(
            depthsTex,
            sliceTex,
            nModalsMap_dmp.getBuffer(), nModalsMap_dmp.stride()[0],
            rcIdDepthMap_dmp.getBuffer(), rcIdDepthMap_dmp.stride()[0],
            volDimX, volDimY, volDimZ, volStepXY, volStepXY, width, height, distLimit, c );

        global_data.pitched_mem_float_point_tex_cache.put( tcDepthMap_arr );
    }

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*nModalsMap_hmh), nModalsMap_dmp);

    global_data.pitched_mem_float_point_tex_cache.put( depths_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_planeSweepNPlanes(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* osim_hmh,
void ps_planeSweepNPlanes( CudaHostMemoryHeap<float, 2>* osim_hmh,
                          CudaHostMemoryHeap<float, 2>* odpt_hmh, float* depths, int ndepths, cameraStruct** cams,
                          int ncams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                          bool verbose, int step)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    auto rimg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    auto timg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );

    getRefTexLAB_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        rimg_dmp->mem->getBuffer(), rimg_dmp->mem->stride()[0],
        width, height );

    auto sliceTex_arr = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    cudaTextureObject_t rTexU4 = rimg_dmp->tex;
    cudaTextureObject_t tTexU4 = timg_dmp->tex;

    dim3 block_step(block_size, block_size, 1);
    dim3 grid_step(divUp(width / step, block_size), divUp(height / step, block_size), 1);

    CudaDeviceMemoryPitched<float, 2> sim_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> cst_dmp(CudaSize<2>(width / step, height / step));
    CudaDeviceMemoryPitched<float, 2> bcst_dmp(CudaSize<2>(width / step, height / step));
    CudaDeviceMemoryPitched<float, 2> bdpt_dmp(CudaSize<2>(width / step, height / step));

    copy( *sliceTex_arr->mem, sim_dmp );
    cudaTextureObject_t sliceTex = sliceTex_arr->tex;

    clock_t tall = tic();

    for(int d = 0; d < ndepths; d++)
    {
        float depth = depths[d];

        reprojTarTexLAB_kernel<<<grid, block>>>(
            t4tex,
            timg_dmp->mem->getBuffer(), timg_dmp->mem->stride()[0],
            width, height, depth);
        cudaThreadSynchronize();

        compWshNccSim_kernel<<<grid, block>>>(
            rTexU4,
            tTexU4,
            sim_dmp.getBuffer(), sim_dmp.stride()[0],
            width, height, 2, 1);
        cudaThreadSynchronize();

        copy( *sliceTex_arr->mem, sim_dmp );

        aggrYKNCCSim_kernel<<<grid_step, block_step>>>(
            rTexU4,
            tTexU4,
            sliceTex,
            cst_dmp.getBuffer(), cst_dmp.stride()[0],
            width, height, 8, step,
            15.5f, 8.0f);
        cudaThreadSynchronize();

        updateBestDepth_kernel<<<grid_step, block_step>>>(bcst_dmp.getBuffer(), bcst_dmp.stride()[0],
                                                          bdpt_dmp.getBuffer(), bdpt_dmp.stride()[0], cst_dmp.getBuffer(),
                                                          cst_dmp.stride()[0], width, height, step, depth, d);
        cudaThreadSynchronize();
    }

    if(verbose)
        printf("gpu comp ncc elapsed time: %f ms \n", toc(tall));

    copy((*osim_hmh), bcst_dmp);
    copy((*odpt_hmh), bdpt_dmp);

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
    global_data.pitched_mem_float_point_tex_cache.put( sliceTex_arr );
}

void ps_planeSweepAggr(CudaHostMemoryHeap<uchar4, 2>& rimg_hmh, CudaHostMemoryHeap<uchar4, 2>& timg_hmh,
                       CudaHostMemoryHeap<float, 2>* osim_hmh, CudaHostMemoryHeap<float, 2>* odepth_hmh,
                       float* depths, int ndepths, cameraStruct** rtcams, int width, int height, int scale, int scales,
                       bool verbose)
{
    clock_t tall = tic();

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    dim3 block(BLOCK_DIM, BLOCK_DIM, 1);
    dim3 grid(divUp(width, BLOCK_DIM), divUp(height, BLOCK_DIM), 1);
    dim3 blockT(BLOCK_DIM, BLOCK_DIM, 1);
    dim3 gridT(divUp(height, BLOCK_DIM), divUp(width, BLOCK_DIM), 1);

    int block_row_size = 64;
    dim3 block_row(block_row_size, 1, 1);
    dim3 grid_row(divUp(width, block_row_size), height, 1);
    dim3 block_col(block_row_size, 1, 1);
    dim3 grid_col(divUp(height, block_row_size), width, 1);

    ///////////////////////////////////////////////////////////////////////////////
    // copy images
    CudaDeviceMemoryPitched<uchar4, 2> rimg_dmp(CudaSize<2>(width, height));
    copy(rimg_dmp, rimg_hmh);
    CudaDeviceMemoryPitched<uchar4, 2> timg_dmp(CudaSize<2>(width, height));
    copy(timg_dmp, timg_hmh);

    auto rtex = global_data.pitched_mem_uchar_linear_tex_cache.get( width, height );
    auto gtex = global_data.pitched_mem_uchar_linear_tex_cache.get( width, height );
    auto btex = global_data.pitched_mem_uchar_linear_tex_cache.get( width, height );

    CudaDeviceMemoryPitched<unsigned char, 2> tex_dmp(CudaSize<2>(width, height));

    copyUchar4Dim2uchar_kernel<<<grid, block>>>(0, timg_dmp.getBuffer(), timg_dmp.stride()[0], tex_dmp.getBuffer(),
                                                tex_dmp.stride()[0], width, height);
    copy( *rtex->mem, tex_dmp );

    copyUchar4Dim2uchar_kernel<<<grid, block>>>(1, timg_dmp.getBuffer(), timg_dmp.stride()[0], tex_dmp.getBuffer(),
                                                tex_dmp.stride()[0], width, height);
    copy( *gtex->mem, tex_dmp );

    copyUchar4Dim2uchar_kernel<<<grid, block>>>(2, timg_dmp.getBuffer(), timg_dmp.stride()[0], tex_dmp.getBuffer(),
                                                tex_dmp.stride()[0], width, height);
    copy( *btex->mem, tex_dmp );

    if(verbose)
        printf("bind\n");

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices(rtcams[0]->P, rtcams[0]->iP, rtcams[0]->R, rtcams[0]->iR, rtcams[0]->K,
                                      rtcams[0]->iK, rtcams[0]->C);
    ps_init_target_camera_matrices(rtcams[1]->P, rtcams[1]->iP, rtcams[1]->R, rtcams[1]->iR, rtcams[1]->K,
                                   rtcams[1]->iK, rtcams[1]->C);

    CudaDeviceMemoryPitched<float, 2> sim_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> depth_dmp(CudaSize<2>(width, height));

    CudaDeviceMemoryPitched<float4, 2> stat1_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float4, 2> stat2_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float4, 2> stat1T_dmp(CudaSize<2>(height, width));
    CudaDeviceMemoryPitched<float4, 2> stat2T_dmp(CudaSize<2>(height, width));

    CudaDeviceMemoryPitched<uchar4, 2> rimgT_dmp(CudaSize<2>(height, width));
    CudaDeviceMemoryPitched<uchar4, 2> timgT_dmp(CudaSize<2>(height, width));

    transpose_uchar4_kernel<<<grid, block>>>(rimg_dmp.getBuffer(), rimg_dmp.stride()[0], rimgT_dmp.getBuffer(),
                                             rimgT_dmp.stride()[0], width, height);
    cudaThreadSynchronize();

    for(int d = 0; d < ndepths; d++)
    {
        float depth = depths[d];

        reprojTarTexRgb_kernel<<<grid, block>>>(
            rtex->tex,
            gtex->tex,
            btex->tex,
            timg_dmp.getBuffer(), timg_dmp.stride()[0],
            width, height, depth);

        transpose_uchar4_kernel<<<grid, block>>>(
            timg_dmp.getBuffer(), timg_dmp.stride()[0], timgT_dmp.getBuffer(),
            timgT_dmp.stride()[0], width, height);

        compAggrNccSim_kernel<<<grid_row, block_row>>>(
            stat1_dmp.getBuffer(), stat1_dmp.stride()[0],
            stat2_dmp.getBuffer(), stat2_dmp.stride()[0],
            rimg_dmp.getBuffer(), rimg_dmp.stride()[0],
            timg_dmp.getBuffer(), timg_dmp.stride()[0],
            width, height, 0, 0);

        compAggrNccSim_kernel<<<grid_row, block_row>>>(
            stat1_dmp.getBuffer(), stat1_dmp.stride()[0],
            stat2_dmp.getBuffer(), stat2_dmp.stride()[0],
            rimg_dmp.getBuffer(), rimg_dmp.stride()[0],
            timg_dmp.getBuffer(), timg_dmp.stride()[0],
            width, height, 32, 0);

        transpose_float4_kernel<<<grid, block>>>(
            stat1_dmp.getBuffer(), stat1_dmp.stride()[0],
            stat1T_dmp.getBuffer(), stat1T_dmp.stride()[0],
            width, height);

        transpose_float4_kernel<<<grid, block>>>(
            stat2_dmp.getBuffer(), stat2_dmp.stride()[0],
            stat2T_dmp.getBuffer(), stat2T_dmp.stride()[0],
            width, height);

        compAggrNccSim_kernel<<<grid_col, block_col>>>(
            stat1T_dmp.getBuffer(), stat1T_dmp.stride()[0],
            stat2T_dmp.getBuffer(), stat2T_dmp.stride()[0],
            rimgT_dmp.getBuffer(), rimgT_dmp.stride()[0],
            timgT_dmp.getBuffer(), timgT_dmp.stride()[0],
            height, width, 0, 1);

        compAggrNccSim_kernel<<<grid_col, block_col>>>(
            stat1T_dmp.getBuffer(), stat1T_dmp.stride()[0],
            stat2T_dmp.getBuffer(), stat2T_dmp.stride()[0],
            rimgT_dmp.getBuffer(), rimgT_dmp.stride()[0],
            timgT_dmp.getBuffer(), timgT_dmp.stride()[0],
            height, width, 32,
            1);

        transpose_float4_kernel<<<gridT, blockT>>>(
            stat1T_dmp.getBuffer(), stat1T_dmp.stride()[0],
            stat1_dmp.getBuffer(), stat1_dmp.stride()[0],
            height, width);

        transpose_float4_kernel<<<gridT, blockT>>>(
            stat2T_dmp.getBuffer(), stat2T_dmp.stride()[0],
            stat2_dmp.getBuffer(), stat2_dmp.stride()[0],
            height, width);

        compNccSimFromStats_kernel<<<grid, block>>>(
            depth_dmp.getBuffer(), depth_dmp.stride()[0],
            sim_dmp.getBuffer(), sim_dmp.stride()[0],
            stat1_dmp.getBuffer(), stat1_dmp.stride()[0],
            stat2_dmp.getBuffer(), stat2_dmp.stride()[0],
            width, height, d, depth);
    }

    copy((*osim_hmh), sim_dmp);
    copy((*odepth_hmh), depth_dmp);

    global_data.pitched_mem_uchar_linear_tex_cache.put( rtex );
    global_data.pitched_mem_uchar_linear_tex_cache.put( gtex );
    global_data.pitched_mem_uchar_linear_tex_cache.put( btex );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_getTexture(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<uchar4, 2>* oimg_hmh, int camId,
void ps_getTexture( CudaHostMemoryHeap<uchar4, 2>* oimg_hmh, int camId,
                   int scale, int CUDAdeviceNo, int ncamsAllocated, int scales)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    // copy((*oimg_hmh), (*ps_texs_arr[camId * scales + scale]));
    copy( (*oimg_hmh), global_data.getScaledPictureArray( scale, camId ) );
    printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_smoothDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* depthMap_hmh,
void ps_smoothDepthMap( CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                       cameraStruct** cams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                       int scales, int wsh, bool verbose, float gammaC, float gammaP)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    copy( *depthMap_arr->mem, *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;


    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    auto depthMap_dmp = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    if(verbose)
        printf("smoothDepthMap_kernel\n");

    //------------------------------------------------------------------------------------------------
    // smooth depth map
    smoothDepthMap_kernel<<<grid, block>>>(
        r4tex,
        depthsTex,
        depthMap_dmp->mem->getBuffer(), depthMap_dmp->mem->stride()[0],
        width, height, wsh, gammaC,
        gammaP );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("copy depth map to host\n");

    copy( *depthMap_hmh, *depthMap_dmp->mem );

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_dmp );
    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_filterDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* depthMap_hmh,
void ps_filterDepthMap( CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                       cameraStruct** cams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                       int scales, int wsh, bool verbose, float gammaC, float minCostThr)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    copy( *depthMap_arr->mem, *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    // CudaDeviceMemoryPitched<float, 2> depthMap_dmp(CudaSize<2>(width, height));
    auto depthMap_dmp = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    if(verbose)
        printf("smoothDepthMap_kernel\n");

    //------------------------------------------------------------------------------------------------
    // smooth depth map
    filterDepthMap_kernel<<<grid, block>>>(
        r4tex,
        depthsTex,
        depthMap_dmp->mem->getBuffer(), depthMap_dmp->mem->stride()[0],
        width, height, wsh, gammaC,
        minCostThr );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("copy depth map to host\n");

    copy( *depthMap_hmh, *depthMap_dmp->mem );

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_dmp );
    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_computeNormalMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
void ps_computeNormalMap( CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
                         CudaHostMemoryHeap<float, 2>* depthMap_hmh, cameraStruct** cams, int width, int height,
                         int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int wsh, bool verbose,
                         float gammaC, float gammaP)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    copy( *depthMap_arr->mem, *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(*normalMap_hmh);

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    if(verbose)
        printf("computeNormalMap_kernel\n");

    //------------------------------------------------------------------------------------------------
    // compute normal map
    computeNormalMap_kernel<<<grid, block>>>(
        r4tex,
        depthsTex,
        normalMap_dmp.getBuffer(), normalMap_dmp.stride()[0],
        width, height, wsh,
        gammaC, gammaP );
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("copy normal map to host\n");

    copy((*normalMap_hmh), normalMap_dmp);
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );
}

// void ps_alignSourceDepthMapToTarget(CudaArray<uchar4, 2>** ps_texs_arr,
void ps_alignSourceDepthMapToTarget(
                                    CudaHostMemoryHeap<float, 2>* sourceDepthMap_hmh,
                                    CudaHostMemoryHeap<float, 2>* targetDepthMap_hmh, cameraStruct** cams, int width,
                                    int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int wsh,
                                    bool verbose, float gammaC, float maxPixelSizeDist)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto sourceDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        sourceDepthMap_hmh->getSize()[0],
        sourceDepthMap_hmh->getSize()[1] );
    copy( *sourceDepthMap_arr->mem, *sourceDepthMap_hmh );
    cudaTextureObject_t depthsTex = sourceDepthMap_arr->tex;

    auto targetDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        targetDepthMap_hmh->getSize()[0],
        targetDepthMap_hmh->getSize()[1] );
    copy( *targetDepthMap_arr->mem, *targetDepthMap_hmh );
    cudaTextureObject_t depthsTex1 = targetDepthMap_arr->tex;

    CudaDeviceMemoryPitched<float, 2> outDepthMap_dmp(CudaSize<2>(width, height));

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    if(verbose)
        printf("smoothDepthMap_kernel\n");

    //------------------------------------------------------------------------------------------------
    // smooth depth map
    alignSourceDepthMapToTarget_kernel<<<grid, block>>>(
        r4tex,
        depthsTex,
        depthsTex1,
        outDepthMap_dmp.getBuffer(), outDepthMap_dmp.stride()[0],
        width, height, wsh, gammaC, maxPixelSizeDist);
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("copy depth map to host\n");

    copy((*sourceDepthMap_hmh), outDepthMap_dmp);

    global_data.pitched_mem_float_point_tex_cache.put( sourceDepthMap_arr );
    global_data.pitched_mem_float_point_tex_cache.put( targetDepthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ps_dilateDepthMap(CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
                       CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp, int width, int height, bool verbose,
                       int niters, float gammaC)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    copy(odepthMap_dmp, idepthMap_dmp);

    auto sourceDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        odepthMap_dmp.getSize()[0],
        odepthMap_dmp.getSize()[1] );
    cudaTextureObject_t depthsTex = sourceDepthMap_arr->tex;

    for(int iter = 0; iter <= niters; iter++)
    {
        copy( *sourceDepthMap_arr->mem, odepthMap_dmp );

        refine_dilateDepthMap_kernel<<<grid, block>>>(
            depthsTex,
            odepthMap_dmp.getBuffer(), odepthMap_dmp.stride()[0],
            width, height, gammaC );
    }

    global_data.pitched_mem_float_point_tex_cache.put( sourceDepthMap_arr );
}

void ps_dilateMaskMap(CudaDeviceMemoryPitched<float, 2>& depthMap_dmp, int width, int height, bool verbose,
                      int niters, float fpPlaneDepth)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    for(int iter = 0; iter <= niters; iter++)
    {
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, 0, 0, fpPlaneDepth);
        cudaThreadSynchronize();
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, -1, 0, fpPlaneDepth);
        cudaThreadSynchronize();
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, +1, 0, fpPlaneDepth);
        cudaThreadSynchronize();
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, 0, -1, fpPlaneDepth);
        cudaThreadSynchronize();
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, 0, +1, fpPlaneDepth);
        cudaThreadSynchronize();
    }

    refine_convertFPPlaneDepthMapToDepthMap_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                    depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                    width, height);
    cudaThreadSynchronize();
}

void ps_refineDepthMapInternal(
    cudaTextureObject_t t4tex,
    CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
    CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
    CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp,
    CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp, int width, int height,
    bool verbose, int wsh, float gammaC, float gammaP, float simThr,
    CudaDeviceMemoryPitched<float3, 2>& dsm_dmp,
    CudaDeviceMemoryPitched<float3, 2>& ssm_dmp,
    cudaTextureObject_t  rTexU4,
    cudaTextureObject_t& tTexU4,
    PitchedMem_Texture<uchar4,cudaFilterModePoint,cudaReadModeElementType>* timg_dmp,
    // CudaArray<uchar4, 2>& tTexU4_arr,
    // CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp,
    bool moveByTcOrRc, float step)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    // computed three depths map ... -1,0,1 ... from dilated input depth map .. and add last
    refine_computeDepthsMapFromDepthMap_kernel<<<grid, block>>>(
        dsm_dmp.getBuffer(), dsm_dmp.stride()[0],
        idepthMap_dmp.getBuffer(), idepthMap_dmp.stride()[0],
        width, height, moveByTcOrRc, step);
    // cudaThreadSynchronize();

    // for each depth map compute sim map in pixels where depthMap is defined
    // int id = 1;
    for(int id = 0; id < 3; id++)
    {
        refine_reprojTarTexLABByDepthsMap_kernel<<<grid, block>>>(
            t4tex,
            dsm_dmp.getBuffer(), dsm_dmp.stride()[0],
            timg_dmp->mem->getBuffer(), timg_dmp->mem->stride()[0],
            width, height, id );

        tTexU4 = timg_dmp->tex;

        refine_compYKNCCSim_kernel<<<grid, block>>>(
            rTexU4,
            tTexU4,
            ssm_dmp.getBuffer(), ssm_dmp.stride()[0],
            id,
            idepthMapMask_dmp.getBuffer(), idepthMapMask_dmp.stride()[0],
            width, height, wsh, gammaC, gammaP);

    }

    refine_computeBestDepthSimMaps_kernel<<<grid, block>>>(
        osimMap_dmp.getBuffer(), osimMap_dmp.stride()[0],
        odepthMap_dmp.getBuffer(), odepthMap_dmp.stride()[0],
        ssm_dmp.getBuffer(), ssm_dmp.stride()[0],
        dsm_dmp.getBuffer(), dsm_dmp.stride()[0],
        width, height, simThr);
}

void ps_computeSimMapForDepthMapInternal(
    cudaTextureObject_t t4tex,
    CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
    CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp,
    int width, int height,
    bool verbose, int wsh, float gammaC, float gammaP,
    cudaTextureObject_t  rTexU4,
    cudaTextureObject_t& tTexU4,
    // CudaArray<uchar4, 2>& tTexU4_arr,
    // CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp,
    PitchedMem_Texture<uchar4,cudaFilterModePoint,cudaReadModeElementType>* timg_dmp,
    float fpPlaneDepth)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    reprojTarTexLAB_kernel<<<grid, block>>>(
        t4tex,
        timg_dmp->mem->getBuffer(), timg_dmp->mem->stride()[0],
        width, height, fpPlaneDepth);

    tTexU4 = timg_dmp->tex;

    refine_compYKNCCSimMap_kernel<<<grid, block>>>(
        rTexU4,
        tTexU4,
        osimMap_dmp.getBuffer(), osimMap_dmp.stride()[0],
        idepthMapMask_dmp.getBuffer(), idepthMapMask_dmp.stride()[0],
        width, height, wsh, gammaC, gammaP);
}

// void ps_growDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<uchar4, 2>* otimg_hmh,
void ps_growDepthMap( CudaHostMemoryHeap<uchar4, 2>* otimg_hmh,
                     CudaHostMemoryHeap<float, 2>* osim_hmh, CudaHostMemoryHeap<float, 2>* odpt_hmh,
                     CudaHostMemoryHeap<float, 2>& depthMap_hmh, cameraStruct** cams, int ncams, float* depths,
                     int ndepths, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                     bool verbose, int wsh, float gammaC, float gammaP, float simThr)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    auto rimg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    auto timg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );

    getRefTexLAB_kernel<<<grid, block>>>(
        r4tex, t4tex,
        rimg_dmp->mem->getBuffer(), rimg_dmp->mem->stride()[0],
        width, height );

    cudaTextureObject_t rTexU4 = rimg_dmp->tex;
    cudaTextureObject_t tTexU4 = rimg_dmp->tex; // set to rimg_dmp, timg_dmp still uninitialized

    /* GRIFF Note:
     * setting tTexU4 to rimg_dmp->tex seems to imply that something in
     * ps_computeSimMapForDepthMapInternal() is first done for rimg_dmp and
     * later for timg_dmp, which is undefined at this point.
     * This does not appear to happen. tTexU4 is actually set again inside
     * that function before it is needed.
     */

    clock_t tall = tic();

    CudaDeviceMemoryPitched<float, 2> finDptMap_dmp(depthMap_hmh);
    CudaDeviceMemoryPitched<float, 2> finSimMap_dmp(CudaSize<2>(width, height));

    CudaDeviceMemoryPitched<float, 2> actDptMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> actSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> lstDptMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> lstSimMap_dmp(CudaSize<2>(width, height));

    for(int d = 1; d < ndepths; d++)
    {
        float fpPlaneDepthLast = depths[d - 1];
        float fpPlaneDepthAct = depths[d];

        refine_selectPartOfDepthMapNearFPPlaneDepth_kernel<<<grid, block>>>(
            lstDptMap_dmp.getBuffer(), lstDptMap_dmp.stride()[0],
            actDptMap_dmp.getBuffer(), actDptMap_dmp.stride()[0],
            finDptMap_dmp.getBuffer(), finDptMap_dmp.stride()[0],
            width, height, fpPlaneDepthLast, fpPlaneDepthAct);
        cudaThreadSynchronize();

        ps_dilateMaskMap(lstDptMap_dmp, width, height, verbose, 1, fpPlaneDepthLast);
        ps_dilateMaskMap(actDptMap_dmp, width, height, verbose, 1, fpPlaneDepthAct);

        ps_computeSimMapForDepthMapInternal(
            t4tex,
            lstSimMap_dmp, lstDptMap_dmp, width, height, verbose, wsh, gammaC, gammaP,
            rTexU4, tTexU4,
            timg_dmp, fpPlaneDepthLast );

        ps_computeSimMapForDepthMapInternal(
            t4tex,
            actSimMap_dmp, actDptMap_dmp, width, height, verbose, wsh, gammaC, gammaP,
            rTexU4, tTexU4,
            timg_dmp, fpPlaneDepthAct );

        refine_fuseThreeDepthSimMaps_kernel<<<grid, block>>>(
            finSimMap_dmp.getBuffer(), finSimMap_dmp.stride()[0],
            finDptMap_dmp.getBuffer(), finDptMap_dmp.stride()[0],
            lstSimMap_dmp.getBuffer(), lstSimMap_dmp.stride()[0],
            lstDptMap_dmp.getBuffer(), lstDptMap_dmp.stride()[0],
            actSimMap_dmp.getBuffer(), actSimMap_dmp.stride()[0],
            actDptMap_dmp.getBuffer(), actDptMap_dmp.stride()[0],
            width, height, simThr);
    }

    /*
            //smooth depth map
            smoothDepthMap_kernel<<<grid, block>>>(
                    depthMap_dmp.getBuffer(),  depthMap_dmp.stride()[0],
                    width, height, wsh, gammaC, gammaP
            );
    */

    if(verbose)
        printf("gpu refine depth map elapsed time: %f ms \n", toc(tall));

    copy((*osim_hmh), finSimMap_dmp);
    copy((*odpt_hmh), finDptMap_dmp);

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
}

void ps_refineDepthMapReproject( CudaHostMemoryHeap<uchar4, 2>* otimg_hmh,
                                CudaHostMemoryHeap<float, 2>* osim_hmh, CudaHostMemoryHeap<float, 2>* odpt_hmh,
                                CudaHostMemoryHeap<float, 2>& depthMap_hmh, cameraStruct** cams, int ncams, int width,
                                int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                                int wsh, float gammaC, float gammaP, float simThr, int niters, bool moveByTcOrRc)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    auto rimg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    auto timg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );

    getRefTexLAB_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        rimg_dmp->mem->getBuffer(), rimg_dmp->mem->stride()[0],
        width, height );

    cudaTextureObject_t rTexU4 = rimg_dmp->tex;
    cudaTextureObject_t tTexU4 = rimg_dmp->tex;

    clock_t tall = tic();

    CudaDeviceMemoryPitched<float3, 2> dsm_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float3, 2> ssm_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bsim_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bdpt_dmp(depthMap_hmh);

    CudaDeviceMemoryPitched<float, 2> depthMap_dmp(bdpt_dmp);

    for(int iter = 1; iter <= niters; iter++)
    {
        ps_refineDepthMapInternal(
            t4tex,
            bsim_dmp, bdpt_dmp,
            // depthMap_dmp,
            bdpt_dmp, bdpt_dmp, width, height, verbose, wsh, gammaC, gammaP, simThr, dsm_dmp,
            ssm_dmp,
            rTexU4,
            tTexU4,
            timg_dmp,
            moveByTcOrRc, (float)iter);

        /*
                        //------------------------------------------------------------------------------------------------
                        //smooth depth map
                        smoothDepthMap_kernel<<<grid, block>>>(
                                bdpt_dmp.getBuffer(),  bdpt_dmp.stride()[0],
                                width, height, wsh, gammaC, gammaP
                        );
        */
    }

    if(verbose)
        printf("gpu refine depth map elapsed time: %f ms \n", toc(tall));

    copy((*osim_hmh), bsim_dmp);
    copy((*odpt_hmh), bdpt_dmp);

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
}

// void ps_computeSimMapsForNShiftsOfRcTcDepthMap(CudaArray<uchar4, 2>** ps_texs_arr,
void ps_computeSimMapsForNShiftsOfRcTcDepthMap(
                                               CudaHostMemoryHeap<float2, 2>** odepthSimMaps_hmh, int ntcsteps,
                                               CudaHostMemoryHeap<float, 2>& rcDepthMap_hmh, cameraStruct** cams,
                                               int ncams, int width, int height, int scale, int CUDAdeviceNo,
                                               int ncamsAllocated, int scales, bool verbose, int wsh, float gammaC,
                                               float gammaP, float epipShift)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    CudaDeviceMemoryPitched<float2, 2> dsm_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(rcDepthMap_hmh);

    clock_t tall = tic();

    for(int i = 0; i < ntcsteps; i++)
    {
        refine_compYKNCCDepthSimMapPatch_kernel<<<grid, block>>>(
            r4tex,
            t4tex,
            dsm_dmp.getBuffer(), dsm_dmp.stride()[0],
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.stride()[0],
            width, height,
            wsh, gammaC, gammaP, epipShift, (float)(i - ntcsteps / 2), true);
        cudaThreadSynchronize();
        copy((*odepthSimMaps_hmh[i]), dsm_dmp);
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_computeSimMapForRcTcDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* osimMap_hmh,
void ps_computeSimMapForRcTcDepthMap( CudaHostMemoryHeap<float, 2>* osimMap_hmh,
                                     CudaHostMemoryHeap<float, 2>& rcTcDepthMap_hmh, cameraStruct** cams, int ncams,
                                     int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                                     bool verbose, int wsh, float gammaC, float gammaP, float epipShift)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    CudaDeviceMemoryPitched<float, 2> osimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcTcDepthMap_dmp(rcTcDepthMap_hmh);

    clock_t tall = tic();

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        osimMap_dmp.getBuffer(), osimMap_dmp.stride()[0], rcTcDepthMap_dmp.getBuffer(), rcTcDepthMap_dmp.stride()[0], width,
        height, wsh, gammaC, gammaP, epipShift, 0.0f, false, 0, width, height);
    cudaThreadSynchronize();

    copy((*osimMap_hmh), osimMap_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_refineRcDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, float* osimMap_hmh,
void ps_refineRcDepthMap( float* osimMap_hmh,
                         float* rcDepthMap_hmh, int ntcsteps,
                         cameraStruct** cams, int ncams, int width,
                         int height, int imWidth, int imHeight, int scale, int CUDAdeviceNo, int ncamsAllocated,
                         int scales, bool verbose, int wsh, float gammaC, float gammaP, float epipShift,
                         bool moveByTcOrRc, int xFrom)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(width, height));
    copy(rcDepthMap_dmp, rcDepthMap_hmh, width, height);
    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(width, height));

    clock_t tall = tic();

#ifdef GRIFF_TEST
    refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0], // output
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0], // output
        rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        ntcsteps,
        moveByTcOrRc, xFrom, imWidth, imHeight,
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0] );

    cudaThreadSynchronize();

    refine_compYKNCCSimMapPatch_kernel_A<<<grid, block>>>(
        r4tex,
        t4tex,
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        -1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight,
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0], 0 );

    refine_compYKNCCSimMapPatch_kernel_A<<<grid, block>>>(
        r4tex,
        t4tex,
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        +1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight,
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0], 2 );

    cudaThreadSynchronize();
#else
    for(int i = 0; i < ntcsteps; i++) // Default ntcsteps = 31
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            r4tex,
            t4tex,
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0],
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.stride()[0],
            width, height, wsh, gammaC, gammaP, epipShift,
            (float)(i - (ntcsteps - 1) / 2), i, moveByTcOrRc, xFrom, imWidth, imHeight);
        cudaThreadSynchronize();
    }

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0],
        width, height, 1);
    cudaThreadSynchronize();

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        -1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight);
    cudaThreadSynchronize();
    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        width, height, 0);
    cudaThreadSynchronize();

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift, +1.0f, moveByTcOrRc, xFrom, imWidth, imHeight);
    cudaThreadSynchronize();
    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        width, height, 2);
    cudaThreadSynchronize();
#endif

    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        width, height,
        moveByTcOrRc, xFrom);

    copy(osimMap_hmh, width, height, bestSimMap_dmp);
    copy(rcDepthMap_hmh, width, height, bestDptMap_dmp);

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
                                             CudaHostMemoryHeap<float2, 2>** depthSimMaps_hmh, int ndepthSimMaps,
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
    CudaDeviceMemoryPitched<float2, 2>** depthSimMaps_dmp;

    depthSimMaps_dmp = new CudaDeviceMemoryPitched<float2, 2>*[ndepthSimMaps];
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
                gsvSampleMap_dmp.getBuffer(), gsvSampleMap_dmp.stride()[0],
                depthSimMaps_dmp[c]->getBuffer(), depthSimMaps_dmp[c]->stride()[0],
                depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->stride()[0],
                width, height, (float)s, c - 1, samplesPerPixSize, twoTimesSigmaPowerTwo);
            cudaThreadSynchronize();
        }
        fuse_updateBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
            bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.stride()[0], gsvSampleMap_dmp.getBuffer(),
            gsvSampleMap_dmp.stride()[0], width, height, (float)s, s + nSamplesHalf);
        cudaThreadSynchronize();
    }

    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
        bestDepthSimMap_dmp.getBuffer(), bestDepthSimMap_dmp.stride()[0], bestGsvSampleMap_dmp.getBuffer(),
        bestGsvSampleMap_dmp.stride()[0], depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->stride()[0], width, height,
        samplesPerPixSize);
    cudaThreadSynchronize();

    copy((*odepthSimMap_hmh), bestDepthSimMap_dmp);

    for(int i = 0; i < ndepthSimMaps; i++)
    {
        delete depthSimMaps_dmp[i];
    }
    delete[] depthSimMaps_dmp;

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}


// void ps_optimizeDepthSimMapGradientDescent(CudaArray<uchar4, 2>** ps_texs_arr,
void ps_optimizeDepthSimMapGradientDescent(
                                           CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                                           CudaHostMemoryHeap<float2, 2>** dataMaps_hmh, int ndataMaps,
                                           int nSamplesHalf, int nDepthsToRefine, int nIters, float sigma,
                                           cameraStruct** cams, int ncams, int width, int height, int scale,
                                           int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, int yFrom)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    float samplesPerPixSize = (float)(nSamplesHalf / ((nDepthsToRefine - 1) / 2));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    CudaDeviceMemoryPitched<float2, 2>** dataMaps_dmp;
    dataMaps_dmp = new CudaDeviceMemoryPitched<float2, 2>*[ndataMaps];
    for(int i = 0; i < ndataMaps; i++)
    {
        dataMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*dataMaps_dmp[i]), (*dataMaps_hmh[i]));
    }

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(CudaSize<2>(width, height));
    copy(optDepthSimMap_dmp, (*dataMaps_dmp[0]));

    auto optDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    for(int iter = 0; iter < nIters; iter++) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOPtDepthSimMap_kernel<<<grid, block>>>(
            optDepthMap_dmp.getBuffer(), optDepthMap_dmp.stride()[0], optDepthSimMap_dmp.getBuffer(),
            optDepthSimMap_dmp.stride()[0], width, height);

        copy( *optDepthMap_arr->mem, optDepthMap_dmp );

        // Adjust depth/sim by using previously computed depths (depthTex is accessed inside this kernel)
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(
            r4tex,
            optDepthMap_arr->tex,
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.stride()[0],
            dataMaps_dmp[0]->getBuffer(), dataMaps_dmp[0]->stride()[0],
            dataMaps_dmp[1]->getBuffer(), dataMaps_dmp[1]->stride()[0],
            width, height, iter, samplesPerPixSize, yFrom);
        cudaThreadSynchronize();
    }

    copy((*odepthSimMap_hmh), optDepthSimMap_dmp);

    for(int i = 0; i < ndataMaps; i++)
    {
        delete dataMaps_dmp[i];
    }
    delete[] dataMaps_dmp;

    global_data.pitched_mem_float_point_tex_cache.put( optDepthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_GC_aggregatePathVolume(CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
                               CudaHostMemoryHeap<unsigned int, 3>& ivol_hmh, int volDimX, int volDimY, int volDimZ)
{
    CudaDeviceMemoryPitched<unsigned int, 3> vol_dmp(ivol_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<int4, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<unsigned int, 2> oxyslice_dmp(CudaSize<2>(volDimX, volDimY));

    volume_GC_K_initXYSliceInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0],
                                                              vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
                                                              volDimX, volDimY, 0);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    for(int z = 1; z < volDimZ; z++)
    {
        update_GC_volumeXYSliceAtZInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0],
                                                                     vol_dmp.getBuffer(), vol_dmp.stride()[1],
                                                                     vol_dmp.stride()[0], volDimX, volDimY, volDimZ, z);
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();
    }

    volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel<<<gridvol, blockvol>>>(
        oxyslice_dmp.getBuffer(), oxyslice_dmp.stride()[0], xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0], volDimX,
        volDimY);

    copy((*ftid_hmh), oxyslice_dmp);
    // pr_printfDeviceMemoryInfo();
}

void ps_GC_K_aggregatePathVolume(CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
                                 CudaHostMemoryHeap<unsigned int, 3>& ivol_hmh, int volDimX, int volDimY, int volDimZ,
                                 int K)
{
    CudaDeviceMemoryPitched<unsigned int, 3> vol_dmp(ivol_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 8;
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    CudaDeviceMemoryPitched<int4, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<unsigned int, 2> oxyslice_dmp(CudaSize<2>(volDimX, volDimY));

    for(int z = 0; z <= std::min(K - 1, volDimZ - 1); z++)
    {
        volume_GC_K_initXYSliceInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0],
                                                                  vol_dmp.getBuffer(), vol_dmp.stride()[1],
                                                                  vol_dmp.stride()[0], volDimX, volDimY, z);
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();
    }

    for(int z = 0; z <= volDimZ - K - 1; z++)
    {
        update_GC_K_volumeXYSliceAtZInt4_kernel<<<gridvol, blockvol>>>(
            xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0], vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, z, K);
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();
    }

    volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel<<<gridvol, blockvol>>>(
        oxyslice_dmp.getBuffer(), oxyslice_dmp.stride()[0], xyslice_dmp.getBuffer(), xyslice_dmp.stride()[0], volDimX,
        volDimY);

    copy((*ftid_hmh), oxyslice_dmp);
    // pr_printfDeviceMemoryInfo();
}

// void ps_ptsStatForRcDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* depthMap_hmh,
void ps_ptsStatForRcDepthMap( CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                             cameraStruct** cams, CudaHostMemoryHeap<float3, 2>& pts_hmh,
                             CudaHostMemoryHeap<float2, 2>& out_hmh, int npts, int width, int height, int scale,
                             int CUDAdeviceNo, int ncamsAllocated, int scales, int maxNPixSize, int wsh, float gammaC,
                             float gammaP, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    copy( *depthMap_arr->mem, *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    CudaDeviceMemoryPitched<float3, 2> pts_dmp(pts_hmh);
    CudaDeviceMemoryPitched<float2, 2> out_dmp(CudaSize<2>(npts, 1));

    /*
    {
            int block_size = 8;
            dim3 block(block_size,block_size,1);
            dim3 grid(divUp(width, block_size),divUp(height,block_size),1);
            smoothDepthMap_kernel<<<grid, block>>>(
                    depthMap_dmp.getBuffer(),  depthMap_dmp.stride()[0],
                    width, height, wsh, gammaC, gammaP
            );
            cudaThreadSynchronize();
            CHECK_CUDA_ERROR();
    }
    */

    {
        int block_size = 64;
        dim3 block(block_size, 1, 1);
        dim3 grid(divUp(npts, block_size), 1, 1);
        ptsStatForRcDepthMap_kernel<<<grid, block>>>(
            r4tex,
            depthsTex,
            out_dmp.getBuffer(), out_dmp.stride()[0], pts_dmp.getBuffer(),
            pts_dmp.stride()[0], npts, width, height, maxNPixSize, wsh, gammaC,
            gammaP);
        cudaThreadSynchronize();
        CHECK_CUDA_ERROR();
    }

    copy(out_hmh, out_dmp);

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_computeSimMapReprojectByDepthMapMovedByStep(CudaArray<uchar4, 2>** ps_texs_arr,
void ps_computeSimMapReprojectByDepthMapMovedByStep(
    CudaHostMemoryHeap<float, 2>* osimMap_hmh,
    CudaHostMemoryHeap<float, 2>* iodepthMap_hmh,
    cameraStruct** cams,
    int ncams, int width, int height, int scale, int CUDAdeviceNo,
    int ncamsAllocated, int scales, bool verbose, int wsh, float gammaC,
    float gammaP, bool moveByTcOrRc, float step )
{
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto rimg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    getRefTexLAB_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        rimg_dmp->mem->getBuffer(), rimg_dmp->mem->stride()[0],
        width, height );

    cudaTextureObject_t rTexU4 = rimg_dmp->tex;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto timg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    CudaDeviceMemoryPitched<float, 2> iodepthMap_dmp(*iodepthMap_hmh);
    refine_reprojTarTexLABByDepthMapMovedByStep_kernel<<<grid, block>>>(
        t4tex,
        iodepthMap_dmp.getBuffer(), iodepthMap_dmp.stride()[0],
        timg_dmp->mem->getBuffer(), timg_dmp->mem->stride()[0],
        width, height, moveByTcOrRc, step);

    cudaTextureObject_t tTexU4 = timg_dmp->tex;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    CudaDeviceMemoryPitched<float, 2> osimMap_dmp(CudaSize<2>(width, height));

    refine_compYKNCCSimMap_kernel<<<grid, block>>>(
        rTexU4,
        tTexU4,
        osimMap_dmp.getBuffer(), osimMap_dmp.stride()[0],
        iodepthMap_dmp.getBuffer(), iodepthMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP);

    copy((*iodepthMap_hmh), iodepthMap_dmp);
    copy((*osimMap_hmh), osimMap_dmp);

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
}

void ps_reprojectRGBTcImageByDepthMap(CudaHostMemoryHeap<uchar4, 2>* iTcoRcRgbImage_hmh,
                                      CudaHostMemoryHeap<float, 2>* rcDepthMap_hmh, cameraStruct** cams, int ncams,
                                      int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                                      int scales, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // CudaDeviceMemoryPitched<uchar4, 2> iTcoRcimg_dmp(*iTcoRcRgbImage_hmh);
    auto iTcoRcimg_dmp =
        global_data.pitched_mem_uchar4_linear_tex_cache.get(
            iTcoRcRgbImage_hmh->getSize()[0],
            iTcoRcRgbImage_hmh->getSize()[1] );
    copy( *iTcoRcRgbImage_hmh, *iTcoRcimg_dmp->mem );
    // CudaArray<uchar4, 2> tTexU4_arr(iTcoRcimg_dmp);
    // cudaBindTextureToArray(t4tex, tTexU4_arr.getArray(), cudaCreateChannelDesc<uchar4>());
    cudaTextureObject_t t4tex = iTcoRcimg_dmp->tex;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(*rcDepthMap_hmh);
    refine_reprojTarTexLABByDepthMapMovedByStep_kernel<<<grid, block>>>(
        t4tex,
        rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.stride()[0],
        iTcoRcimg_dmp->mem->getBuffer(), iTcoRcimg_dmp->mem->stride()[0],
        width, height, false, 0.0f);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    copy((*iTcoRcRgbImage_hmh), *iTcoRcimg_dmp->mem );

    // cudaUnbindTexture(t4tex);
    global_data.pitched_mem_uchar4_linear_tex_cache.put( iTcoRcimg_dmp );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_computeRcTcDepthMap(CudaHostMemoryHeap<float, 2>& iRcDepthMap_oRcTcDepthMap_hmh,
                            CudaHostMemoryHeap<float, 2>& tcDepthMap_hmh, float pixSizeRatioThr, cameraStruct** cams,
                            int ncams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                            int scales, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);

    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(iRcDepthMap_oRcTcDepthMap_hmh);
    // CudaArray<float, 2> depthMap_arr(tcDepthMap_hmh);
    // cudaBindTextureToArray(depthsTex, depthMap_arr.getArray(), cudaCreateChannelDesc<float>());
    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        tcDepthMap_hmh.getSize()[0],
        tcDepthMap_hmh.getSize()[1] );
    copy( *depthMap_arr->mem, tcDepthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    refine_computeRcTcDepthMap_kernel<<<grid, block>>>(
        depthsTex,
        rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.stride()[0],
        width, height, pixSizeRatioThr);

    copy(iRcDepthMap_oRcTcDepthMap_hmh, rcDepthMap_dmp);

    // cudaUnbindTexture(depthsTex);
    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_getSilhoueteMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
void ps_getSilhoueteMap( CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
                        int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int step, int camId,
                        uchar4 maskColorRgb, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    uchar4 maskColorLab;
    float3 flab = xyz2lab(rgb2xyz(uchar4_to_float3(maskColorRgb)));
    maskColorLab.x = (unsigned char)(flab.x);
    maskColorLab.y = (unsigned char)(flab.y);
    maskColorLab.z = (unsigned char)(flab.z);
    maskColorLab.w = 0;

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width / step, block_size), divUp(height / step, block_size), 1);

    // cudaBindTextureToArray(rTexU4, ps_texs_arr[camId * scales + scale]->getArray(), cudaCreateChannelDesc<uchar4>());
    cudaTextureObject_t rTexU4 = global_data.getScaledPictureTexPoint( scale, camId );

    CudaDeviceMemoryPitched<bool, 2> map_dmp(CudaSize<2>(width / step, height / step));
    getSilhoueteMap_kernel<<<grid, block>>>(
        rTexU4,
        map_dmp.getBuffer(), map_dmp.stride()[0],
        step, width, height, maskColorLab );
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    // cudaUnbindTexture(rTexU4);

    copy((*omap_hmh), map_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_retexture(CudaHostMemoryHeap<uchar4, 2>* bmpOrig_hmh, CudaHostMemoryHeap<uchar4, 2>* bmpObj_hmh,
                  CudaHostMemoryHeap<float4, 2>* retexturePixs_hmh, int wObj, int hObj, int wOrig, int hOrig,
                  int slicesAtTime, int ntimes, int npixs, int CUDAdeviceNo, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    // CudaArray<uchar4, 2> bmpOrig_arr(*bmpOrig_hmh);
    // cudaBindTextureToArray(r4tex, bmpOrig_arr.getArray(), cudaCreateChannelDesc<uchar4>());
    auto bmpOrig_arr =
        global_data.pitched_mem_uchar4_linear_tex_cache.get(
            bmpOrig_hmh->getSize()[0],
            bmpOrig_hmh->getSize()[1] );
    copy( *bmpOrig_hmh, *bmpOrig_arr->mem );
    cudaTextureObject_t r4tex = bmpOrig_arr->tex;

    CudaDeviceMemoryPitched<uchar4, 2> bmpObj_dmp(*bmpObj_hmh);
    CudaDeviceMemoryPitched<float4, 2> retexturePixs_dmp(*retexturePixs_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(slicesAtTime, block_size), divUp(ntimes, block_size), 1);

    retexture_kernel<<<grid, block>>>(
        r4tex,
        bmpObj_dmp.getBuffer(), bmpObj_dmp.stride()[0],
        retexturePixs_dmp.getBuffer(), retexturePixs_dmp.stride()[0],
        slicesAtTime, ntimes, npixs);
    CHECK_CUDA_ERROR();

    // cudaUnbindTexture(r4tex);

    copy( *bmpObj_hmh, bmpObj_dmp);
    global_data.pitched_mem_uchar4_linear_tex_cache.put( bmpOrig_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_retextureComputeNormalMap(CudaHostMemoryHeap<uchar4, 2>* bmpObj_hmh,
                                  CudaHostMemoryHeap<float2, 2>* retexturePixs_hmh,
                                  CudaHostMemoryHeap<float3, 2>* retextureNorms_hmh, int wObj, int hObj,
                                  int slicesAtTime, int ntimes, int npixs, int CUDAdeviceNo, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    CudaDeviceMemoryPitched<uchar4, 2> bmpObj_dmp(*bmpObj_hmh);
    CudaDeviceMemoryPitched<float2, 2> retexturePixs_dmp(*retexturePixs_hmh);
    CudaDeviceMemoryPitched<float3, 2> retexturePixsNorms_dmp(*retextureNorms_hmh);

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(slicesAtTime, block_size), divUp(ntimes, block_size), 1);

    retextureComputeNormalMap_kernel<<<grid, block>>>(
        bmpObj_dmp.getBuffer(), bmpObj_dmp.stride()[0], retexturePixs_dmp.getBuffer(), retexturePixs_dmp.stride()[0],
        retexturePixsNorms_dmp.getBuffer(), retexturePixsNorms_dmp.stride()[0], slicesAtTime, ntimes, npixs);
    cudaThreadSynchronize();
    CHECK_CUDA_ERROR();

    copy((*bmpObj_hmh), bmpObj_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_colorExtractionPushPull(CudaHostMemoryHeap<uchar4, 2>* bmp_hmh, int w, int h, int CUDAdeviceNo, bool verbose)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    int block_size = 16;
    int npyramidLevels = 10;

    global_data.allocPyramidArrays( npyramidLevels, w, h );

    // pyramid arr
    // CudaArray<uchar4, 2>** pyramid_arr = new CudaArray<uchar4, 2>*[npyramidLevels];
    int wact = w;
    int hact = h;
    int wLevels[npyramidLevels];
    int hLevels[npyramidLevels];

    // push
    for(int i = 0; i < npyramidLevels; i++)
    {
        // pyramid_arr[i] = new CudaArray<uchar4, 2>(CudaSize<2>(wact, hact));
        wLevels[i] = wact;
        hLevels[i] = hact;

        if(i == 0)
        {
            // copy((*pyramid_arr[i]), (*bmp_hmh));
            copy( global_data.getPyramidArray(i), (*bmp_hmh));
        }
        else
        {
            // cudaBindTextureToArray(r4tex, pyramid_arr[i - 1]->getArray(), cudaCreateChannelDesc<uchar4>());
            cudaTextureObject_t r4tex = global_data.getPyramidTex( i-1 );

            // CudaDeviceMemoryPitched<uchar4, 2> bmpNextLevel_dmp(CudaSize<2>(wact, hact));
            CudaDeviceMemoryPitched<uchar4, 2>& bmpNextLevel_dmp = global_data.getPyramidArray( i );
            dim3 block(block_size, block_size, 1);
            dim3 grid(divUp(wact, block_size), divUp(hact, block_size), 1);
            pushPull_Push_kernel<<<grid, block>>>( r4tex, bmpNextLevel_dmp.getBuffer(), bmpNextLevel_dmp.stride()[0], wact, hact);
            // cudaThreadSynchronize();
            CHECK_CUDA_ERROR();
            // cudaUnbindTexture(r4tex);

            // copy((*pyramid_arr[i]), bmpNextLevel_dmp);
            // copy( global_data.getPyramidArray(i), bmpNextLevel_dmp);
        }
        wact = wact / 2;
        hact = hact / 2;
        printf("push level %i\n", i);
    }

    // pull
    for(int i = npyramidLevels - 1; i >= 1; i--)
    {
        // cudaBindTextureToArray(r4tex, pyramid_arr[i]->getArray(), cudaCreateChannelDesc<uchar4>());
        cudaTextureObject_t r4tex = global_data.getPyramidTex( i );

        // CudaDeviceMemoryPitched<uchar4, 2> bmpNextLevel_dmp(CudaSize<2>(wLevels[i - 1], hLevels[i - 1]));
        // copy(bmpNextLevel_dmp, (*pyramid_arr[i - 1]));
        CudaDeviceMemoryPitched<uchar4, 2>& bmpNextLevel_dmp = global_data.getPyramidArray( i-1 );

        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(wLevels[i - 1], block_size), divUp(hLevels[i - 1], block_size), 1);
        pushPull_Pull_kernel<<<grid, block>>>( r4tex, bmpNextLevel_dmp.getBuffer(), bmpNextLevel_dmp.stride()[0], wLevels[i - 1],
                                              hLevels[i - 1]);
        CHECK_CUDA_ERROR();
        // cudaUnbindTexture(r4tex);

        // copy((*pyramid_arr[i - 1]), bmpNextLevel_dmp);
        printf("pull level %i\n", i);
    }

    // copy((*bmp_hmh), (*pyramid_arr[0]));
    CudaDeviceMemoryPitched<uchar4, 2>& bmp = global_data.getPyramidArray( 0 );
    copy((*bmp_hmh), bmp );

    global_data.freePyramidArrays();
}

} // namespace depthMap
} // namespace aliceVision
