// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/depthMap/cuda/planeSweeping/onoff.cuh"
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

__host__ std::shared_ptr<CameraBaseStruct> ps_init_camera_vectors( const Point3d&    CA,
                                                                   const Matrix3x4&  P,
                                                                   const Matrix3x3&  iP,
                                                                   const Matrix3x3&  RA,
                                                                   const Matrix3x3&  iRA,
                                                                   const Matrix3x3&  K,
                                                                   const Matrix3x3&  iK )
{
    std::shared_ptr<CameraBaseStruct> cam( new CameraBaseStruct );

    cam->C.x = CA.x;
    cam->C.y = CA.y;
    cam->C.z = CA.z;

    cam->P[0] = P.m11;
    cam->P[1] = P.m21;
    cam->P[2] = P.m31;
    cam->P[3] = P.m12;
    cam->P[4] = P.m22;
    cam->P[5] = P.m32;
    cam->P[6] = P.m13;
    cam->P[7] = P.m23;
    cam->P[8] = P.m33;
    cam->P[9] = P.m14;
    cam->P[10] = P.m24;
    cam->P[11] = P.m34;

    cam->iP[0] = iP.m11;
    cam->iP[1] = iP.m21;
    cam->iP[2] = iP.m31;
    cam->iP[3] = iP.m12;
    cam->iP[4] = iP.m22;
    cam->iP[5] = iP.m32;
    cam->iP[6] = iP.m13;
    cam->iP[7] = iP.m23;
    cam->iP[8] = iP.m33;

    cam->R[0] = RA.m11;
    cam->R[1] = RA.m21;
    cam->R[2] = RA.m31;
    cam->R[3] = RA.m12;
    cam->R[4] = RA.m22;
    cam->R[5] = RA.m32;
    cam->R[6] = RA.m13;
    cam->R[7] = RA.m23;
    cam->R[8] = RA.m33;

    cam->iR[0] = iRA.m11;
    cam->iR[1] = iRA.m21;
    cam->iR[2] = iRA.m31;
    cam->iR[3] = iRA.m12;
    cam->iR[4] = iRA.m22;
    cam->iR[5] = iRA.m32;
    cam->iR[6] = iRA.m13;
    cam->iR[7] = iRA.m23;
    cam->iR[8] = iRA.m33;

    cam->K[0] = K.m11;
    cam->K[1] = K.m21;
    cam->K[2] = K.m31;
    cam->K[3] = K.m12;
    cam->K[4] = K.m22;
    cam->K[5] = K.m32;
    cam->K[6] = K.m13;
    cam->K[7] = K.m23;
    cam->K[8] = K.m33;

    cam->iK[0] = iK.m11;
    cam->iK[1] = iK.m21;
    cam->iK[2] = iK.m31;
    cam->iK[3] = iK.m12;
    cam->iK[4] = iK.m22;
    cam->iK[5] = iK.m32;
    cam->iK[6] = iK.m13;
    cam->iK[7] = iK.m23;
    cam->iK[8] = iK.m33;


    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    cam->ZVect = ps_M3x3mulV3(cam->iR, z);
    ps_normalize(cam->ZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    cam->YVect = ps_M3x3mulV3(cam->iR, y);
    ps_normalize(cam->YVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    cam->XVect = ps_M3x3mulV3(cam->iR, x);
    ps_normalize(cam->XVect);

    return cam;
}

void pr_printfDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;
    cudaError_t err = cudaMemGetInfo(&iavail, &itotal);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get CUDA memory info" );
    size_t iused = itotal - iavail;

    float avail = (float)iavail / (1024.0f * 1024.0f);
    float total = (float)itotal / (1024.0f * 1024.0f);
    float used = (float)iused / (1024.0f * 1024.0f);

    int CUDAdeviceNo;
    err = cudaGetDevice(&CUDAdeviceNo);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get current CUDA device" );

    printf("Device %i memory - used: %f, free: %f, total: %f\n", CUDAdeviceNo, used, avail, total);
}

float3 ps_getDeviceMemoryInfo()
{
    size_t iavail;
    size_t itotal;
    cudaError_t err = cudaMemGetInfo(&iavail, &itotal);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get CUDA memory info" );
    size_t iused = itotal - iavail;

    float avail = (float)iavail / (1024.0f * 1024.0f);
    float total = (float)itotal / (1024.0f * 1024.0f);
    float used = (float)iused / (1024.0f * 1024.0f);

    return make_float3(avail, total, used);
}

static __host__ void ps_init_reference_camera_matrices( const CameraBaseStruct& cam )
{
    cudaError_t err;

    err = cudaMemcpyToSymbol(sg_s_rP, cam.P, sizeof(float) * 3 * 4);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_riP, cam.iP, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_rR, cam.R, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_riR, cam.iR, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_rK, cam.K, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_riK, cam.iK, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_rC, &cam.C, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );

    err = cudaMemcpyToSymbol(sg_s_rXVect, &cam.XVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_rYVect, &cam.YVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_rZVect, &cam.ZVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
}

static __host__ void ps_init_target_camera_matrices( const CameraBaseStruct& cam )
{
    cudaError_t err;

    err = cudaMemcpyToSymbol(sg_s_tP, cam.P, sizeof(float) * 3 * 4);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tiP, cam.iP, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tR, cam.R, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tiR, cam.iR, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tK, cam.K, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tiK, cam.iK, sizeof(float) * 3 * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tC, &cam.C, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );

    err = cudaMemcpyToSymbol(sg_s_tXVect, &cam.XVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tYVect, &cam.YVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
    err = cudaMemcpyToSymbol(sg_s_tZVect, &cam.ZVect, sizeof(float) * 3);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed copying to symbol" );
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
            err = cudaGetDeviceProperties(&dprop, i);
            memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get current CUDA device properties" );
            printf("   %d: %s\n", i, dprop.name);
            printf("       max 1D texture size: %d bytes\n", dprop.maxTexture1D );
            printf("       max 1D surface size: %d bytes\n", dprop.maxSurface1D );
            printf("       max 2D texture size: (%d,%d) bytes\n", dprop.maxTexture2D[0], dprop.maxTexture2D[1] );
            printf("       max 2D surface size: (%d,%d) bytes\n", dprop.maxSurface2D[0], dprop.maxSurface2D[1] );
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
    cudaError_t err = cudaGetDeviceCount(&num_gpus);
    memOpErrorCheck( err, __FILE__, __LINE__, "Failed to get current CUDA device count" );

    if( deviceId >= num_gpus )
    {
        printf("requested CUDA device no %d does not exist, only %d devices\n", deviceId, num_gpus );
    }
    else
    {
        cudaError_t outval = cudaSetDevice(deviceId);
        if( outval == cudaSuccess )
        {
            printf("CUDA device no %i for %i\n", outval, deviceId);
        }
        else
        {
            printf("Could not select CUDA device no %d\n", deviceId);
        }

        outval = cudaGetDeviceProperties( &global_data.dev_properties, deviceId );
    }

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

    // printf("total size of preallocated images in GPU memory: %f\n",(float)allBytes/(1024.0f*1024.0f));

    pr_printfDeviceMemoryInfo();
    // printf("ps_deviceAllocate - done\n");
}

int ps_getTexture2DLinear()
{
    return global_data.dev_properties.maxTexture2DLinear[0];
}

void testCUDAdeviceNo(int CUDAdeviceNo)
{
    cudaError_t err;
    int num_gpus;

    err = cudaGetDeviceCount(&num_gpus);
    if( err != cudaSuccess )
    {
        printf("Cannot enumerate GPUs\n");
    }
    else
    {
    	printf("Number of GPUs in the system: %d\n", num_gpus );

    	int myCUDAdeviceNo;
    	err = cudaGetDevice(&myCUDAdeviceNo);
    	if( err != cudaSuccess )
    	{
	        printf( "Could not retrieve own device number\n" );
    	}

    	if(myCUDAdeviceNo != CUDAdeviceNo)
    	{
            printf("WARNING different device %i %i\n", myCUDAdeviceNo, CUDAdeviceNo);
    	}
    }
}

void ps_deviceUpdateCam( const cameraStruct* const cam, int camId, int CUDAdeviceNo,
                         int ncamsAllocated, int scales, int w, int h, int varianceWsh)
{
    std::cerr << "    INFO " << __FUNCTION__ << " Calling with scales=" << scales << std::endl;
    testCUDAdeviceNo(CUDAdeviceNo);

    CudaArray<uchar4,2>& array0 = global_data.getScaledPictureArray( 0, camId );
    cudaTextureObject_t  r4tex  = global_data.getScaledPictureTex  ( 0, camId );

    // compute gradient
    {
        CudaDeviceMemoryPitched<uchar4, 2> tex_lab_dmp(CudaSize<2>(w, h));
        std::cerr << "    INFO " << __FUNCTION__ << " Allocating pitch memory with " << w << "X" << h << " entries" << std::endl;
        // tex_lab_dmp.copyFrom( *cam->tex_rgba_hmh ); // copy host to device
        tex_lab_dmp.copyFrom( cam->getRefTexRGBA() ); // copy host to device

        int block_size = 16;
        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(w, block_size), divUp(h, block_size), 1);
        rgb2lab_kernel<<<grid, block>>>(tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0], w, h);
        cudaDeviceSynchronize();
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        array0.copyFrom( tex_lab_dmp );

        if(varianceWsh > 0)
        {
            compute_varLofLABtoW_kernel<<<grid, block>>>(
                    r4tex,
                    tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0], w, h,
                    varianceWsh);
            memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

            array0.copyFrom( tex_lab_dmp );
        }
    }

    // for each scale
    for(int scale = 1; scale < scales; scale++)
    {
        int radius = scale + 1;
        GaussianArray* gaussian_arr = ps_create_gaussian_arr(1.0f, radius);

        int block_size = 16;
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

        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        CudaArray<uchar4,2>& array = global_data.getScaledPictureArray( scale, camId );

        array.copyFrom( tex_lab_dmp );

        if(varianceWsh > 0)
        {
            r4tex = global_data.getScaledPictureTex( scale, camId );
            compute_varLofLABtoW_kernel<<<grid, block>>>(
                r4tex,
                tex_lab_dmp.getBuffer(), tex_lab_dmp.stride()[0],
                w / (scale + 1), h / (scale + 1), varianceWsh);
            memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

            array.copyFrom( tex_lab_dmp );
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

/**
 * @param[inout] d_volSimT similarity volume with some transposition applied
 */
static void ps_aggregatePathVolume(
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
    int block_size = 16;
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    // Set the first Z plane from 'd_volSimT' to 255
    volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
        d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], volDimX, volDimY, volDimZ, 0, 255);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    for(int z = 1; z < volDimZ; z++)
    {
        d_xySliceForZM1.copyFrom( d_xySliceForZ );
        xySliceForZM1_arr->mem->copyFrom( d_xySliceForZM1 );

        // For each column: compute the best score
        // Foreach x:
        //   d_xSliceBestInColSimForZM1[x] = min(d_xySliceForZ[1:height])
        volume_computeBestXSliceUInt_kernel<<<gridvolrow, blockvolrow>>>(
            sliceTexUInt,
            d_xSliceBestInColSimForZM1.getBuffer(),
            volDimX, volDimY);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        // Copy the 'z' plane from 'd_volSimT' into 'd_xySliceForZ'
        volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned char><<<gridvol, blockvol>>>(
            d_xySliceForZ.getBuffer(), d_xySliceForZ.stride()[0],
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0],
            volDimX, volDimY, volDimZ, z);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        volume_agregateCostVolumeAtZinSlices_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            r4tex,
            d_xySliceForZ.getBuffer(), d_xySliceForZ.stride()[0],              // inout: xySliceForZ
            d_xySliceForZM1.getBuffer(), d_xySliceForZM1.stride()[0],          // in:    xySliceForZM1
            d_xSliceBestInColSimForZM1.getBuffer(),                          // in:    xSliceBestInColSimForZM1
            d_volSimT.getBuffer(), d_volSimT.stride()[1], d_volSimT.stride()[0], // out:   volSimT
            volDimX, volDimY, volDimZ,
            z, P1, P2, transfer, volLUX,
            volLUY, dimTrnX, doInvZ);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }

    global_data.pitched_mem_uint_point_tex_cache.put( xySliceForZM1_arr );

    if(verbose)
        printf("ps_aggregatePathVolume done\n");
}

/**
 * @param[out] volAgr_dmp output volume where we will aggregate the best XXX
 * @param[in] d_volSim input similarity volume
 */
static void ps_updateAggrVolume(
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
    int block_size = 16;
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
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }
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
            memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
        }
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
            memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
        }
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
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }
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

    ps_init_reference_camera_matrices( *rccam->cam );

    // bind 'r4tex' from the image in Lab colorspace at the scale used
    CudaArray<uchar4,2>& array = global_data.getScaledPictureArray( scale, rccam->camId );
    cudaTextureObject_t  r4tex = global_data.getScaledPictureTex( scale, rccam->camId );

    CudaDeviceMemoryPitched<unsigned char, 3> volSim_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));
    copy(volSim_dmp, iovol_hmh, volDimX, volDimY, volDimZ);

    clock_t tall = tic();

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
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

void ps_computeSimilarityVolume(
    CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp, cameraStruct** cams, int ncams,
    int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ, int volLUX,
    int volLUY, int volLUZ,
    CudaHostMemoryHeap<int, 2>& volPixs_hmh_x,
    CudaHostMemoryHeap<int, 2>& volPixs_hmh_y,
    CudaHostMemoryHeap<int, 2>& volPixs_hmh_z,
    CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
    int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
    int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, bool doUsePixelsDepths,
    int nbest, bool useTcOrRcPixSize, float gammaC, float gammaP, bool subPixel,
    float epipShift )
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);
    CHECK_CUDA_ERROR();

    // if(verbose)
    printf("nDepths %i, nDepthsToSearch %i \n", nDepths, nDepthsToSearch);
    printf("volPixs_hmh_x.getBytes(): %li\n", long(volPixs_hmh_x.getBytes()));
    printf("volPixs_hmh_x.getSize().dim(): %li\n", long(volPixs_hmh_x.getSize().dim()));
    printf("volPixs_hmh_x.getSize(): (%li, %li)\n", long(volPixs_hmh_x.getSize()[0]), long(volPixs_hmh_x.getSize()[1]));
    printf("nDepths %i, nDepthsToSearch %i \n", nDepths, nDepthsToSearch);

    auto volPixs_arr_x = global_data.pitched_mem_int_point_tex_cache.get(
        volPixs_hmh_x.getSize()[0],
        volPixs_hmh_x.getSize()[1] );
    volPixs_arr_x->mem->copyFrom( volPixs_hmh_x );

    auto volPixs_arr_y = global_data.pitched_mem_int_point_tex_cache.get(
        volPixs_hmh_y.getSize()[0],
        volPixs_hmh_y.getSize()[1] );
    volPixs_arr_y->mem->copyFrom( volPixs_hmh_y );

    auto volPixs_arr_z = global_data.pitched_mem_int_point_tex_cache.get(
        volPixs_hmh_z.getSize()[0],
        volPixs_hmh_z.getSize()[1] );
    volPixs_arr_z->mem->copyFrom( volPixs_hmh_z );

    auto depths_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depths_hmh.getSize()[0],
        depths_hmh.getSize()[1] );
    depths_arr->mem->copyFrom( depths_hmh );

    dim3 block(32, 16, 1);
    dim3 grid(divUp(nDepthsToSearch, block.x), divUp(slicesAtTime, block.y), 1);
    dim3 blockvol(block.x, block.y, 1);
    dim3 gridvol(divUp(volDimX, block.x), divUp(volDimY, block.y), 1);

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices( *cams[0]->cam );
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices( *cams[c]->cam );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );
    CHECK_CUDA_ERROR();

    //--------------------------------------------------------------------------------------------------
    // init similarity volume
    {
        dim3 blockvol3d(8, 8, 8);
        dim3 gridvol3d( divUp(volDimX, blockvol3d.x),
                        divUp(volDimY, blockvol3d.y),
                        divUp(volDimZ, blockvol3d.z) );
        volume_initFullVolume_kernel<unsigned char><<<gridvol3d, blockvol3d >>>(
            vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            volDimX, volDimY, volDimZ, 255 );
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
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
            volPixs_arr_x->tex,
            volPixs_arr_y->tex,
            volPixs_arr_z->tex,
            slice_dmp.getBuffer(), slice_dmp.stride()[0],
            nDepthsToSearch, nDepths,
            slicesAtTime, width, height, wsh, t, npixs, gammaC, gammaP, epipShift);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        volume_saveSliceToVolume_kernel<<<grid, block>>>(
            volPixs_arr_x->tex,
            volPixs_arr_y->tex,
            volPixs_arr_z->tex,
            vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
            slice_dmp.getBuffer(), slice_dmp.stride()[0],
            nDepthsToSearch, nDepths,
            slicesAtTime, width, height, t, npixs, volStepXY,
            volDimX, volDimY, volDimZ, volLUX, volLUY, volLUZ);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }
    CHECK_CUDA_ERROR();

    global_data.pitched_mem_float_point_tex_cache.put( depths_arr );
    global_data.pitched_mem_int_point_tex_cache .put( volPixs_arr_x );
    global_data.pitched_mem_int_point_tex_cache .put( volPixs_arr_y );
    global_data.pitched_mem_int_point_tex_cache .put( volPixs_arr_z );

    if(verbose)
        printf("ps_computeSimilarityVolume elapsed time: %f ms \n", toc(tall));
}

float ps_planeSweepingGPUPixelsVolume(
                                      unsigned char* ovol_hmh, cameraStruct** cams, int ncams,
                                      int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ,
                                      int volLUX, int volLUY, int volLUZ,
				      CudaHostMemoryHeap<int, 2>& volPixs_hmh_x,
				      CudaHostMemoryHeap<int, 2>& volPixs_hmh_y,
				      CudaHostMemoryHeap<int, 2>& volPixs_hmh_z,
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
        volDimZ, volLUX, volLUY, volLUZ,
        volPixs_hmh_x,
        volPixs_hmh_y,
        volPixs_hmh_z,
        depths_hmh, nDepthsToSearch, slicesAtTime,
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

// void ps_smoothDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* depthMap_hmh,
void ps_smoothDepthMap( CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                        const cameraStruct& cams, int width, int height, int scale, int CUDAdeviceNo,
                        // int ncamsAllocated,
                        int scales, int wsh, bool verbose, float gammaC, float gammaP)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    depthMap_arr->mem->copyFrom( *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;


    ps_init_reference_camera_matrices(*cams.cam);
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams.camId );

    auto depthMap_dmp = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    int block_size = 16;
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    if(verbose)
        printf("copy depth map to host\n");

    depthMap_hmh->copyFrom( *depthMap_dmp->mem );

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_dmp );
    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_filterDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float, 2>* depthMap_hmh,
void ps_filterDepthMap( CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                       const cameraStruct& cams,
                       int width, int height, int scale, int CUDAdeviceNo,
                       // int ncamsAllocated,
                       int scales, int wsh, bool verbose, float gammaC, float minCostThr)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    depthMap_arr->mem->copyFrom( *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    ps_init_reference_camera_matrices( *cams.cam );

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams.camId );

    // CudaDeviceMemoryPitched<float, 2> depthMap_dmp(CudaSize<2>(width, height));
    auto depthMap_dmp = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    int block_size = 16;
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    if(verbose)
        printf("copy depth map to host\n");

    depthMap_hmh->copyFrom( *depthMap_dmp->mem );

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_dmp );
    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

// void ps_computeNormalMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
void ps_computeNormalMap( CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
                         CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                         const cameraStruct& cams, int width, int height,
                         int scale, int CUDAdeviceNo,
                         // int ncamsAllocated,
                         int scales, int wsh, bool verbose,
                         float gammaC, float gammaP)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto depthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        depthMap_hmh->getSize()[0],
        depthMap_hmh->getSize()[1] );
    depthMap_arr->mem->copyFrom( *depthMap_hmh );
    cudaTextureObject_t depthsTex = depthMap_arr->tex;

    ps_init_reference_camera_matrices(*cams.cam);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams.camId );

    CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(*normalMap_hmh);

    int block_size = 16;
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    if(verbose)
        printf("copy normal map to host\n");

    normalMap_hmh->copyFrom( normalMap_dmp );
    CHECK_CUDA_ERROR();

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));

    global_data.pitched_mem_float_point_tex_cache.put( depthMap_arr );
}

// void ps_alignSourceDepthMapToTarget(CudaArray<uchar4, 2>** ps_texs_arr,
void ps_alignSourceDepthMapToTarget(
                                    CudaHostMemoryHeap<float, 2>* sourceDepthMap_hmh,
                                    CudaHostMemoryHeap<float, 2>* targetDepthMap_hmh,
                                    const cameraStruct& cams, int width,
                                    int height, int scale, int CUDAdeviceNo,
                                    // int ncamsAllocated,
                                    int scales, int wsh,
                                    bool verbose, float gammaC, float maxPixelSizeDist)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    auto sourceDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        sourceDepthMap_hmh->getSize()[0],
        sourceDepthMap_hmh->getSize()[1] );
    sourceDepthMap_arr->mem->copyFrom( *sourceDepthMap_hmh );
    cudaTextureObject_t depthsTex = sourceDepthMap_arr->tex;

    auto targetDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        targetDepthMap_hmh->getSize()[0],
        targetDepthMap_hmh->getSize()[1] );
    targetDepthMap_arr->mem->copyFrom( *targetDepthMap_hmh );
    cudaTextureObject_t depthsTex1 = targetDepthMap_arr->tex;

    CudaDeviceMemoryPitched<float, 2> outDepthMap_dmp(CudaSize<2>(width, height));

    ps_init_reference_camera_matrices( *cams.cam );

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams.camId );

    int block_size = 16;
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    if(verbose)
        printf("copy depth map to host\n");

    sourceDepthMap_hmh->copyFrom( outDepthMap_dmp );

    global_data.pitched_mem_float_point_tex_cache.put( sourceDepthMap_arr );
    global_data.pitched_mem_float_point_tex_cache.put( targetDepthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 0
void ps_dilateDepthMap(CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
                       CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp, int width, int height, bool verbose,
                       int niters, float gammaC)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    odepthMap_dmp.copyFrom( idepthMap_dmp );

    auto sourceDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get(
        odepthMap_dmp.getSize()[0],
        odepthMap_dmp.getSize()[1] );
    cudaTextureObject_t depthsTex = sourceDepthMap_arr->tex;

    for(int iter = 0; iter <= niters; iter++)
    {
        sourceDepthMap_arr->mem->copyFrom( odepthMap_dmp );

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
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, -1, 0, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, +1, 0, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, 0, -1, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                 width, height, 0, +1, fpPlaneDepth);
    }

    refine_convertFPPlaneDepthMapToDepthMap_kernel<<<grid, block>>>(depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                    depthMap_dmp.getBuffer(), depthMap_dmp.stride()[0],
                                                                    width, height);
}
#endif

static void ps_refineDepthMapInternal(
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
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
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        tTexU4 = timg_dmp->tex;

        refine_compYKNCCSim_kernel<<<grid, block>>>(
            rTexU4,
            tTexU4,
            ssm_dmp.getBuffer(), ssm_dmp.stride()[0],
            id,
            idepthMapMask_dmp.getBuffer(), idepthMapMask_dmp.stride()[0],
            width, height, wsh, gammaC, gammaP);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    }

    refine_computeBestDepthSimMaps_kernel<<<grid, block>>>(
        osimMap_dmp.getBuffer(), osimMap_dmp.stride()[0],
        odepthMap_dmp.getBuffer(), odepthMap_dmp.stride()[0],
        ssm_dmp.getBuffer(), ssm_dmp.stride()[0],
        dsm_dmp.getBuffer(), dsm_dmp.stride()[0],
        width, height, simThr);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
}

#if 0
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

    ps_init_reference_camera_matrices(cams[0]->cam );
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->cam );
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

    osim_hmh->copyFrom( finSimMap_dmp );
    odpt_hmh->copyFrom( finDptMap_dmp );

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
}
#endif

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

    ps_init_reference_camera_matrices( *cams[0]->cam );
    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );

    int c = 1;
    ps_init_target_camera_matrices( *cams[c]->cam );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    auto rimg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );
    auto timg_dmp = global_data.pitched_mem_uchar4_point_tex_cache.get( width, height );

    getRefTexLAB_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        rimg_dmp->mem->getBuffer(), rimg_dmp->mem->stride()[0],
        width, height );
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

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

    osim_hmh->copyFrom( bsim_dmp );
    odpt_hmh->copyFrom( bdpt_dmp );

    global_data.pitched_mem_uchar4_point_tex_cache.put( rimg_dmp );
    global_data.pitched_mem_uchar4_point_tex_cache.put( timg_dmp );
}

#if 0
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

    ps_init_reference_camera_matrices(cams[0]->cam);

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->cam);

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
        odepthSimMaps_hmh[i]->copyFrom( dsm_dmp );
    }

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}
#endif

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

    ps_init_reference_camera_matrices(*cams[0]->cam);

    int c = 1;
    ps_init_target_camera_matrices(*cams[c]->cam);

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

    osimMap_hmh->copyFrom( osimMap_dmp );

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

    ps_init_reference_camera_matrices(*cams[0]->cam);

    int c = 1;
    ps_init_target_camera_matrices(*cams[c]->cam);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams[0]->camId );
    cudaTextureObject_t t4tex = global_data.getScaledPictureTex( scale, cams[c]->camId );

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(width, height));
    copy(rcDepthMap_dmp, rcDepthMap_hmh, width, height);
    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(width, height));

    clock_t tall = tic();

#ifdef MERGE_REFINE_KERNELS
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
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_compYKNCCSimMapPatch_kernel_A<<<grid, block>>>(
        r4tex,
        t4tex,
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        -1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight,
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0], 0 );
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_compYKNCCSimMapPatch_kernel_A<<<grid, block>>>(
        r4tex,
        t4tex,
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        +1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight,
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0], 2 );
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

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
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0],
        width, height, 1);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift,
        -1.0f,
        moveByTcOrRc, xFrom, imWidth, imHeight);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        width, height, 0);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        r4tex,
        t4tex,
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        width, height,
        wsh, gammaC, gammaP, epipShift, +1.0f, moveByTcOrRc, xFrom, imWidth, imHeight);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        simMap_dmp.getBuffer(), simMap_dmp.stride()[0],
        width, height, 2);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
#endif

    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.stride()[0],
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.stride()[0],
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.stride()[0],
        width, height,
        moveByTcOrRc, xFrom);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

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
        depthSimMaps_dmp[i]->copyFrom( *depthSimMaps_hmh[i] );
    }

#if 0
    /* Hint: device-specific ideal block sizes can be found by passing functions
     * to cudaOccupancyMaxPotentialBlockSize()
     * The result doesn't change, you could keep it in a static variable.
     */

    {
        int minGridSize;
        int blockSize;
        cudaError_t err = cudaOccupancyMaxPotentialBlockSize( &minGridSize, &blockSize, fuse_computeGaussianKernelVotingSampleMap_kernel );
    }
#endif

    for(int s = -nSamplesHalf; s <= nSamplesHalf; s++) // (-150, 150)
    {
        for(int c = 1; c < ndepthSimMaps; c++) // number of Tc cameras
        {
            fuse_computeGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
                gsvSampleMap_dmp.getBuffer(), gsvSampleMap_dmp.stride()[0],
                depthSimMaps_dmp[c]->getBuffer(), depthSimMaps_dmp[c]->stride()[0],
                depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->stride()[0],
                width, height, (float)s, c - 1, samplesPerPixSize, twoTimesSigmaPowerTwo);
            memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
        }
        fuse_updateBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
            bestGsvSampleMap_dmp.getBuffer(), bestGsvSampleMap_dmp.stride()[0], gsvSampleMap_dmp.getBuffer(),
            gsvSampleMap_dmp.stride()[0], width, height, (float)s, s + nSamplesHalf);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }

    fuse_computeFusedDepthSimMapFromBestGaussianKernelVotingSampleMap_kernel<<<grid, block>>>(
        bestDepthSimMap_dmp.getBuffer(), bestDepthSimMap_dmp.stride()[0], bestGsvSampleMap_dmp.getBuffer(),
        bestGsvSampleMap_dmp.stride()[0], depthSimMaps_dmp[0]->getBuffer(), depthSimMaps_dmp[0]->stride()[0], width, height,
        samplesPerPixSize);
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    odepthSimMap_hmh->copyFrom( bestDepthSimMap_dmp );

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
                                           const cameraStruct& cams,
					   // int ncams,
					   int width, int height, int scale,
                                           int CUDAdeviceNo,
					   // int ncamsAllocated,
					   int scales, bool verbose, int yFrom)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    float samplesPerPixSize = (float)(nSamplesHalf / ((nDepthsToRefine - 1) / 2));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(*cams.cam);

    cudaTextureObject_t r4tex = global_data.getScaledPictureTex( scale, cams.camId );

    CudaDeviceMemoryPitched<float2, 2>** dataMaps_dmp;
    dataMaps_dmp = new CudaDeviceMemoryPitched<float2, 2>*[ndataMaps];
    for(int i = 0; i < ndataMaps; i++)
    {
        dataMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        dataMaps_dmp[i]->copyFrom( *dataMaps_hmh[i] );
    }

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(CudaSize<2>(width, height));
    optDepthSimMap_dmp.copyFrom( *dataMaps_dmp[0] );

    auto optDepthMap_arr = global_data.pitched_mem_float_point_tex_cache.get( width, height );

    for(int iter = 0; iter < nIters; iter++) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOPtDepthSimMap_kernel<<<grid, block>>>(
            optDepthMap_dmp.getBuffer(), optDepthMap_dmp.stride()[0], optDepthSimMap_dmp.getBuffer(),
            optDepthSimMap_dmp.stride()[0], width, height);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

        optDepthMap_arr->mem->copyFrom( optDepthMap_dmp );

        // Adjust depth/sim by using previously computed depths (depthTex is accessed inside this kernel)
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(
            r4tex,
            optDepthMap_arr->tex,
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.stride()[0],
            dataMaps_dmp[0]->getBuffer(), dataMaps_dmp[0]->stride()[0],
            dataMaps_dmp[1]->getBuffer(), dataMaps_dmp[1]->stride()[0],
            width, height, iter, samplesPerPixSize, yFrom);
        memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );
    }

    odepthSimMap_hmh->copyFrom( optDepthSimMap_dmp );

    for(int i = 0; i < ndataMaps; i++)
    {
        delete dataMaps_dmp[i];
    }
    delete[] dataMaps_dmp;

    global_data.pitched_mem_float_point_tex_cache.put( optDepthMap_arr );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

#if 0
// void ps_GC_aggregatePathVolume(CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
//                                CudaHostMemoryHeap<unsigned int, 3>& ivol_hmh, int volDimX, int volDimY, int volDimZ)
// {
//     CudaDeviceMemoryPitched<unsigned int, 3> vol_dmp(ivol_hmh);
// 
//     ///////////////////////////////////////////////////////////////////////////////
//     // setup block and grid
//     int block_size = 16;
//     dim3 blockvol(block_size, block_size, 1);
//     dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);
// 
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_x(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_y(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_z(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_w(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<unsigned int, 2> oxyslice_dmp(CudaSize<2>(volDimX, volDimY));
// 
//     volume_GC_K_initXYSliceInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
//                                                               xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
//                                                               xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
//                                                               xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
//                                                               vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
//                                                               volDimX, volDimY, 0);
//     CHECK_CUDA_ERROR();
// 
//     for(int z = 1; z < volDimZ; z++)
//     {
//         update_GC_volumeXYSliceAtZInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
// 								     xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
// 								     xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
// 								     xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
//                                                                      vol_dmp.getBuffer(), vol_dmp.stride()[1],
//                                                                      vol_dmp.stride()[0], volDimX, volDimY, volDimZ, z);
//         CHECK_CUDA_ERROR();
//     }
// 
//     volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel<<<gridvol, blockvol>>>(
//         oxyslice_dmp.getBuffer(),  oxyslice_dmp.stride()[0],
// 	xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
// 	xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
// 	xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
// 	xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
// 	volDimX, volDimY);
// 
//     ftid_hmh->copyFrom( oxyslice_dmp );
//     // pr_printfDeviceMemoryInfo();
// }
// 
// void ps_GC_K_aggregatePathVolume(CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
//                                  CudaHostMemoryHeap<unsigned int,
// 				 3>& ivol_hmh, int volDimX, int volDimY, int volDimZ,
//                                  int K)
// {
//     CudaDeviceMemoryPitched<unsigned int, 3> vol_dmp(ivol_hmh);
// 
//     ///////////////////////////////////////////////////////////////////////////////
//     // setup block and grid
//     int block_size = 16;
//     dim3 blockvol(block_size, block_size, 1);
//     dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);
// 
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_x(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_y(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_z(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<int, 2> xyslice_dmp_w(CudaSize<2>(volDimX, volDimY));
//     CudaDeviceMemoryPitched<unsigned int, 2> oxyslice_dmp(CudaSize<2>(volDimX, volDimY));
// 
//     for(int z = 0; z <= std::min(K - 1, volDimZ - 1); z++)
//     {
//         volume_GC_K_initXYSliceInt4_kernel<<<gridvol, blockvol>>>(xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
// 								  xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
// 								  xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
// 								  xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
//                                                                   vol_dmp.getBuffer(), vol_dmp.stride()[1],
//                                                                   vol_dmp.stride()[0], volDimX, volDimY, z);
//         CHECK_CUDA_ERROR();
//     }
// 
//     for(int z = 0; z <= volDimZ - K - 1; z++)
//     {
//         update_GC_K_volumeXYSliceAtZInt4_kernel<<<gridvol, blockvol>>>(
//             xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
//             xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
//             xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
//             xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
// 	    vol_dmp.getBuffer(), vol_dmp.stride()[1], vol_dmp.stride()[0],
//             volDimX, volDimY, volDimZ, z, K);
//         CHECK_CUDA_ERROR();
//     }
// 
//     volume_GC_K_getVolumeXYSliceInt4ToUintDimZ_kernel<<<gridvol, blockvol>>>(
//         oxyslice_dmp.getBuffer(),  oxyslice_dmp.stride()[0],
// 	xyslice_dmp_x.getBuffer(), xyslice_dmp_x.stride()[0],
// 	xyslice_dmp_y.getBuffer(), xyslice_dmp_y.stride()[0],
// 	xyslice_dmp_z.getBuffer(), xyslice_dmp_z.stride()[0],
// 	xyslice_dmp_w.getBuffer(), xyslice_dmp_w.stride()[0],
// 	volDimX,
//         volDimY);
// 
//     ftid_hmh->copyFrom( oxyslice_dmp );
//     // pr_printfDeviceMemoryInfo();
// }
#endif

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

    cudaTextureObject_t rTexU4 = global_data.getScaledPictureTexPoint( scale, camId );

    CudaDeviceMemoryPitched<bool, 2> map_dmp(CudaSize<2>(width / step, height / step));

    getSilhoueteMap_kernel<<<grid, block>>>(
        rTexU4,
        map_dmp.getBuffer(), map_dmp.stride()[0],
        step, width, height, maskColorLab );
    memOpErrorCheck( cudaGetLastError(), __FILE__, __LINE__, "Failed to execute kernel" );

    omap_hmh->copyFrom( map_dmp );

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

#if 0
// void ps_colorExtractionPushPull(CudaHostMemoryHeap<uchar4, 2>* bmp_hmh, int w, int h, int CUDAdeviceNo, bool verbose)
// {
//     clock_t tall = tic();
//     testCUDAdeviceNo(CUDAdeviceNo);
// 
//     int block_size = 16;
//     int npyramidLevels = 10;
// 
//     global_data.allocPyramidArrays( npyramidLevels, w, h );
// 
//     // pyramid arr
//     // CudaArray<uchar4, 2>** pyramid_arr = new CudaArray<uchar4, 2>*[npyramidLevels];
//     int wact = w;
//     int hact = h;
//     int wLevels[npyramidLevels];
//     int hLevels[npyramidLevels];
// 
//     // push
//     for(int i = 0; i < npyramidLevels; i++)
//     {
//         // pyramid_arr[i] = new CudaArray<uchar4, 2>(CudaSize<2>(wact, hact));
//         wLevels[i] = wact;
//         hLevels[i] = hact;
// 
//         if(i == 0)
//         {
//             global_data.getPyramidArray(i).copyFrom( *bmp_hmh );
//         }
//         else
//         {
//             // cudaBindTextureToArray(r4tex, pyramid_arr[i - 1]->getArray(), cudaCreateChannelDesc<uchar4>());
//             cudaTextureObject_t r4tex = global_data.getPyramidTex( i-1 );
// 
//             // CudaDeviceMemoryPitched<uchar4, 2> bmpNextLevel_dmp(CudaSize<2>(wact, hact));
//             CudaDeviceMemoryPitched<uchar4, 2>& bmpNextLevel_dmp = global_data.getPyramidArray( i );
//             dim3 block(block_size, block_size, 1);
//             dim3 grid(divUp(wact, block_size), divUp(hact, block_size), 1);
//             pushPull_Push_kernel<<<grid, block>>>( r4tex, bmpNextLevel_dmp.getBuffer(), bmpNextLevel_dmp.stride()[0], wact, hact);
//             CHECK_CUDA_ERROR();
// 
//             // copy((*pyramid_arr[i]), bmpNextLevel_dmp);
//             // copy( global_data.getPyramidArray(i), bmpNextLevel_dmp);
//         }
//         wact = wact / 2;
//         hact = hact / 2;
//         printf("push level %i\n", i);
//     }
// 
//     // pull
//     for(int i = npyramidLevels - 1; i >= 1; i--)
//     {
//         // cudaBindTextureToArray(r4tex, pyramid_arr[i]->getArray(), cudaCreateChannelDesc<uchar4>());
//         cudaTextureObject_t r4tex = global_data.getPyramidTex( i );
// 
//         // CudaDeviceMemoryPitched<uchar4, 2> bmpNextLevel_dmp(CudaSize<2>(wLevels[i - 1], hLevels[i - 1]));
//         // copy(bmpNextLevel_dmp, (*pyramid_arr[i - 1]));
//         CudaDeviceMemoryPitched<uchar4, 2>& bmpNextLevel_dmp = global_data.getPyramidArray( i-1 );
// 
//         dim3 block(block_size, block_size, 1);
//         dim3 grid(divUp(wLevels[i - 1], block_size), divUp(hLevels[i - 1], block_size), 1);
//         pushPull_Pull_kernel<<<grid, block>>>( r4tex, bmpNextLevel_dmp.getBuffer(), bmpNextLevel_dmp.stride()[0], wLevels[i - 1],
//                                               hLevels[i - 1]);
//         CHECK_CUDA_ERROR();
//         // cudaUnbindTexture(r4tex);
// 
//         // copy((*pyramid_arr[i - 1]), bmpNextLevel_dmp);
//         printf("pull level %i\n", i);
//     }
// 
//     bmp_hmh->copyFrom( global_data.getPyramidArray( 0 ) );
// 
//     global_data.freePyramidArrays();
// }
#endif

} // namespace depthMap
} // namespace aliceVision
