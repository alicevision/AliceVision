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

#include <math_constants.h>

#include <iostream>
#include <algorithm>

namespace aliceVision {
namespace depthMap {

// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                    \
    if(cudaError_t err = cudaGetLastError())                                  \
                                                                              \
{                                                                             \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));      \
        fprintf(stderr, "  file:       %s\n", __FILE__);                      \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                  \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                    \
                                                                              \
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

__host__ void ps_init_reference_camera_matrices(const float* _P, const float* _iP, const float* _R, const float* _iR, const float* _K, const float* _iK, const float* _C)
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

/*
void calcDCTCoefficients(void)
{
  float dctCoefficientsLoc[DCT_DIMENSION][DCT_DIMENSION][DCT_DIMENSION][DCT_DIMENSION];
  for (int p = 0; p < DCT_DIMENSION; p ++)
       for (int q = 0; q < DCT_DIMENSION; q ++)
           {
             double factor = (p == 0 ? 1 : sqrt (2.0)) * (q == 0 ? 1 : sqrt (2.0));
             for (int m = 0; m < DCT_DIMENSION; m ++)
                  for (int n = 0; n < DCT_DIMENSION; n ++)
                      {
                        dctCoefficientsLoc[p][q][m][n] = cos(CUDART_PI_F*(2*m + 1)*p/2/DCT_DIMENSION) *
cos(CUDART_PI_F*(2*n+1)*q/2/DCT_DIMENSION) * factor;
                      }
           }

   cudaMemcpyToSymbol(dctCoefficients, dctCoefficientsLoc,
sizeof(float)*DCT_DIMENSION*DCT_DIMENSION*DCT_DIMENSION*DCT_DIMENSION);
}
*/

__host__ cudaArray* ps_create_gaussian_arr(float delta, int radius)
{
    int size = 2 * radius + 1;

    float* d_gaussian;
    cudaMalloc((void**)&d_gaussian, (2 * radius + 1) * sizeof(float));

    // generate gaussian array
    generateGaussian_kernel<<<1, size>>>(d_gaussian, delta, radius);

    cudaArray* d_gaussianArray = NULL;

    // create cuda array
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
    cudaMallocArray(&d_gaussianArray, &channelDesc, size, 1);
    cudaMemcpyToArray(d_gaussianArray, 0, 0, d_gaussian, size * sizeof(float), cudaMemcpyDeviceToDevice);
    cudaFree(d_gaussian);

    return d_gaussianArray;
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

void ps_deviceAllocate(CudaArray<uchar4, 2>*** ps_texs_arr, int ncams, int width, int height, int scales,
                       int deviceId)
{
    int num_gpus = 0;
    cudaGetDeviceCount(&num_gpus);


    int outval = cudaSetDevice(deviceId);
    printf("CUDA device no %i for %i\n", outval, deviceId);

    // printf("ps_deviceAllocate\n");
    // pr_printfDeviceMemoryInfo();

    ///////////////////////////////////////////////////////////////////////////////
    // setup textures parameters
    rtex.filterMode = cudaFilterModeLinear;
    rtex.normalized = false;
    ttex.filterMode = cudaFilterModeLinear;
    ttex.normalized = false;

    r4tex.filterMode = cudaFilterModeLinear;
    r4tex.normalized = false;
    t4tex.filterMode = cudaFilterModeLinear;
    t4tex.normalized = false;

    rTexU4.filterMode = cudaFilterModePoint;
    rTexU4.normalized = false;
    tTexU4.filterMode = cudaFilterModePoint;
    tTexU4.normalized = false;

    volPixsTex.filterMode = cudaFilterModePoint;
    volPixsTex.normalized = false;
    pixsTex.filterMode = cudaFilterModePoint;
    pixsTex.normalized = false;
    gradTex.filterMode = cudaFilterModePoint;
    gradTex.normalized = false;
    depthsTex.filterMode = cudaFilterModePoint;
    depthsTex.normalized = false;
    depthsTex1.filterMode = cudaFilterModePoint;
    depthsTex1.normalized = false;
    normalsTex.filterMode = cudaFilterModePoint;
    normalsTex.normalized = false;
    sliceTex.filterMode = cudaFilterModePoint;
    sliceTex.normalized = false;
    sliceTexFloat2.filterMode = cudaFilterModePoint;
    sliceTexFloat2.normalized = false;
    sliceTexUChar.filterMode = cudaFilterModePoint;
    sliceTexUChar.normalized = false;
    sliceTexUInt2.filterMode = cudaFilterModePoint;
    sliceTexUInt2.normalized = false;
    sliceTexUInt.filterMode = cudaFilterModePoint;
    sliceTexUInt.normalized = false;
    f4Tex.filterMode = cudaFilterModePoint;
    f4Tex.normalized = false;

    pr_printfDeviceMemoryInfo();

    ///////////////////////////////////////////////////////////////////////////////
    // copy textures to the device
    int allBytes = 0;
    (*ps_texs_arr) = new CudaArray<uchar4, 2>*[scales * ncams];
    for(int c = 0; c < ncams; c++)
    {
        for(int s = 0; s < scales; s++)
        {
            int w = width / (s + 1);
            int h = height / (s + 1);
            (*ps_texs_arr)[c * scales + s] = new CudaArray<uchar4, 2>(CudaSize<2>(w, h));
            allBytes += (*ps_texs_arr)[c * scales + s]->getBytesUnpadded();
        }
    }
    CHECK_CUDA_ERROR();

    // calcDCTCoefficients();
    // CHECK_CUDA_ERROR();

    // printf("scales %i\n",scales);

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

void ps_deviceUpdateCam(CudaArray<uchar4, 2>** ps_texs_arr, cameraStruct* cam, int camId, int CUDAdeviceNo,
                        int ncamsAllocated, int scales, int w, int h, int varianceWsh)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    // compute gradient
    {
        CudaDeviceMemoryPitched<uchar4, 2> tex_lab_dmp(CudaSize<2>(w, h));
        copy(tex_lab_dmp, (*cam->tex_rgba_hmh));

        int block_size = 8;
        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(w, block_size), divUp(h, block_size), 1);
        rgb2lab_kernel<<<grid, block>>>( tex_lab_dmp.getBuffer(),
                                         tex_lab_dmp.getPitch(),
                                         w,
                                         h );
        copy((*ps_texs_arr[camId * scales + 0]), tex_lab_dmp);

        if(varianceWsh > 0)
        {
            cudaBindTextureToArray(r4tex, ps_texs_arr[camId * scales + 0]->getArray(), cudaCreateChannelDesc<uchar4>());
            compute_varLofLABtoW_kernel<<<grid, block>>>(tex_lab_dmp.getBuffer(),
                                                         tex_lab_dmp.getPitch(),
                                                         w, h,
                                                         varianceWsh);
            cudaUnbindTexture(r4tex);
            copy((*ps_texs_arr[camId * scales + 0]), tex_lab_dmp);
        }
    }

    cudaBindTextureToArray(r4tex, ps_texs_arr[camId * scales + 0]->getArray(), cudaCreateChannelDesc<uchar4>());

    // for each scale
    for(int scale = 1; scale < scales; scale++)
    {
        int radius = scale + 1;
        cudaArray* gaussian_arr = ps_create_gaussian_arr(1.0f, radius);
        cudaBindTextureToArray(gaussianTex, gaussian_arr, cudaCreateChannelDesc<float>());

        int block_size = 8;
        dim3 block(block_size, block_size, 1);
        dim3 grid(divUp(w / (scale + 1), block_size), divUp(h / (scale + 1), block_size), 1);

        CudaDeviceMemoryPitched<uchar4, 2> tex_lab_dmp(CudaSize<2>(w / (scale + 1), h / (scale + 1)));
        // downscale_bilateral_smooth_lab_kernel<<<grid, block>>>(
        downscale_gauss_smooth_lab_kernel<<<grid, block>>>(
            // downscale_mean_smooth_lab_kernel<<<grid, block>>>(
            tex_lab_dmp.getBuffer(),
            tex_lab_dmp.getPitch(),
            w / (scale + 1), h / (scale + 1), scale + 1,
            radius //, 15.5f
            );
        copy((*ps_texs_arr[camId * scales + scale]), tex_lab_dmp);

        if(varianceWsh > 0)
        {
            cudaUnbindTexture(r4tex);
            cudaBindTextureToArray(r4tex, ps_texs_arr[camId * scales + scale]->getArray(),
                                   cudaCreateChannelDesc<uchar4>());
            compute_varLofLABtoW_kernel<<<grid, block>>>(
                tex_lab_dmp.getBuffer(),
                tex_lab_dmp.getPitch(),
                w / (scale + 1), h / (scale + 1), varianceWsh);
            cudaUnbindTexture(r4tex);
            copy((*ps_texs_arr[camId * scales + scale]), tex_lab_dmp);
            cudaBindTextureToArray(r4tex, ps_texs_arr[camId * scales + 0]->getArray(), cudaCreateChannelDesc<uchar4>());
        }

        cudaUnbindTexture(gaussianTex);
        cudaFreeArray(gaussian_arr);
    }

    cudaUnbindTexture(r4tex);

    CHECK_CUDA_ERROR();
}

void ps_deviceDeallocate(CudaArray<uchar4, 2>*** ps_texs_arr, int CUDAdeviceNo, int ncams, int scales)
{
    testCUDAdeviceNo(CUDAdeviceNo);

    for(int c = 0; c < ncams * scales; c++)
    {
        delete(*ps_texs_arr)[c];
    }
    delete[](*ps_texs_arr);
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

    CudaDeviceMemoryPitched<unsigned char, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));
    CudaArray<unsigned char, 2> xyslice_arr(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<unsigned char, 2> xsliceBestInColSim_dmp(CudaSize<2>(volDimX, 1));
    // CudaDeviceMemoryPitched<int,2>    xsliceBestInColRow_dmp(CudaSize<2>(volDimX, 1));

    for(int z = 0; z < volDimZ; z++)
    {
        volume_agregateCostVolumeAtZ_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            vol_dmp.getBuffer(),
            vol_dmp.getBytesPaddedUpToDim(1),
            vol_dmp.getBytesPaddedUpToDim(0),
            xsliceBestInColSim_dmp.getBuffer(), volDimX,
            volDimY, volDimZ, z, P1, P2, transfer);
        CHECK_CUDA_ERROR();

        volume_getVolumeXYSliceAtZ_kernel<unsigned char, unsigned char><<<gridvol, blockvol>>>(
            xyslice_dmp.getBuffer(),
            xyslice_dmp.getPitch(),
            vol_dmp.getBuffer(),
            vol_dmp.getBytesPaddedUpToDim(1),
            vol_dmp.getBytesPaddedUpToDim(0),
            volDimX, volDimY, volDimZ, z);
        CHECK_CUDA_ERROR();
        copy((xyslice_arr), xyslice_dmp);
        cudaBindTextureToArray(sliceTexUChar, xyslice_arr.getArray(), cudaCreateChannelDesc<unsigned char>());
        volume_computeBestXSlice_kernel<<<gridvolrow, blockvolrow>>>(xsliceBestInColSim_dmp.getBuffer(),
                                                                     // xsliceBestInColRow_dmp.getBuffer(),
                                                                     volDimX, volDimY);
        CHECK_CUDA_ERROR();
        cudaUnbindTexture(sliceTexUChar);
    }

    // pr_printfDeviceMemoryInfo();
}

/**
 * @param[inout] d_volSimT similarity volume with some transposition applied
 */
void ps_aggregatePathVolume(CudaDeviceMemoryPitched<unsigned char, 3>& d_volSimT,
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
    CudaArray<unsigned int, 2> xySliceForZM1_arr(CudaSize<2>(volDimX, volDimY));
    CudaDeviceMemoryPitched<unsigned int, 2> d_xSliceBestInColSimForZM1(CudaSize<2>(volDimX, 1));

    // Copy the first Z plane from 'd_volSimT' into 'xysliceForZ_dmp'
    volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned char><<<gridvol, blockvol>>>(
        d_xySliceForZ.getBuffer(),
        d_xySliceForZ.getPitch(),
        d_volSimT.getBuffer(),
        d_volSimT.getBytesPaddedUpToDim(1),
        d_volSimT.getBytesPaddedUpToDim(0),
        volDimX, volDimY, volDimZ, 0); // Z=0
    CHECK_CUDA_ERROR();

    // Set the first Z plane from 'd_volSimT' to 255
    volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
        d_volSimT.getBuffer(),
        d_volSimT.getBytesPaddedUpToDim(1),
        d_volSimT.getBytesPaddedUpToDim(0),
        volDimX, volDimY, volDimZ, 0, 255);
    CHECK_CUDA_ERROR();

    for(int z = 1; z < volDimZ; z++)
    {
        copy(d_xySliceForZM1, d_xySliceForZ);
        copy((xySliceForZM1_arr), d_xySliceForZM1);
        cudaBindTextureToArray(sliceTexUInt, xySliceForZM1_arr.getArray(), cudaCreateChannelDesc<unsigned int>());
        // For each column: compute the best score
        // Foreach x:
        //   d_xSliceBestInColSimForZM1[x] = min(d_xySliceForZ[1:height])
        volume_computeBestXSliceUInt_kernel<<<gridvolrow, blockvolrow>>>(
            d_xSliceBestInColSimForZM1.getBuffer(),
            volDimX, volDimY);
        CHECK_CUDA_ERROR();
        cudaUnbindTexture(sliceTexUInt);

        // Copy the 'z' plane from 'd_volSimT' into 'd_xySliceForZ'
        volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned char><<<gridvol, blockvol>>>(
            d_xySliceForZ.getBuffer(),
            d_xySliceForZ.getPitch(),
            d_volSimT.getBuffer(),
            d_volSimT.getBytesPaddedUpToDim(1),
            d_volSimT.getBytesPaddedUpToDim(0),
            volDimX, volDimY, volDimZ, z);
        CHECK_CUDA_ERROR();

        volume_agregateCostVolumeAtZinSlices_kernel<<<gridvolrowAllCols, blockvolrow>>>(
            d_xySliceForZ.getBuffer(), d_xySliceForZ.getPitch(),              // inout: xySliceForZ
            d_xySliceForZM1.getBuffer(), d_xySliceForZM1.getPitch(),          // in:    xySliceForZM1
            d_xSliceBestInColSimForZM1.getBuffer(),                          // in:    xSliceBestInColSimForZM1
            d_volSimT.getBuffer(),
            d_volSimT.getBytesPaddedUpToDim(1),
            d_volSimT.getBytesPaddedUpToDim(0), // out:   volSimT
            volDimX, volDimY, volDimZ,
            z, P1, P2, transfer, volLUX,
            volLUY, dimTrnX, doInvZ);
        CHECK_CUDA_ERROR();
    }

    if(verbose)
        printf("ps_aggregatePathVolume done\n");
}

/**
 * @param[out] volAgr_dmp output volume where we will aggregate the best XXX
 * @param[in] d_volSim input similarity volume
 */
void ps_updateAggrVolume(CudaDeviceMemoryPitched<unsigned char, 3>& volAgr_dmp,
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

    if(doInvZ == true)
    {
        // clock_t tall = tic();
        for(int z = 0; z < volDims[dimsTrn[2]] / 2; z++)
        {
            volume_shiftZVolumeTempl_kernel<unsigned char><<<gridT, blockT>>>(
                d_volSimT.getBuffer(),
                d_volSimT.getBytesPaddedUpToDim(1),
                d_volSimT.getBytesPaddedUpToDim(0),
                volDims[dimsTrn[0]], volDims[dimsTrn[1]], volDims[dimsTrn[2]],
                z);
        }
        CHECK_CUDA_ERROR();
        // if (verbose) printf("shift z volume gpu elapsed time: %f ms \n", toc(tall));
    }

    // clock_t tall = tic();
    ps_aggregatePathVolume(
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
                d_volSimT.getBuffer(),
                d_volSimT.getBytesPaddedUpToDim(1),
                d_volSimT.getBytesPaddedUpToDim(0),
                volDims[dimsTrn[0]],
                volDims[dimsTrn[1]],
                volDims[dimsTrn[2]],
                z);
        }
        CHECK_CUDA_ERROR();
        // if (verbose) printf("shift z volume gpu elapsed time: %f ms \n", toc(tall));
    }

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
void ps_SGMoptimizeSimVolume(CudaArray<uchar4, 2>** ps_texs_arr,
                             cameraStruct* rccam,
                             unsigned char* iovol_hmh,
                             int volDimX, int volDimY, int volDimZ,
                             int volStepXY, int volLUX, int volLUY,
                             bool verbose, unsigned char P1, unsigned char P2,
                             int scale, int CUDAdeviceNo, int ncamsAllocated, int scales)
{
    if(verbose)
        printf("ps_SGMoptimizeSimVolume\n");

    ps_init_reference_camera_matrices(rccam->P, rccam->iP, rccam->R, rccam->iR, rccam->K, rccam->iK, rccam->C);

    // bind 'r4tex' from the image in Lab colorspace at the scale used
    cudaBindTextureToArray(r4tex, ps_texs_arr[rccam->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    //CudaDeviceMemoryPitched<unsigned char, 3> volSim_dmp(*iovol_hmh);
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
                                      ps_updateAggrVolume(volAgr_dmp,
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

    cudaUnbindTexture(r4tex);

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

void ps_computeSimilarityVolume(CudaArray<uchar4, 2>** ps_texs_arr,
                                CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp, cameraStruct** cams, int ncams,
                                int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ, int volLUX,
                                int volLUY, int volLUZ, CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
                                CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
                                int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
                                int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, bool doUsePixelsDepths,
                                int nbest, bool useTcOrRcPixSize, float gammaC, float gammaP, bool subPixel,
                                float epipShift)
{
    clock_t tall = tic();
    testCUDAdeviceNo(CUDAdeviceNo);

    if(verbose)
        printf("nDepths %i, nDepthsToSearch %i \n", nDepths, nDepthsToSearch);
    CudaArray<int4, 2> volPixs_arr(volPixs_hmh);
    CudaArray<float, 2> depths_arr(depths_hmh);

    cudaBindTextureToArray(volPixsTex, volPixs_arr.getArray(), cudaCreateChannelDesc<int4>());
    cudaBindTextureToArray(depthsTex, depths_arr.getArray(), cudaCreateChannelDesc<float>());

    int block_size = 8;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(nDepthsToSearch, block_size), divUp(slicesAtTime, block_size), 1);
    dim3 blockvol(block_size, block_size, 1);
    dim3 gridvol(divUp(volDimX, block_size), divUp(volDimY, block_size), 1);

    // setup cameras matrices to the constant memory
    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaBindTextureToArray(r4tex, ps_texs_arr[cams[0]->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaBindTextureToArray(t4tex, ps_texs_arr[cams[c]->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    //--------------------------------------------------------------------------------------------------
    // init similarity volume
    for(int z = 0; z < volDimZ; z++)
    {
        volume_initVolume_kernel<unsigned char><<<gridvol, blockvol>>>(
            vol_dmp.getBuffer(),
            vol_dmp.getBytesPaddedUpToDim(1),
            vol_dmp.getBytesPaddedUpToDim(0),
            volDimX, volDimY, volDimZ, z, 255);
    }

    //--------------------------------------------------------------------------------------------------
    // compute similarity volume
    CudaDeviceMemoryPitched<unsigned char, 2> slice_dmp(CudaSize<2>(nDepthsToSearch, slicesAtTime));
    for(int t = 0; t < ntimes; t++)
    {
        volume_slice_kernel<<<grid, block>>>(slice_dmp.getBuffer(),
                                             slice_dmp.getPitch(),
                                             nDepthsToSearch, nDepths,
                                             slicesAtTime, width, height, wsh, t, npixs, gammaC, gammaP, epipShift);

        volume_saveSliceToVolume_kernel<<<grid, block>>>(vol_dmp.getBuffer(),
                                                         vol_dmp.getBytesPaddedUpToDim(1),
                                                         vol_dmp.getBytesPaddedUpToDim(0),
                                                         slice_dmp.getBuffer(),
                                                         slice_dmp.getPitch(),
                                                         nDepthsToSearch,
                                                         nDepths, slicesAtTime, width, height, t, npixs, volStepXY,
                                                         volDimX, volDimY, volDimZ, volLUX, volLUY, volLUZ);
        CHECK_CUDA_ERROR();
    }

    cudaUnbindTexture(r4tex);
    cudaUnbindTexture(t4tex);
    cudaUnbindTexture(volPixsTex);
    cudaUnbindTexture(depthsTex);

    if(verbose)
        printf("ps_computeSimilarityVolume elapsed time: %f ms \n", toc(tall));
}

float ps_planeSweepingGPUPixelsVolume(CudaArray<uchar4, 2>** ps_texs_arr,
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
        printf("total size of volume map in GPU memory: %f\n", (float)volSim_dmp.getBytesPadded() / (1024.0f * 1024.0f));

    //--------------------------------------------------------------------------------------------------
    // compute similarity volume
    ps_computeSimilarityVolume(ps_texs_arr, volSim_dmp, cams, ncams, width, height, volStepXY, volDimX, volDimY,
                               volDimZ, volLUX, volLUY, volLUZ, volPixs_hmh, depths_hmh, nDepthsToSearch, slicesAtTime,
                               ntimes, npixs, wsh, kernelSizeHalf, nDepths, scale, CUDAdeviceNo, ncamsAllocated, scales,
                               verbose, doUsePixelsDepths, nbest, useTcOrRcPixSize, gammaC, gammaP, subPixel, epipShift);

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy(ovol_hmh, volDimX, volDimY, volDimZ, volSim_dmp);

    return (float)volSim_dmp.getBytesPadded() / (1024.0f * 1024.0f);
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
    // CudaDeviceMemoryPitched<unsigned int,3>  ovol_dmp(CudaSize<3>(volDimX, volDimY, volDimZ));
    CudaDeviceMemoryPitched<unsigned int, 3> ovol_dmp(ivol_dmp);

    CudaDeviceMemoryPitched<unsigned int, 2> xyslice_dmp(CudaSize<2>(volDimX, volDimY));
    CudaArray<unsigned int, 2> xyslice_arr(CudaSize<2>(volDimX, volDimY));

    /*
    //--------------------------------------------------------------------------------------------------
    //init similarity volume
    for (int z=0;z<volDimZ;z++)	{
            volume_initVolume_kernel<unsigned int><<<gridvol, blockvol>>>(
                    ovol_dmp.getBuffer(), ovol_dmp.stride()[1], ovol_dmp.stride()[0],
                    volDimX, volDimY, volDimZ, z, 0);
    }
    */

    for(int K = -3; K <= 3; K++)
    {
        for(int z = 0; z < volDimZ; z++)
        {
            if((z + K >= 0) && (z + K < volDimZ))
            {
                volume_getVolumeXYSliceAtZ_kernel<unsigned int, unsigned int><<<gridvol, blockvol>>>(
                    xyslice_dmp.getBuffer(),
                    xyslice_dmp.getPitch(),
                    ivol_dmp.getBuffer(),
                    ivol_dmp.getBytesPaddedUpToDim(1),
                    ivol_dmp.getBytesPaddedUpToDim(0),
                    volDimX, volDimY, volDimZ,
                    z + K);

                copy((xyslice_arr), xyslice_dmp);
                cudaBindTextureToArray(sliceTexUInt, xyslice_arr.getArray(), cudaCreateChannelDesc<unsigned int>());
                volume_filter_VisTVolume_kernel<<<gridvol, blockvol>>>(
                    ovol_dmp.getBuffer(),
                    ovol_dmp.getBytesPaddedUpToDim(1),
                    ovol_dmp.getBytesPaddedUpToDim(0),
                    volDimX, volDimY, volDimZ,
                    z, K);
                CHECK_CUDA_ERROR();
                cudaUnbindTexture(sliceTexUInt);
            }
        }
    }

    //--------------------------------------------------------------------------------------------------
    // copy to host
    copy((*iovol_hmh), ovol_dmp);

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
        volume_compute_rDP1_kernel<<<gridvol, blockvol>>>(
            xySlice_dmp.getBuffer(), xySlice_dmp.getPitch(),
            ovol_dmp.getBuffer(),
            ovol_dmp.getBytesPaddedUpToDim(1),
            ovol_dmp.getBytesPaddedUpToDim(0),
            ivol_dmp.getBuffer(),
            ivol_dmp.getBytesPaddedUpToDim(1),
            ivol_dmp.getBytesPaddedUpToDim(0),
            volDimX, volDimY, volDimZ, z);
    }

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
                xySlice_dmp.getBuffer(), xySlice_dmp.getPitch(),
                iovol_dmp.getBuffer(),
                iovol_dmp.getBytesPaddedUpToDim(1),
                iovol_dmp.getBytesPaddedUpToDim(0),
                volDimX, volDimY, volDimZpart, z, zPart, volDimZ);
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
                xySlice_dmp.getBuffer(), xySlice_dmp.getPitch(),
                iovol_dmp.getBuffer(),
                iovol_dmp.getBytesPaddedUpToDim(1),
                iovol_dmp.getBytesPaddedUpToDim(0),
                volDimX, volDimY, volDimZpart, z, zPart, volDimZ);
        }

        //--------------------------------------------------------------------------------------------------
        // copy to host
        copy((*iovols_hmh[zPart]), iovol_dmp);

    } // for zPart

    if(verbose)
        printf("elapsed time: %f ms \n", toc(tall));
}

void ps_getTexture(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<uchar4, 2>* oimg_hmh, int camId,
                   int scale, int CUDAdeviceNo, int ncamsAllocated, int scales)
{
    clock_t tall = tic();

    copy((*oimg_hmh), (*ps_texs_arr[camId * scales + scale]));
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

    for(int iter = 0; iter <= niters; iter++)
    {
        CudaArray<float, 2> sourceDepthMap_arr(odepthMap_dmp);
        cudaBindTextureToArray(depthsTex, sourceDepthMap_arr.getArray(), cudaCreateChannelDesc<float>());
        refine_dilateDepthMap_kernel<<<grid, block>>>(
            odepthMap_dmp.getBuffer(),
            odepthMap_dmp.getPitch(),
            width, height,
            gammaC);
        cudaUnbindTexture(depthsTex);
    }
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
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            width, height, 0, 0, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            width, height, -1, 0, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            width, height, +1, 0, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            width, height, 0, -1, fpPlaneDepth);
        refine_dilateFPPlaneDepthMapXpYp_kernel<<<grid, block>>>(
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
            width, height, 0, +1, fpPlaneDepth);
    }

    refine_convertFPPlaneDepthMapToDepthMap_kernel<<<grid, block>>>(
        depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
        depthMap_dmp.getBuffer(), depthMap_dmp.getPitch(),
        width, height);
}

void ps_refineDepthMapInternal(CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
                               CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
                               CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp,
                               CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp, int width, int height,
                               bool verbose, int wsh, float gammaC, float gammaP, float simThr,
                               CudaDeviceMemoryPitched<float3, 2>& dsm_dmp,
                               CudaDeviceMemoryPitched<float3, 2>& ssm_dmp, CudaArray<uchar4, 2>& tTexU4_arr,
                               CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp, bool moveByTcOrRc, float step)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    // computed three depths map ... -1,0,1 ... from dilated input depth map .. and add last
    refine_computeDepthsMapFromDepthMap_kernel<<<grid, block>>>(
        dsm_dmp.getBuffer(), dsm_dmp.getPitch(),
        idepthMap_dmp.getBuffer(), idepthMap_dmp.getPitch(),
        width, height, moveByTcOrRc, step);

    // for each depth map compute sim map in pixels where depthMap is defined
    // int id = 1;
    for(int id = 0; id < 3; id++)
    {
        refine_reprojTarTexLABByDepthsMap_kernel<<<grid, block>>>(
            dsm_dmp.getBuffer(), dsm_dmp.getPitch(),
            timg_dmp.getBuffer(), timg_dmp.getPitch(),
            width, height, id);

        cudaUnbindTexture(tTexU4);
        copy((tTexU4_arr), timg_dmp);
        cudaBindTextureToArray(tTexU4, tTexU4_arr.getArray(), cudaCreateChannelDesc<uchar4>());

        refine_compYKNCCSim_kernel<<<grid, block>>>(
            ssm_dmp.getBuffer(), ssm_dmp.getPitch(),
            id,
            idepthMapMask_dmp.getBuffer(), idepthMapMask_dmp.getPitch(),
            width, height,
            wsh, gammaC, gammaP);

    }

    refine_computeBestDepthSimMaps_kernel<<<grid, block>>>(
        osimMap_dmp.getBuffer(),   osimMap_dmp.getPitch(),
        odepthMap_dmp.getBuffer(), odepthMap_dmp.getPitch(),
        ssm_dmp.getBuffer(),       ssm_dmp.getPitch(),
        dsm_dmp.getBuffer(),       dsm_dmp.getPitch(),
        width, height, simThr);
}

void ps_computeSimMapForDepthMapInternal(CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
                                         CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp, int width, int height,
                                         bool verbose, int wsh, float gammaC, float gammaP,
                                         CudaArray<uchar4, 2>& tTexU4_arr,
                                         CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp, float fpPlaneDepth)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    reprojTarTexLAB_kernel<<<grid, block>>>(
        timg_dmp.getBuffer(), timg_dmp.getPitch(),
        width, height, fpPlaneDepth);

    cudaUnbindTexture(tTexU4);
    copy((tTexU4_arr), timg_dmp);
    cudaBindTextureToArray(tTexU4, tTexU4_arr.getArray(), cudaCreateChannelDesc<uchar4>());

    refine_compYKNCCSimMap_kernel<<<grid, block>>>(
        osimMap_dmp.getBuffer(), osimMap_dmp.getPitch(),
        idepthMapMask_dmp.getBuffer(), idepthMapMask_dmp.getPitch(),
        width, height,
        wsh, gammaC, gammaP);
}

void ps_refineRcDepthMap(CudaArray<uchar4, 2>** ps_texs_arr, float* osimMap_hmh,
                         float* rcDepthMap_hmh, int ntcsteps,
                         cameraStruct** cams, int ncams, int width,
                         int height, int imWidth, int imHeight, int scale, int CUDAdeviceNo, int ncamsAllocated,
                         int scales, bool verbose, int wsh, float gammaC, float gammaP, float epipShift,
                         bool moveByTcOrRc, int xFrom)
{
    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaBindTextureToArray(r4tex, ps_texs_arr[cams[0]->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    int c = 1;
    ps_init_target_camera_matrices(cams[c]->P, cams[c]->iP, cams[c]->R, cams[c]->iR, cams[c]->K, cams[c]->iK,
                                   cams[c]->C);
    cudaBindTextureToArray(t4tex, ps_texs_arr[cams[c]->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    CudaDeviceMemoryPitched<float3, 2> lastThreeSimsMap(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> simMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> rcDepthMap_dmp(CudaSize<2>(width, height));
    copy(rcDepthMap_dmp, rcDepthMap_hmh, width, height);
    CudaDeviceMemoryPitched<float, 2> bestSimMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float, 2> bestDptMap_dmp(CudaSize<2>(width, height));

    clock_t tall = tic();

    for(int i = 0; i < ntcsteps; i++) // Default ntcsteps = 31
    {
        refine_compUpdateYKNCCSimMapPatch_kernel<<<grid, block>>>(
            bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
            bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
            rcDepthMap_dmp.getBuffer(), rcDepthMap_dmp.getPitch(),
            width, height, wsh, gammaC, gammaP, epipShift,
            (float)(i - (ntcsteps - 1) / 2), i, moveByTcOrRc, xFrom, imWidth, imHeight);
    }

    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        width, height, 1);

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        width,
        height, wsh, gammaC, gammaP, epipShift, -1.0f, moveByTcOrRc, xFrom, imWidth, imHeight);


    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        width, height, 0);

    refine_compYKNCCSimMapPatch_kernel<<<grid, block>>>(
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        width,
        height, wsh, gammaC, gammaP, epipShift, +1.0f, moveByTcOrRc, xFrom, imWidth, imHeight);


    refine_setLastThreeSimsMap_kernel<<<grid, block>>>(
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        simMap_dmp.getBuffer(), simMap_dmp.getPitch(),
        width, height, 2);

    refine_computeDepthSimMapFromLastThreeSimsMap_kernel<<<grid, block>>>(
        bestSimMap_dmp.getBuffer(), bestSimMap_dmp.getPitch(),
        bestDptMap_dmp.getBuffer(), bestDptMap_dmp.getPitch(),
        lastThreeSimsMap.getBuffer(), lastThreeSimsMap.getPitch(),
        width, height, moveByTcOrRc, xFrom);

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
    delete[] depthSimMaps_dmp;

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_optimizeDepthSimMapGradientDescent(CudaArray<uchar4, 2>** ps_texs_arr,
                                           CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                                           CudaHostMemoryHeap<float2, 2>** dataMaps_hmh, int ndataMaps,
                                           int nSamplesHalf, int nDepthsToRefine, int nIters, float sigma,
                                           cameraStruct** cams, int ncams, int width, int height, int scale,
                                           int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, int yFrom)
{
    clock_t tall = tic();

    float samplesPerPixSize = (float)(nSamplesHalf / ((nDepthsToRefine - 1) / 2));

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

    ps_init_reference_camera_matrices(cams[0]->P, cams[0]->iP, cams[0]->R, cams[0]->iR, cams[0]->K, cams[0]->iK,
                                      cams[0]->C);
    cudaBindTextureToArray(r4tex, ps_texs_arr[cams[0]->camId * scales + scale]->getArray(),
                           cudaCreateChannelDesc<uchar4>());

    CudaDeviceMemoryPitched<float2, 2>** dataMaps_dmp;
    dataMaps_dmp = new CudaDeviceMemoryPitched<float2, 2>*[ndataMaps];
    for(int i = 0; i < ndataMaps; i++)
    {
        dataMaps_dmp[i] = new CudaDeviceMemoryPitched<float2, 2>(CudaSize<2>(width, height));
        copy((*dataMaps_dmp[i]), (*dataMaps_hmh[i]));
    }

    CudaDeviceMemoryPitched<float, 2> optDepthMap_dmp(CudaSize<2>(width, height));
    CudaDeviceMemoryPitched<float2, 2> optDepthSimMap_dmp(CudaSize<2>(width, height));
    CudaArray<float, 2> optDepthMap_arr(CudaSize<2>(width, height));
    copy(optDepthSimMap_dmp, (*dataMaps_dmp[0]));

    for(int iter = 0; iter < nIters; iter++) // nIters: 100 by default
    {
        // Copy depths values from optDepthSimMap to optDepthMap
        fuse_getOptDeptMapFromOPtDepthSimMap_kernel<<<grid, block>>>(
            optDepthMap_dmp.getBuffer(), optDepthMap_dmp.getPitch(),
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
            width, height);
        copy((optDepthMap_arr), optDepthMap_dmp);
        // Bind those depth values as a texture
        cudaBindTextureToArray(depthsTex, optDepthMap_arr.getArray(), cudaCreateChannelDesc<float>());

        // Adjust depth/sim by using previously computed depths (depthTex is accessed inside this kernel)
        fuse_optimizeDepthSimMap_kernel<<<grid, block>>>(
            optDepthSimMap_dmp.getBuffer(), optDepthSimMap_dmp.getPitch(),
            dataMaps_dmp[0]->getBuffer(), dataMaps_dmp[0]->getPitch(),
            dataMaps_dmp[1]->getBuffer(), dataMaps_dmp[1]->getPitch(),
            width, height, iter, samplesPerPixSize, yFrom);

        cudaUnbindTexture(depthsTex);
    }

    copy((*odepthSimMap_hmh), optDepthSimMap_dmp);

    for(int i = 0; i < ndataMaps; i++)
    {
        delete dataMaps_dmp[i];
    }
    delete[] dataMaps_dmp;

    cudaUnbindTexture(r4tex);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}

void ps_getSilhoueteMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
                        int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int step, int camId,
                        uchar4 maskColorRgb, bool verbose)
{
    clock_t tall = tic();

    uchar4 maskColorLab;
    float3 flab = h_xyz2lab(h_rgb2xyz(uchar4_to_float3(maskColorRgb)));
    maskColorLab.x = (unsigned char)(flab.x);
    maskColorLab.y = (unsigned char)(flab.y);
    maskColorLab.z = (unsigned char)(flab.z);
    maskColorLab.w = 0;

    ///////////////////////////////////////////////////////////////////////////////
    // setup block and grid
    int block_size = 16;
    dim3 block(block_size, block_size, 1);
    dim3 grid(divUp(width / step, block_size), divUp(height / step, block_size), 1);

    cudaBindTextureToArray(rTexU4, ps_texs_arr[camId * scales + scale]->getArray(), cudaCreateChannelDesc<uchar4>());

    CudaDeviceMemoryPitched<bool, 2> map_dmp(CudaSize<2>(width / step, height / step));
    getSilhoueteMap_kernel<<<grid, block>>>(
        map_dmp.getBuffer(), map_dmp.getPitch(),
        step, width, height, maskColorLab);
    CHECK_CUDA_ERROR();

    cudaUnbindTexture(rTexU4);

    copy((*omap_hmh), map_dmp);

    if(verbose)
        printf("gpu elapsed time: %f ms \n", toc(tall));
}


void ps_computeNormalMap(CudaArray<uchar4, 2>** ps_texs_arr, CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
  CudaHostMemoryHeap<float, 2>* depthMap_hmh, const cameraStruct& camera, int width, int height,
  int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int wsh, bool verbose,
  float gammaC, float gammaP)
{
  clock_t tall = tic();
  testCUDAdeviceNo(CUDAdeviceNo);

  CudaArray<float, 2> depthMap_arr(*depthMap_hmh);
  cudaBindTextureToArray(depthsTex, depthMap_arr.getArray(), cudaCreateChannelDesc<float>());

  ps_init_reference_camera_matrices(camera.P, camera.iP, camera.R, camera.iR, camera.K, camera.iK, camera.C);

  CudaDeviceMemoryPitched<float3, 2> normalMap_dmp(*normalMap_hmh);

  int block_size = 8;
  dim3 block(block_size, block_size, 1);
  dim3 grid(divUp(width, block_size), divUp(height, block_size), 1);

  if (verbose)
    printf("computeNormalMap_kernel\n");

  //------------------------------------------------------------------------------------------------
  // compute normal map
  computeNormalMap_kernel<<<grid, block>>>(
    normalMap_dmp.getBuffer(),
    normalMap_dmp.getPitch(),
    width, height, wsh,
    gammaC, gammaP);
  cudaThreadSynchronize();
  CHECK_CUDA_ERROR();

  if (verbose)
    printf("copy normal map to host\n");

  copy((*normalMap_hmh), normalMap_dmp);
  CHECK_CUDA_ERROR();

  if (verbose)
    printf("gpu elapsed time: %f ms \n", toc(tall));

  cudaUnbindTexture(r4tex);
  cudaUnbindTexture(depthsTex);
}

} // namespace depthMap
} // namespace aliceVision
