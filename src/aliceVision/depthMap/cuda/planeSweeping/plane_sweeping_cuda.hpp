// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/depthMap/cuda/commonStructures.hpp"

namespace aliceVision {
namespace depthMap {

// Macro for checking cuda errors
#define CHECK_CUDA_ERROR()                                                    \
    cudaDeviceSynchronize();                                                  \
    if(cudaError_t err = cudaGetLastError())                                  \
                                                                              \
{                                                                             \
        fprintf(stderr, "\n\nCUDAError: %s\n", cudaGetErrorString(err));      \
        fprintf(stderr, "  file:       %s\n", __FILE__);                      \
        fprintf(stderr, "  function:   %s\n", __FUNCTION__);                  \
        fprintf(stderr, "  line:       %d\n\n", __LINE__);                    \
                                                                              \
}


// Round a / b to nearest higher integer value.
inline unsigned int divUp(unsigned int a, unsigned int b) {
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

float3 ps_M3x3mulV3(float* M3x3, const float3& V);

void ps_normalize(float3& a);

void pr_printfDeviceMemoryInfo();

float3 ps_getDeviceMemoryInfo();

void ps_init_reference_camera_matrices(
                    float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                    float* _C);

void ps_init_target_camera_matrices(
                    float* _P, float* _iP, float* _R, float* _iR, float* _K, float* _iK,
                    float* _C);

// opaque declaration
struct GaussianArray;

GaussianArray* ps_create_gaussian_arr(float delta, int radius);

int ps_listCUDADevices(bool verbose);

// void ps_deviceAllocate(
//                     CudaArray<uchar4, 2>*** ps_texs_arr,
//                     int ncams, int width, int height, int scales,
//                     int deviceId);
void ps_deviceAllocate(
                    int ncams, int width, int height, int scales,
                    int deviceId);

void testCUDAdeviceNo(int CUDAdeviceNo);

void ps_deviceUpdateCam(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    cameraStruct* cam, int camId, int CUDAdeviceNo,
                    int ncamsAllocated, int scales, int w, int h, int varianceWsh);

// void ps_deviceDeallocate(
//                     CudaArray<uchar4, 2>*** ps_texs_arr,
//                     int CUDAdeviceNo, int ncams, int scales);
void ps_deviceDeallocate(
                    int CUDAdeviceNo, int ncams, int scales);

void ps_aggregatePathVolume2(
                    CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp, int volDimX, int volDimY,
                    int volDimZ, float P1, float P2, bool transfer);

/**
 * @param[inout] d_volSimT similarity volume with some transposition applied
 */
void ps_aggregatePathVolume(
                    CudaDeviceMemoryPitched<unsigned char, 3>& d_volSimT,
                    int volDimX, int volDimY, int volDimZ,
                    float P1, float P2, bool transfer,
                    int volLUX, int volLUY,
                    int dimTrnX, bool doInvZ, bool verbose);

/**
 * @param[out] volAgr_dmp output volume where we will aggregate the best XXX
 * @param[in] d_volSim input similarity volume
 */
void ps_updateAggrVolume(
                    CudaDeviceMemoryPitched<unsigned char, 3>& volAgr_dmp,
                    const CudaDeviceMemoryPitched<unsigned char, 3>& d_volSim,
                    int volDimX, int volDimY, int volDimZ,
                    int volStepXY, int volLUX, int volLUY,
                    int dimTrnX, int dimTrnY, int dimTrnZ,
                    unsigned char P1, unsigned char P2, 
                    bool verbose, bool doInvZ, int lastN);

/**
* @param[in] ps_texs_arr table of image (in Lab colorspace) for all scales
* @param[in] rccam RC camera
* @param[inout] iovol_hmh input similarity volume (after Z reduction)
*/
void ps_SGMoptimizeSimVolume(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    cameraStruct* rccam,
                    unsigned char* iovol_hmh,
                    int volDimX, int volDimY, int volDimZ,
                    int volStepXY, int volLUX, int volLUY,
                    bool verbose, unsigned char P1, unsigned char P2,
                    int scale, int CUDAdeviceNo, int ncamsAllocated, int scales);

void ps_transposeVolume(
                    CudaHostMemoryHeap<unsigned char, 3>* ovol_hmh,
                    CudaHostMemoryHeap<unsigned char, 3>* ivol_hmh,
                    int volDimX, int volDimY, int volDimZ,
                    int dimTrnX, int dimTrnY, int dimTrnZ, bool verbose);


void ps_computeSimilarityVolume(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaDeviceMemoryPitched<unsigned char, 3>& vol_dmp,
                    cameraStruct** cams, int ncams,
                    int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ, int volLUX,
                    int volLUY, int volLUZ, CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
                    CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
                    int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, bool doUsePixelsDepths,
                    int nbest, bool useTcOrRcPixSize, float gammaC, float gammaP, bool subPixel,
                    float epipShift);

float ps_planeSweepingGPUPixelsVolume(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    unsigned char* ovol_hmh, cameraStruct** cams, int ncams,
                    int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ,
                    int volLUX, int volLUY, int volLUZ, CudaHostMemoryHeap<int4, 2>& volPixs_hmh,
                    CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepthsToSearch, int slicesAtTime,
                    int ntimes, int npixs, int wsh, int kernelSizeHalf, int nDepths, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                    bool doUsePixelsDepths, int nbest, bool useTcOrRcPixSize, float gammaC,
                    float gammaP, bool subPixel, float epipShift);

void ps_filterVisTVolume(
                    CudaHostMemoryHeap<unsigned int, 3>* iovol_hmh,
                    int volDimX, int volDimY, int volDimZ,
                    bool verbose);

void ps_enforceTweigthInVolumeInternal(
                    CudaDeviceMemoryPitched<unsigned int, 3>& ivol_dmp,
                    CudaDeviceMemoryPitched<unsigned int, 3>& ovol_dmp, int volDimX, int volDimY,
                    int volDimZ, bool verbose);

void ps_enforceTweigthInVolume(
                    CudaHostMemoryHeap<unsigned int, 3>* iovol_hmh,
                    int volDimX, int volDimY, int volDimZ,
                    bool verbose);

void ps_computeDP1Volume(
                    CudaHostMemoryHeap<int, 3>* ovol_hmh,
                    CudaHostMemoryHeap<unsigned int, 3>* ivol_hmh,
                    int volDimX, int volDimY, int volDimZ, bool verbose);

void ps_normalizeDP1Volume(
                    CudaHostMemoryHeap<int, 3>** iovols_hmh,
                    int nZparts, int volDimZpart, int volDimX,
                    int volDimY, int volDimZ, bool verbose);

void ps_computeRcVolumeForTcDepthSimMaps(
                    CudaHostMemoryHeap<unsigned int, 3>** ovols_hmh, int nZparts,
                    int volDimZpart, cameraStruct** cams, int ncams, float2* camsMinMaxFpDepths,
                    int width, int height, int volStepXY, int volDimX, int volDimY, int volDimZ,
                    CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales,
                    CudaHostMemoryHeap<float2, 2>** rcTcsDepthSimMaps_hmh, bool verbose,
                    float maxTcRcPixSizeInVoxRatio, bool considerNegativeDepthAsInfinity);

void ps_filterRcIdDepthMapByTcDepthMap(
                    CudaHostMemoryHeap<unsigned short, 2>* rcIdDepthMap_hmh, cameraStruct** cams,
                    int ncams, int width, int height, int volStepXY, int volDimX, int volDimY,
                    int volDimZ, CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales,
                    CudaHostMemoryHeap<float, 2>& tcDepthMap_hmh, bool verbose, int distLimit);

void ps_filterRcIdDepthMapByTcDepthMaps(
                    CudaHostMemoryHeap<unsigned short, 2>* nModalsMap_hmh,
                    CudaHostMemoryHeap<unsigned short, 2>* rcIdDepthMap_hmh, cameraStruct** cams,
                    int ncams, int width, int height, int volStepXY, int volDimX, int volDimY,
                    int volDimZ, CudaHostMemoryHeap<float, 2>& depths_hmh, int nDepths, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales,
                    CudaHostMemoryHeap<float, 2>** tcDepthMaps_hmh, bool verbose, int distLimit);

#if 0
void ps_planeSweepingGPUPixelsFine(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* odpt_hmh,
                    CudaHostMemoryHeap<float, 2>* osim_hmh, cameraStruct** cams, int ncams, int width,
                    int height, CudaHostMemoryHeap<int2, 2>& pixs_hmh,
                    CudaHostMemoryHeap<float, 2>& depths_hmh,
                    CudaHostMemoryHeap<float4, 2>& normals_hmh, int slicesAtTime, int ntimes,
                    int npixs, int wsh, int kernelSizeHalf, int nPlanes, int scale, int CUDAdeviceNo,
                    int ncamsAllocated, int scales, bool verbose, float gammaC, float gammaP,
                    float epipShift = 0.0f);
#endif

void ps_planeSweepNPlanes(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* osim_hmh,
                    CudaHostMemoryHeap<float, 2>* odpt_hmh,
                    float* depths, int ndepths, cameraStruct** cams,
                    int ncams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                    bool verbose, int step);

void ps_planeSweepAggr(
                    CudaHostMemoryHeap<uchar4, 2>& rimg_hmh, CudaHostMemoryHeap<uchar4, 2>& timg_hmh,
                    CudaHostMemoryHeap<float, 2>* osim_hmh, CudaHostMemoryHeap<float, 2>* odepth_hmh,
                    float* depths, int ndepths, cameraStruct** rtcams, int width, int height, int scale, int scales,
                    bool verbose);

void ps_getTexture(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<uchar4, 2>* oimg_hmh, int camId,
                    int scale, int CUDAdeviceNo, int ncamsAllocated, int scales);

void ps_smoothDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                    cameraStruct** cams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                    int scales, int wsh, bool verbose, float gammaC, float gammaP);

void ps_filterDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                    cameraStruct** cams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                    int scales, int wsh, bool verbose, float gammaC, float minCostThr);

void ps_computeNormalMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float3, 2>* normalMap_hmh,
                    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                    cameraStruct** cams, int width, int height,
                    int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int wsh, bool verbose,
                    float gammaC, float gammaP);
;

void ps_alignSourceDepthMapToTarget(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* sourceDepthMap_hmh,
                    CudaHostMemoryHeap<float, 2>* targetDepthMap_hmh, cameraStruct** cams, int width,
                    int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int wsh,
                    bool verbose, float gammaC, float maxPixelSizeDist);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ps_dilateDepthMap(
                    CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
                    CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp, int width, int height, bool verbose,
                    int niters, float gammaC);

void ps_dilateMaskMap(
                    CudaDeviceMemoryPitched<float, 2>& depthMap_dmp,
                    int width, int height, bool verbose,
                    int niters, float fpPlaneDepth);

void ps_refineDepthMapInternal(
                    CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
                    CudaDeviceMemoryPitched<float, 2>& odepthMap_dmp,
                    CudaDeviceMemoryPitched<float, 2>& idepthMap_dmp,
                    CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp, int width, int height,
                    bool verbose, int wsh, float gammaC, float gammaP, float simThr,
                    CudaDeviceMemoryPitched<float3, 2>& dsm_dmp,
                    CudaDeviceMemoryPitched<float3, 2>& ssm_dmp, CudaArray<uchar4, 2>& tTexU4_arr,
                    CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp, bool moveByTcOrRc, float step);

void ps_computeSimMapForDepthMapInternal(
                    cudaTextureObject_t t4tex,
                    CudaDeviceMemoryPitched<float, 2>& osimMap_dmp,
                    CudaDeviceMemoryPitched<float, 2>& idepthMapMask_dmp, int width, int height,
                    bool verbose, int wsh, float gammaC, float gammaP,
                    CudaArray<uchar4, 2>& tTexU4_arr,
                    CudaDeviceMemoryPitched<uchar4, 2>& timg_dmp, float fpPlaneDepth);

void ps_growDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<uchar4, 2>* otimg_hmh,
                    CudaHostMemoryHeap<float, 2>* osim_hmh, CudaHostMemoryHeap<float, 2>* odpt_hmh,
                    CudaHostMemoryHeap<float, 2>& depthMap_hmh, cameraStruct** cams, int ncams, float* depths,
                    int ndepths, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                    bool verbose, int wsh, float gammaC, float gammaP, float simThr);

void ps_refineDepthMapReproject(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<uchar4, 2>* otimg_hmh,
                    CudaHostMemoryHeap<float, 2>* osim_hmh,
                    CudaHostMemoryHeap<float, 2>* odpt_hmh,
                    CudaHostMemoryHeap<float, 2>& depthMap_hmh,
                    cameraStruct** cams, int ncams, int width,
                    int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                    int wsh, float gammaC, float gammaP, float simThr, int niters, bool moveByTcOrRc);

void ps_computeRcTcPhotoErrMapReproject(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float4, 2>* osdpi_hmh,
                    CudaHostMemoryHeap<float, 2>* oerr_hmh, CudaHostMemoryHeap<float, 2>* oderr_hmh,
                    CudaHostMemoryHeap<float, 2>& rcDepthMap_hmh, CudaHostMemoryHeap<float, 2>& tcDepthMap_hmh, cameraStruct** cams,
                    int ncams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose,
                    int wsh, float gammaC, float gammaP, float depthMapShift);

void ps_computeSimMapsForNShiftsOfRcTcDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float2, 2>** odepthSimMaps_hmh, int ntcsteps,
                    CudaHostMemoryHeap<float, 2>& rcDepthMap_hmh, cameraStruct** cams,
                    int ncams, int width, int height, int scale, int CUDAdeviceNo,
                    int ncamsAllocated, int scales, bool verbose, int wsh, float gammaC,
                    float gammaP, float epipShift);

void ps_computeSimMapForRcTcDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* osimMap_hmh,
                    CudaHostMemoryHeap<float, 2>& rcTcDepthMap_hmh, cameraStruct** cams, int ncams,
                    int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales,
                    bool verbose, int wsh, float gammaC, float gammaP, float epipShift);

void ps_refineRcDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    float* osimMap_hmh,
                    float* rcDepthMap_hmh, int ntcsteps,
                    cameraStruct** cams, int ncams, int width,
                    int height, int imWidth, int imHeight, int scale, int CUDAdeviceNo, int ncamsAllocated,
                    int scales, bool verbose, int wsh, float gammaC, float gammaP, float epipShift,
                    bool moveByTcOrRc, int xFrom);

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
void ps_fuseDepthSimMapsGaussianKernelVoting(
                    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                    CudaHostMemoryHeap<float2, 2>** depthSimMaps_hmh, int ndepthSimMaps,
                    int nSamplesHalf, int nDepthsToRefine, float sigma, int width, int height,
                    bool verbose);

void ps_optimizeDepthSimMapGradientDescent(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float2, 2>* odepthSimMap_hmh,
                    CudaHostMemoryHeap<float2, 2>** dataMaps_hmh, int ndataMaps,
                    int nSamplesHalf, int nDepthsToRefine, int nIters, float sigma,
                    cameraStruct** cams, int ncams, int width, int height, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales, bool verbose, int yFrom);

void ps_GC_aggregatePathVolume(
                    CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
                    CudaHostMemoryHeap<unsigned int, 3>& ivol_hmh,
                    int volDimX, int volDimY, int volDimZ);

void ps_GC_K_aggregatePathVolume(
                    CudaHostMemoryHeap<unsigned int, 2>* ftid_hmh, // f-irst t-label id
                    CudaHostMemoryHeap<unsigned int, 3>& ivol_hmh,
                    int volDimX, int volDimY, int volDimZ,
                    int K);

void ps_ptsStatForRcDepthMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* depthMap_hmh,
                    cameraStruct** cams, CudaHostMemoryHeap<float3, 2>& pts_hmh,
                    CudaHostMemoryHeap<float2, 2>& out_hmh, int npts, int width, int height, int scale,
                    int CUDAdeviceNo, int ncamsAllocated, int scales, int maxNPixSize, int wsh, float gammaC,
                    float gammaP, bool verbose);

void ps_computeSimMapReprojectByDepthMapMovedByStep(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<float, 2>* osimMap_hmh,
                    CudaHostMemoryHeap<float, 2>* iodepthMap_hmh, cameraStruct** cams,
                    int ncams, int width, int height, int scale, int CUDAdeviceNo,
                    int ncamsAllocated, int scales, bool verbose, int wsh, float gammaC,
                    float gammaP, bool moveByTcOrRc, float step);

void ps_reprojectRGBTcImageByDepthMap(
                    CudaHostMemoryHeap<uchar4, 2>* iTcoRcRgbImage_hmh,
                    CudaHostMemoryHeap<float, 2>* rcDepthMap_hmh, cameraStruct** cams, int ncams,
                    int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                    int scales, bool verbose);

void ps_computeRcTcDepthMap(
                    CudaHostMemoryHeap<float, 2>& iRcDepthMap_oRcTcDepthMap_hmh,
                    CudaHostMemoryHeap<float, 2>& tcDepthMap_hmh, float pixSizeRatioThr, cameraStruct** cams,
                    int ncams, int width, int height, int scale, int CUDAdeviceNo, int ncamsAllocated,
                    int scales, bool verbose);

void ps_getSilhoueteMap(
                    // CudaArray<uchar4, 2>** ps_texs_arr,
                    CudaHostMemoryHeap<bool, 2>* omap_hmh, int width,
                    int height, int scale, int CUDAdeviceNo, int ncamsAllocated, int scales, int step, int camId,
                    uchar4 maskColorRgb, bool verbose);

void ps_retexture(
                    CudaHostMemoryHeap<uchar4, 2>* bmpOrig_hmh,
                    CudaHostMemoryHeap<uchar4, 2>* bmpObj_hmh,
                    CudaHostMemoryHeap<float4, 2>* retexturePixs_hmh,
                    int wObj, int hObj, int wOrig, int hOrig,
                    int slicesAtTime, int ntimes, int npixs, int CUDAdeviceNo, bool verbose);

void ps_retextureComputeNormalMap(
                    CudaHostMemoryHeap<uchar4, 2>* bmpObj_hmh,
                    CudaHostMemoryHeap<float2, 2>* retexturePixs_hmh,
                    CudaHostMemoryHeap<float3, 2>* retextureNorms_hmh, int wObj, int hObj,
                    int slicesAtTime, int ntimes, int npixs, int CUDAdeviceNo, bool verbose);

void ps_colorExtractionPushPull(
                    CudaHostMemoryHeap<uchar4, 2>* bmp_hmh,
                    int w, int h, int CUDAdeviceNo, bool verbose);

} // namespace depthMap
} // namespace aliceVision
