// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceCache.hpp"

#include <aliceVision/system/Logger.hpp>

#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>

#define DEVICE_MAX_DOWNSCALE  ( MAX_CONSTANT_GAUSS_SCALES - 1 ) // maximum pre-computed Gaussian scales

namespace aliceVision {
namespace depthMap {

float3 M3x3mulV3(const float* M3x3, const float3& V)
{
    return make_float3(M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, 
                       M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z,
                       M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

void normalize(float3& a)
{
    float d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

void initCameraMatrix(DeviceCameraParams& cameraParameters_h)
{
    float3 z;
    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
    cameraParameters_h.ZVect = M3x3mulV3(cameraParameters_h.iR, z);
    normalize(cameraParameters_h.ZVect);

    float3 y;
    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;
    cameraParameters_h.YVect = M3x3mulV3(cameraParameters_h.iR, y);
    normalize(cameraParameters_h.YVect);

    float3 x;
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;
    cameraParameters_h.XVect = M3x3mulV3(cameraParameters_h.iR, x);
    normalize(cameraParameters_h.XVect);
}

void fillHostCameraParameters(DeviceCameraParams& cameraParameters_h, int globalCamId, int downscale, const mvsUtils::MultiViewParams& mp)
{

    Matrix3x3 scaleM;
    scaleM.m11 = 1.0 / float(downscale);
    scaleM.m12 = 0.0;
    scaleM.m13 = 0.0;
    scaleM.m21 = 0.0;
    scaleM.m22 = 1.0 / float(downscale);
    scaleM.m23 = 0.0;
    scaleM.m31 = 0.0;
    scaleM.m32 = 0.0;
    scaleM.m33 = 1.0;

    Matrix3x3 K = scaleM * mp.KArr[globalCamId];
    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp.RArr[globalCamId] | (Point3d(0.0, 0.0, 0.0) - mp.RArr[globalCamId] * mp.CArr[globalCamId]));
    Matrix3x3 iP = mp.iRArr[globalCamId] * iK;

    cameraParameters_h.C.x = mp.CArr[globalCamId].x;
    cameraParameters_h.C.y = mp.CArr[globalCamId].y;
    cameraParameters_h.C.z = mp.CArr[globalCamId].z;

    cameraParameters_h.P[0] = P.m11;
    cameraParameters_h.P[1] = P.m21;
    cameraParameters_h.P[2] = P.m31;
    cameraParameters_h.P[3] = P.m12;
    cameraParameters_h.P[4] = P.m22;
    cameraParameters_h.P[5] = P.m32;
    cameraParameters_h.P[6] = P.m13;
    cameraParameters_h.P[7] = P.m23;
    cameraParameters_h.P[8] = P.m33;
    cameraParameters_h.P[9] = P.m14;
    cameraParameters_h.P[10] = P.m24;
    cameraParameters_h.P[11] = P.m34;

    cameraParameters_h.iP[0] = iP.m11;
    cameraParameters_h.iP[1] = iP.m21;
    cameraParameters_h.iP[2] = iP.m31;
    cameraParameters_h.iP[3] = iP.m12;
    cameraParameters_h.iP[4] = iP.m22;
    cameraParameters_h.iP[5] = iP.m32;
    cameraParameters_h.iP[6] = iP.m13;
    cameraParameters_h.iP[7] = iP.m23;
    cameraParameters_h.iP[8] = iP.m33;

    cameraParameters_h.R[0] = mp.RArr[globalCamId].m11;
    cameraParameters_h.R[1] = mp.RArr[globalCamId].m21;
    cameraParameters_h.R[2] = mp.RArr[globalCamId].m31;
    cameraParameters_h.R[3] = mp.RArr[globalCamId].m12;
    cameraParameters_h.R[4] = mp.RArr[globalCamId].m22;
    cameraParameters_h.R[5] = mp.RArr[globalCamId].m32;
    cameraParameters_h.R[6] = mp.RArr[globalCamId].m13;
    cameraParameters_h.R[7] = mp.RArr[globalCamId].m23;
    cameraParameters_h.R[8] = mp.RArr[globalCamId].m33;

    cameraParameters_h.iR[0] = mp.iRArr[globalCamId].m11;
    cameraParameters_h.iR[1] = mp.iRArr[globalCamId].m21;
    cameraParameters_h.iR[2] = mp.iRArr[globalCamId].m31;
    cameraParameters_h.iR[3] = mp.iRArr[globalCamId].m12;
    cameraParameters_h.iR[4] = mp.iRArr[globalCamId].m22;
    cameraParameters_h.iR[5] = mp.iRArr[globalCamId].m32;
    cameraParameters_h.iR[6] = mp.iRArr[globalCamId].m13;
    cameraParameters_h.iR[7] = mp.iRArr[globalCamId].m23;
    cameraParameters_h.iR[8] = mp.iRArr[globalCamId].m33;

    cameraParameters_h.K[0] = K.m11;
    cameraParameters_h.K[1] = K.m21;
    cameraParameters_h.K[2] = K.m31;
    cameraParameters_h.K[3] = K.m12;
    cameraParameters_h.K[4] = K.m22;
    cameraParameters_h.K[5] = K.m32;
    cameraParameters_h.K[6] = K.m13;
    cameraParameters_h.K[7] = K.m23;
    cameraParameters_h.K[8] = K.m33;

    cameraParameters_h.iK[0] = iK.m11;
    cameraParameters_h.iK[1] = iK.m21;
    cameraParameters_h.iK[2] = iK.m31;
    cameraParameters_h.iK[3] = iK.m12;
    cameraParameters_h.iK[4] = iK.m22;
    cameraParameters_h.iK[5] = iK.m32;
    cameraParameters_h.iK[6] = iK.m13;
    cameraParameters_h.iK[7] = iK.m23;
    cameraParameters_h.iK[8] = iK.m33;

    initCameraMatrix(cameraParameters_h);
}

DeviceCache::SingleDeviceCache::SingleDeviceCache(int maxNbCameras)
    : cameraCache(maxNbCameras)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    ALICEVISION_LOG_TRACE("Initialize device cache (device id: " << cudaDeviceId << ", cameras: " << maxNbCameras << ").");

    // initialize Gaussian filters in GPU constant memory
    cuda_createConstantGaussianArray(cudaDeviceId, DEVICE_MAX_DOWNSCALE); // force at compilation to build with maximum pre-computed Gaussian scales. 

    if(maxNbCameras > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
        ALICEVISION_THROW_ERROR("Cannot initialize device cache with more than " << ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS << " cameras (device id: " << cudaDeviceId << ", cameras: " << maxNbCameras << ").")

    // initialize cached camera containers
    cameras.reserve(maxNbCameras);
    for(int i = 0; i < maxNbCameras; ++i)
    {
        cameras.push_back(std::make_unique<DeviceCamera>(i));
    }
}

void DeviceCache::clear()
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    auto it = _cachePerDevice.find(cudaDeviceId);

    // if found, erase SingleDeviceCache data
    if(it != _cachePerDevice.end())
        _cachePerDevice.erase(it);
}

void DeviceCache::buildCache(int maxNbCameras)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // reset the current device cache
    _cachePerDevice[cudaDeviceId].reset(new SingleDeviceCache(maxNbCameras));
}

void DeviceCache::addCamera(int globalCamId, int downscale, mvsUtils::ImagesCache<ImageRGBAf>& imageCache, const mvsUtils::MultiViewParams& mp)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // get the current device cache
    if(_cachePerDevice[cudaDeviceId] == nullptr)
        ALICEVISION_THROW_ERROR("Cannot add camera, device cache is not initialized (cuda device id: " << cudaDeviceId <<").")

    SingleDeviceCache& currentDeviceCache = *_cachePerDevice[cudaDeviceId];

    // find out with the LRU (Least Recently Used) strategy if the camera is already in the cache
    int deviceCamId;  
    const CameraSelection newCameraSelection(globalCamId, downscale);
    const bool isNewInsertion = currentDeviceCache.cameraCache.insert(newCameraSelection, &deviceCamId);
    DeviceCamera& deviceCamera = *(currentDeviceCache.cameras.at(deviceCamId));

    // get corresponding view id for logs
    const IndexT viewId = mp.getViewId(globalCamId);

    // check if the camera is already in cache
    if(!isNewInsertion)
    {
        // nothing to do
        ALICEVISION_LOG_TRACE("Add camera on device cache: Camera already on cache (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").");
        return;
    }

    // update the cached camera container
    if(deviceCamera.getGlobalCamId() < 0)
      ALICEVISION_LOG_TRACE("Add camera on device cache (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").");
    else
      ALICEVISION_LOG_TRACE("Add camera on device cache (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ")."
                            << "Replace camera (id: " << deviceCamera.getGlobalCamId() << ", view id: " << mp.getViewId(deviceCamera.getGlobalCamId()) << ", downscale: " << deviceCamera.getDownscale() << ")");

    mvsUtils::ImagesCache<ImageRGBAf>::ImgSharedPtr img = imageCache.getImg_sync(globalCamId);

    // allocate the frame full size host-sided data buffer
    CudaSize<2> originalFrameSize(img->width(), img->height());
    CudaHostMemoryHeap<CudaRGBA, 2> frame_hmh(originalFrameSize);

    // copy data for cached image "globalCamId" into an host-side data buffer
    #pragma omp parallel for
    for(int y = 0; y < originalFrameSize.y(); ++y)
    {
        for(int x = 0; x < originalFrameSize.x(); ++x)
        {
            const ColorRGBAf& floatRGBA = img->at(x, y);
            CudaRGBA& cudaRGBA = frame_hmh(x, y);
            cudaRGBA.x = floatRGBA.r * 255.0f;
            cudaRGBA.y = floatRGBA.g * 255.0f;
            cudaRGBA.z = floatRGBA.b * 255.0f;
            cudaRGBA.w = floatRGBA.a * 255.0f;
        }
    }

    // build host-side device camera parameters struct
    DeviceCameraParams cameraParameters_h;
    fillHostCameraParameters(cameraParameters_h, globalCamId, downscale, mp);

    // update device camera
    deviceCamera.fill(globalCamId, downscale, originalFrameSize.x(), originalFrameSize.y(), frame_hmh, cameraParameters_h);
}

const DeviceCamera& DeviceCache::requestCamera(int globalCamId, int downscale, const mvsUtils::MultiViewParams& mp)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // get the current device cache
    if(_cachePerDevice[cudaDeviceId] == nullptr)
        ALICEVISION_THROW_ERROR("Cannot add camera, device cache is not initialized (cuda device id: " << cudaDeviceId <<").")

    SingleDeviceCache& currentDeviceCache = *_cachePerDevice[cudaDeviceId];

    // find out with the LRU (Least Recently Used) strategy if the camera is already in the cache
    int deviceCamId;  
    const CameraSelection newCameraSelection(globalCamId, downscale);
    const bool isNewInsertion = currentDeviceCache.cameraCache.insert(newCameraSelection, &deviceCamId);
    const DeviceCamera& deviceCamera = *(currentDeviceCache.cameras.at(deviceCamId));

    // get corresponding view id for logs
    const IndexT viewId = mp.getViewId(globalCamId);

    // check if the camera is already in cache
    if(isNewInsertion)
    {
        ALICEVISION_THROW_ERROR("Request camera on device cache: Not found (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").")
    }

    ALICEVISION_LOG_TRACE("Request camera on device cache (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").");

    // return the cached device camera
    return deviceCamera;
}

} // namespace depthMap
} // namespace aliceVision
