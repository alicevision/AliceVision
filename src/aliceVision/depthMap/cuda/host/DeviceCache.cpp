// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceCache.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/depthMap/cuda/host/utils.hpp>
#include <aliceVision/depthMap/cuda/device/DeviceCameraParams.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>

// maximum pre-computed Gaussian scales
#define DEVICE_MAX_DOWNSCALE (MAX_CONSTANT_GAUSS_SCALES - 1)

namespace aliceVision {
namespace depthMap {

float3 M3x3mulV3(const float* M3x3, const float3& V)
{
    return make_float3(
      M3x3[0] * V.x + M3x3[3] * V.y + M3x3[6] * V.z, M3x3[1] * V.x + M3x3[4] * V.y + M3x3[7] * V.z, M3x3[2] * V.x + M3x3[5] * V.y + M3x3[8] * V.z);
}

void normalize(float3& a)
{
    float d = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    a.x /= d;
    a.y /= d;
    a.z /= d;
}

/**
 * @brief Fill the host-side camera parameters from multi-view parameters.
 * @param[in,out] cameraParameters_h the host-side camera parameters
 * @param[in] camId the camera index in the ImagesCache / MultiViewParams
 * @param[in] downscale the downscale to apply on parameters
 * @param[in] mp the multi-view parameters
 */
void fillHostCameraParameters(DeviceCameraParams& cameraParameters_h, int camId, int downscale, const mvsUtils::MultiViewParams& mp)
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

    Matrix3x3 K = scaleM * mp.KArr[camId];
    Matrix3x3 iK = K.inverse();
    Matrix3x4 P = K * (mp.RArr[camId] | (Point3d(0.0, 0.0, 0.0) - mp.RArr[camId] * mp.CArr[camId]));
    Matrix3x3 iP = mp.iRArr[camId] * iK;

    cameraParameters_h.C.x = mp.CArr[camId].x;
    cameraParameters_h.C.y = mp.CArr[camId].y;
    cameraParameters_h.C.z = mp.CArr[camId].z;

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

    cameraParameters_h.R[0] = mp.RArr[camId].m11;
    cameraParameters_h.R[1] = mp.RArr[camId].m21;
    cameraParameters_h.R[2] = mp.RArr[camId].m31;
    cameraParameters_h.R[3] = mp.RArr[camId].m12;
    cameraParameters_h.R[4] = mp.RArr[camId].m22;
    cameraParameters_h.R[5] = mp.RArr[camId].m32;
    cameraParameters_h.R[6] = mp.RArr[camId].m13;
    cameraParameters_h.R[7] = mp.RArr[camId].m23;
    cameraParameters_h.R[8] = mp.RArr[camId].m33;

    cameraParameters_h.iR[0] = mp.iRArr[camId].m11;
    cameraParameters_h.iR[1] = mp.iRArr[camId].m21;
    cameraParameters_h.iR[2] = mp.iRArr[camId].m31;
    cameraParameters_h.iR[3] = mp.iRArr[camId].m12;
    cameraParameters_h.iR[4] = mp.iRArr[camId].m22;
    cameraParameters_h.iR[5] = mp.iRArr[camId].m32;
    cameraParameters_h.iR[6] = mp.iRArr[camId].m13;
    cameraParameters_h.iR[7] = mp.iRArr[camId].m23;
    cameraParameters_h.iR[8] = mp.iRArr[camId].m33;

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

    cameraParameters_h.XVect = M3x3mulV3(cameraParameters_h.iR, make_float3(1.f, 0.f, 0.f));
    normalize(cameraParameters_h.XVect);

    cameraParameters_h.YVect = M3x3mulV3(cameraParameters_h.iR, make_float3(0.f, 1.f, 0.f));
    normalize(cameraParameters_h.YVect);

    cameraParameters_h.ZVect = M3x3mulV3(cameraParameters_h.iR, make_float3(0.f, 0.f, 1.f));
    normalize(cameraParameters_h.ZVect);
}

/**
 * @brief Fill the device-side camera parameters array (constant memory)
 *        with the given host-side camera parameters.
 * @param[in] cameraParameters_h the host-side camera parameters
 * @param[in] deviceCameraParamsId the constant camera parameters array
 */
void fillDeviceCameraParameters(const DeviceCameraParams& cameraParameters_h, int deviceCameraParamsId)
{
    const cudaMemcpyKind kind = cudaMemcpyHostToDevice;
    const cudaError_t err = cudaMemcpyToSymbol(
      constantCameraParametersArray_d, &cameraParameters_h, sizeof(DeviceCameraParams), deviceCameraParamsId * sizeof(DeviceCameraParams), kind);
    CHECK_CUDA_RETURN_ERROR(err);
    THROW_ON_CUDA_ERROR(err, "Failed to copy camera parameters from host to device.");
}

DeviceCache::SingleDeviceCache::SingleDeviceCache(int maxMipmapImages, int maxCameraParams)
  : mipmapCache(maxMipmapImages),
    cameraParamCache(maxCameraParams)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    ALICEVISION_LOG_TRACE("Initialize device cache (device id: " << cudaDeviceId << "):" << std::endl
                                                                 << "\t - # mipmap images: " << maxMipmapImages << std::endl
                                                                 << "\t - # cameras parameters: " << maxCameraParams);

    // initialize Gaussian filters in GPU constant memory
    // force at compilation to build with maximum pre-computed Gaussian scales
    // note: useful for downscale with gaussian blur, volume gaussian blur (Z, XYZ)
    cuda_createConstantGaussianArray(cudaDeviceId, DEVICE_MAX_DOWNSCALE);

    // the maximum number of camera parameters in device cache cannot be superior
    // to the number of camera parameters in the array in device constant memory
    if (maxCameraParams > ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS)
        ALICEVISION_THROW_ERROR("Cannot initialize device cache with more than " << ALICEVISION_DEVICE_MAX_CONSTANT_CAMERA_PARAM_SETS
                                                                                 << " camera parameters (device id: " << cudaDeviceId
                                                                                 << ", # cameras parameters: " << maxCameraParams << ").")

    // initialize cached mipmap image containers
    mipmaps.reserve(maxMipmapImages);
    for (int i = 0; i < maxMipmapImages; ++i)
    {
        mipmaps.push_back(std::make_unique<DeviceMipmapImage>());
    }
}

void DeviceCache::clear()
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // find the current SingleDeviceCache
    auto it = _cachePerDevice.find(cudaDeviceId);

    // if found, erase SingleDeviceCache data
    if (it != _cachePerDevice.end())
        _cachePerDevice.erase(it);
}

void DeviceCache::build(int maxMipmapImages, int maxCameraParams)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // reset the current device cache
    _cachePerDevice[cudaDeviceId].reset(new SingleDeviceCache(maxMipmapImages, maxCameraParams));
}

DeviceCache::SingleDeviceCache& DeviceCache::getCurrentDeviceCache()
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // find the current SingleDeviceCache
    auto it = _cachePerDevice.find(cudaDeviceId);

    // check found and initialized
    if (it == _cachePerDevice.end() || it->second == nullptr)
    {
        ALICEVISION_THROW_ERROR("Device cache is not initialized (cuda device id: " << cudaDeviceId << ").")
    }

    // return current SingleDeviceCache reference
    return *(it->second);
}

void DeviceCache::addMipmapImage(int camId,
                                 int minDownscale,
                                 int maxDownscale,
                                 mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>& imageCache,
                                 const mvsUtils::MultiViewParams& mp)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache();

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    // find out with the LRU (Least Recently Used) strategy if the mipmap image is already in the cache
    // note: if new insertion in the cache, we need to replace a cached object with the new one
    int deviceMipmapId;
    const bool newInsertion = currentDeviceCache.mipmapCache.insert(camId, &deviceMipmapId);

    // check if the camera is already in cache
    if (!newInsertion)
    {
        ALICEVISION_LOG_TRACE("Add mipmap image on device cache: already on cache (id: " << camId << ", view id: " << viewId << ").");
        return;  // nothing to do
    }

    ALICEVISION_LOG_TRACE("Add mipmap image on device cache (id: " << camId << ", view id: " << viewId << ").");

    // get image buffer
    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>::ImgSharedPtr img = imageCache.getImg_sync(camId);

    // allocate the full size host-sided image buffer
    CudaSize<2> imgSize(img->width(), img->height());
    CudaHostMemoryHeap<CudaRGBA, 2> img_hmh(imgSize);

    // copy image from imageCache to CUDA host-side image buffer
#pragma omp parallel for
    for (int y = 0; y < imgSize.y(); ++y)
    {
        for (int x = 0; x < imgSize.x(); ++x)
        {
            const image::RGBAfColor& floatRGBA = (*img)(y, x);
            CudaRGBA& cudaRGBA = img_hmh(x, y);

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
            // explicit float to half conversion
            cudaRGBA.x = __float2half(floatRGBA.r() * 255.0f);
            cudaRGBA.y = __float2half(floatRGBA.g() * 255.0f);
            cudaRGBA.z = __float2half(floatRGBA.b() * 255.0f);
            cudaRGBA.w = __float2half(floatRGBA.a() * 255.0f);
#else
            cudaRGBA.x = floatRGBA.r() * 255.0f;
            cudaRGBA.y = floatRGBA.g() * 255.0f;
            cudaRGBA.z = floatRGBA.b() * 255.0f;
            cudaRGBA.w = floatRGBA.a() * 255.0f;
#endif
        }
    }

    DeviceMipmapImage& deviceMipmapImage = *(currentDeviceCache.mipmaps.at(deviceMipmapId));
    deviceMipmapImage.fill(img_hmh, minDownscale, maxDownscale);
}

void DeviceCache::addCameraParams(int camId, int downscale, const mvsUtils::MultiViewParams& mp)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache();

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    // find out with the LRU (Least Recently Used) strategy if the camera parameters is already in the cache
    // note: if new insertion in the cache, we need to replace a cached object with the new one
    int deviceCameraParamsId;
    const bool newInsertion = currentDeviceCache.cameraParamCache.insert(CameraPair(camId, downscale), &deviceCameraParamsId);

    // check if the camera is already in cache
    if (!newInsertion)
    {
        ALICEVISION_LOG_TRACE("Add camera parameters on device cache: already on cache (id: " << camId << ", view id: " << viewId
                                                                                              << ", downscale: " << downscale << ").");
        return;  // nothing to do
    }

    ALICEVISION_LOG_TRACE("Add camera parameters on device cache (id: " << camId << ", view id: " << viewId << ", downscale: " << downscale << ").");

    // build host-side device camera parameters struct
    DeviceCameraParams* cameraParameters_h = nullptr;

    const cudaError_t err = cudaMallocHost(&cameraParameters_h, sizeof(DeviceCameraParams));
    CHECK_CUDA_RETURN_ERROR(err);
    THROW_ON_CUDA_ERROR(err, "Failed to allocate camera parameters in pinned host memory.");

    // fill the host-side camera parameters from multi-view parameters.
    fillHostCameraParameters(*cameraParameters_h, camId, downscale, mp);

    // copy host-side device camera parameters struct to device-side camera parameters array
    // note: device-side camera parameters array is in constant memory
    fillDeviceCameraParameters(*cameraParameters_h, deviceCameraParamsId);

    // free host-side device camera parameters struct
    CHECK_CUDA_RETURN_ERROR(cudaFreeHost(cameraParameters_h));

    // check last error
    CHECK_CUDA_ERROR();
}

const DeviceMipmapImage& DeviceCache::requestMipmapImage(int camId, const mvsUtils::MultiViewParams& mp)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache();

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    ALICEVISION_LOG_TRACE("Request mipmap image on device cache (id: " << camId << ", view id: " << viewId << ").");

    // find out with the LRU (Least Recently Used) strategy if the mipmap image is already in the cache
    // note: if not found in cache we need to throw an error: in that case we insert an orphan id in the cache
    int deviceMipmapId;
    const bool notFound = currentDeviceCache.mipmapCache.insert(camId, &deviceMipmapId);

    // check if the mipmap image is in the cache
    if (notFound)
        ALICEVISION_THROW_ERROR("Request mipmap image on device cache: Not found (id: " << camId << ", view id: " << viewId << ").")

    // return the cached device mipmap image
    return *(currentDeviceCache.mipmaps.at(deviceMipmapId));
}

const int DeviceCache::requestCameraParamsId(int camId, int downscale, const mvsUtils::MultiViewParams& mp)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache();

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    ALICEVISION_LOG_TRACE("Request camera parameters on device cache (id: " << camId << ", view id: " << viewId << ", downscale: " << downscale
                                                                            << ").");

    // find out with the LRU (Least Recently Used) strategy if the camera parameters object is already in the cache
    // note: if not found in cache we need to throw an error: in that case we insert an orphan id in the cache
    int deviceCameraParamsId;
    const bool notFound = currentDeviceCache.cameraParamCache.insert(CameraPair(camId, downscale), &deviceCameraParamsId);

    // check if the camera is in the cache
    if (notFound)
        ALICEVISION_THROW_ERROR("Request camera parameters on device cache: Not found (id: " << camId << ", view id: " << viewId
                                                                                             << ", downscale: " << downscale << ").")

    // return the cached camera parameters id
    return deviceCameraParamsId;
}

}  // namespace depthMap
}  // namespace aliceVision
