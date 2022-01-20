// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceCache.hpp"

#include <aliceVision/system/Logger.hpp>

#include <aliceVision/depthMap/cuda/utils.hpp>
#include <aliceVision/depthMap/cuda/imageProcessing/deviceGaussianFilter.hpp>

#define DEVICE_MAX_DOWNSCALE  ( MAX_CONSTANT_GAUSS_SCALES - 1 ) // maximum pre-computed Gaussian scales

namespace aliceVision {
namespace depthMap {

DeviceCache::SingleDeviceCache::SingleDeviceCache()
    : cameraCache(maxNbCameras)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    ALICEVISION_LOG_TRACE("Initialize device cache (CUDA device id: " << cudaDeviceId  << ").");

    // initialize Gaussian filters in GPU constant memory
    cuda_createConstantGaussianArray(cudaDeviceId, DEVICE_MAX_DOWNSCALE); // force at compilation to build with maximum pre-computed Gaussian scales. 

    // initialize cached camera containers
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

const DeviceCamera& DeviceCache::requestCamera(int globalCamId, int downscale, 
                                               mvsUtils::ImagesCache<ImageRGBAf>& imageCache,
                                               const mvsUtils::MultiViewParams& mp, 
                                               cudaStream_t stream)
{
    // get the current device id
    const int cudaDeviceId = getCudaDeviceId();

    // get the current device cache
    SingleDeviceCache& currentDeviceCache = _cachePerDevice[cudaDeviceId];

    // find out with the LRU (Least Recently Used) strategy if the camera is already in the cache
    int deviceCamId;  
    const CameraSelection newCameraSelection(globalCamId, downscale);
    const bool isNewInsertion = currentDeviceCache.cameraCache.insert(newCameraSelection, &deviceCamId);
    DeviceCamera& deviceCamera = *(currentDeviceCache.cameras.at(deviceCamId));

    // get corresponding view id for logs
    const IndexT viewId = mp.getViewId(globalCamId);

    // update the cached cached camera container
    if(isNewInsertion)
    {
        ALICEVISION_LOG_TRACE("Request camera on device cache: Add camera (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").");
        deviceCamera.fill(globalCamId, downscale, imageCache, mp, stream);
    }
    else
    {
        ALICEVISION_LOG_TRACE("Request camera on device cache: Camera already on cache (id: " << globalCamId << ", view id: " << viewId << ", downscale: " << downscale << ").");
    }

    return deviceCamera;
}

} // namespace depthMap
} // namespace aliceVision
