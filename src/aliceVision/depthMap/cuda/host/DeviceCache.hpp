// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <memory>

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>

#include <aliceVision/depthMap/cuda/host/DeviceCamera.hpp>
#include <aliceVision/depthMap/cuda/host/LRUCameraCache.hpp>

namespace aliceVision {
namespace depthMap {

/*
 * @class DeviceCache
 * @brief This singleton allows to access the current gpu cache.
 */
class DeviceCache
{
public:

    static DeviceCache& getInstance()
    {
        static DeviceCache instance;
        return instance;
    }

    // Singleton, no copy constructor
    DeviceCache(DeviceCache const&) = delete;

    // Singleton, no copy operator
    void operator=(DeviceCache const&) = delete;

    /**
     * @brief Clear the current gpu device cache.
     */
    void clear();

    /**
     * @brief Build the current device cache with the given maximum number of cameras.
     * @param[in] maxNbCameras the maximum number of cameras in the current device cache
     */
    void buildCache(int maxNbCameras);

    /**
     * @brief Add a camera (images + parameters) in current gpu device cache.
     * @param[in] globalCamId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     */
    void addCamera(int globalCamId, int downscale, mvsUtils::ImagesCache<ImageRGBAf>& imageCache, const mvsUtils::MultiViewParams& mp);

    /**
     * @brief Request a camera (images + parameters) in current gpu device cache.
     * @param[in] globalCamId the camera index in the ImagesCache / MultiViewParams
     * @param[in] downscale the downscale to apply on gpu
     * @param[in] mp the multi-view parameters
     * @return DeviceCamera (images + parameters)
     */
    const DeviceCamera& requestCamera(int globalCamId, int downscale, const mvsUtils::MultiViewParams& mp);

private:

    // Singleton, private default constructor
    DeviceCache() = default;

    // Singleton, private default destructor
    ~DeviceCache() = default;

    /*
     * @struct SingleDeviceCache
     * @brief This class keeps the cache data for a single gpu device.
     */
    struct SingleDeviceCache 
    {
        SingleDeviceCache(int maxNbCameras);
        ~SingleDeviceCache() = default;

        LRUCameraCache cameraCache; // Least Recently Used device camera id cache
        std::vector<std::unique_ptr<DeviceCamera>> cameras;
    };

    std::map <int, std::unique_ptr<SingleDeviceCache>> _cachePerDevice; // <cudaDeviceId, SingleDeviceCachePtr>
};

/**
  * @brief Fill the host-side camera parameters from multi-view parameters.
  * @param[in,out] cameraParameters_h the host-side camera parameters
  * @param[in] globalCamId the camera index in the ImagesCache / MultiViewParams
  * @param[in] downscale the downscale to apply on gpu
  * @param[in] mp the multi-view parameters
  */
void fillHostCameraParameters(DeviceCameraParams& cameraParameters_h, int globalCamId, int downscale, const mvsUtils::MultiViewParams& mp);

} // namespace depthMap
} // namespace aliceVision
