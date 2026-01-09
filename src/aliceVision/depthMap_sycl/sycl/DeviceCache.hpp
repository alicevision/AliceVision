// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/depthMap_sycl/sycl/DeviceMipmapImage.hpp>

namespace aliceVision {
namespace depthMap {

/**
 * @class Device cache
 * @brief This singleton allows to access the current device cache.
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
     * @brief Clear the current device cache.
     * @param[in] queue the queue to identify sycl device/context pair
     */
    void clear(const sycl::queue& queue);

    /**
     * @brief Add a mipmap image in current device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] minDownscale the min downscale factor
     * @param[in] maxDownscale the max downscale factor
     * @param[in,out] imageCache the image cache to get host-side data
     * @param[in] mp the multi-view parameters
     * @param[in] queue the queue to identify sycl device/context pair
     */
    void addMipmapImage(int camId,
                        int minDownscale,
                        int maxDownscale,
                        mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>& imageCache,
                        const mvsUtils::MultiViewParams& mp,
                        bool& allocSuccess,
                        sycl::queue& queue);

    /**
     * @brief Request a mipmap image in current device cache.
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] mp the multi-view parameters
     * @param[in] queue the queue to identify sycl device/context pair
     * @return DeviceMipmapImage
     */
    const DeviceMipmapImage& requestMipmapImage(int camId, const mvsUtils::MultiViewParams& mp, const sycl::queue& queue);

    /**
     * @brief Reduce user count of a MipmapImage, and free it's memory if it has no users
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] queue the queue to identify sycl device/context pair
     * @return DeviceMipmapImage
     */
    void freeMipmapImage(int camId, sycl::queue& queue);

  private:
    // private members

    /*
     * @struct SingleDeviceCache
     * @brief This class keeps the cache data for a single gpu device.
     */
    struct SingleDeviceCache
    {
        std::unordered_map<int, std::pair<DeviceMipmapImage, unsigned int>> mipmaps_ctd;  //< <camId, <DeviceMipmapImage, nbUsers>> cached device mipmap images
    };
    std::unordered_map<std::size_t, SingleDeviceCache> _cachePerDevice;  // <sycl::queue hash, SingleDeviceCache>

    // Singleton, private default constructor
    DeviceCache() = default;

    // Singleton, private default destructor
    ~DeviceCache() = default;

    /**
     * @brief Get the SingleDeviceCache associated to the current queue
     * @return SingleDeviceCache
     */
    SingleDeviceCache& getCurrentDeviceCache(const sycl::queue& queue);
};

}  // namespace depthMap
}  // namespace aliceVision
