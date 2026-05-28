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
namespace depthMap_sycl {

size_t hash_device(const sycl::device& device);

template<typename T>
class PerDevice
{
public:
    /**
     * @brief If the item associated with the device is already initialized, return it. Otherwise, construct it in-place
     * @param[in] device the device to identify sycl device/context pair
     * @param[in] args any arguments to pass to the constructor of the item (can be omitted)
     */
    template<typename... Args>
    inline T& getOrConstruct(const sycl::device& device, Args&&... args)
    {
        // get unique id
        const size_t id = hash_device(device);

        // find the current SingleDeviceCache
        _lock.lock_shared();
        auto it = _perDevice.find(id);
        _lock.unlock_shared();

        // check found
        if (it == _perDevice.end())
        {
            // modification of top level container must be protected by lock
            std::lock_guard lkg{_lock};
            it = _perDevice.find(id); // check if another thread has added it in the meantime
            if(it != _perDevice.end())
            {
                return it->second;
            }
            it = _perDevice.emplace(std::piecewise_construct,
                                    std::forward_as_tuple(id),
                                    std::forward_as_tuple(args...)
                                    ).first;
        }

        // return current class reference
        return it->second;
    }

    /**
     * @brief Clear the item associated with the current device
     * @param[in] queue the queue to identify sycl device/context pair
     */
    inline void erase(const sycl::device& device)
    {
        std::lock_guard lkg{_lock};
        // get unique id
        const size_t id = hash_device(device);
        _perDevice.erase(id);
    }

private:
    std::unordered_map<std::size_t, T> _perDevice;  // <sycl::device hash, item>
    std::shared_mutex _lock; // to prevent concurrent modification of the map container

public:
    PerDevice() = default;
    ~PerDevice() = default;
};

/*
 * @struct SingleDeviceCache
 * @brief This class keeps the cache data for a single gpu device.
 */
struct SingleDeviceCache
{
    // ctors and dtors must be public, so that PerDevice can construct them in-place
    SingleDeviceCache() = default;
    ~SingleDeviceCache() = default;

private: // only allow global singleton to actually use a cache instance
    friend class DeviceCache;

    std::unordered_map<int, std::pair<DeviceMipmapImage, std::atomic<unsigned int>>> mipmaps_ctd;  //< <camId, <DeviceMipmapImage, nbUsers>> cached device mipmap images
    std::mutex _lock; // to prevent concurrent modification of the map container
};

/**
 * @class Device cache
 * @brief This singleton allows to access the current device cache.
 */
class DeviceCache : private PerDevice<SingleDeviceCache>
{
  public:
    // Singleton, no copy constructor
    DeviceCache(DeviceCache const&) = delete;

    // Singleton, no copy operator
    void operator=(DeviceCache const&) = delete;

    /**
     * @brief Clear the current device cache.
     * @note Does not synchronize with other threads!
     * @param[in] device the device to identify sycl device/context pair
     */
    inline void clear(const sycl::device& device) { erase(device); };

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
     * @param[in] device the device to identify sycl device/context pair
     * @return DeviceMipmapImage
     */
    const DeviceMipmapImage& requestMipmapImage(int camId, const mvsUtils::MultiViewParams& mp, const sycl::device& device);

    /**
     * @brief Reduce user count of a MipmapImage, and free it's memory if it has no users
     * @param[in] camId the camera index in the ImagesCache / MultiViewParams
     * @param[in] device the device to identify sycl device/context pair
     * @return DeviceMipmapImage
     */
    void freeMipmapImage(int camId, const sycl::device& device);

    // default constructor and destructor
    DeviceCache() = default;
    ~DeviceCache() = default;
};

}  // namespace depthMap_sycl
}  // namespace aliceVision
