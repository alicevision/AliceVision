// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DeviceCache.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/depthMap_sycl/sycl/CameraParams.hpp>
#include <aliceVision/depthMap_sycl/sycl/matrix.hpp>

inline static size_t hash_queue(const sycl::queue& queue) {
    const sycl::device& device = queue.get_device();
    // get unique id
    size_t id;
    {
        const std::hash<std::string> strhsr{};
        const std::hash<uint> inthsr{};
        const sycl::platform& platform = device.get_platform();
        id = inthsr(device.get_info<sycl::info::device::vendor_id>()) +
            strhsr(platform.get_info<sycl::info::platform::name>()); // + has the advantage of avoiding collisions around zero, unlike xor. It's also fast, and commutativity is not a problem for our usecase
    }
    return id;
}

namespace aliceVision {
namespace depthMap {

DeviceCache::SingleDeviceCache& DeviceCache::getCurrentDeviceCache(const sycl::queue& queue)
{
    // get unique id
    const size_t id = hash_queue(queue);

    // Used for debug messages
    //const auto deviceName = queue.get_device().get_info<sycl::info::device::name>();
    //ALICEVISION_LOG_TRACE(deviceName << ": has hash " << id);

    // find the current SingleDeviceCache
    auto it = _cachePerDevice.find(id);

    // check found
    if (it == _cachePerDevice.end())
    {
        it = _cachePerDevice.emplace(std::piecewise_construct,
                                     std::forward_as_tuple(id),
                                     std::forward_as_tuple()
                                     ).first;
    }

    // return current SingleDeviceCache reference
    return it->second;
}

void DeviceCache::clear(const sycl::queue& queue)
{
    const size_t id = hash_queue(queue);
    _cachePerDevice.erase(id);
}

void DeviceCache::addMipmapImage(int camId,
                                 int minDownscale,
                                 int maxDownscale,
                                 mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>& imageCache,
                                 const mvsUtils::MultiViewParams& mp,
                                 bool& allocSuccess,
                                 sycl::queue& queue)
{
    if (not allocSuccess) return;
    // Used for debug messages
    const auto deviceName = queue.get_device().get_info<sycl::info::device::name>();

    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache(queue);

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    // check if the camera is already in cache
    auto mipmap_ctd = currentDeviceCache.mipmaps_ctd.find(camId);
    if (mipmap_ctd != currentDeviceCache.mipmaps_ctd.end())
    {
        ALICEVISION_LOG_TRACE(deviceName << ": Add mipmap image on device cache: already on cache (id: " << camId << ", view id: " << viewId << ").");
        mipmap_ctd->second.second++; // aument user counter
        return;
    }

    ALICEVISION_LOG_TRACE(deviceName << ": Add mipmap image on device cache (id: " << camId << ", view id: " << viewId << ").");

    // get image buffer
    mvsUtils::ImagesCache<image::Image<image::RGBAfColor>>::ImgSharedPtr img = imageCache.getImg_sync(camId);

    // allocate the full size device-sided image buffer
    SyclSize<2> imgSize(img->width(), img->height());
    SyclDeviceMemoryPitched<sycl::float4, 2> img_dmp(imgSize, allocSuccess, queue);
    if(not allocSuccess) return;

    // copy image from imageCache to SYCL device-side image buffer
    img_dmp.copyFrom(*img, sycl::event()).wait();

    ALICEVISION_LOG_TRACE(deviceName << ": Building mipmap image in device memory (id: " << camId << ", view id: " << viewId << ").");

    currentDeviceCache.mipmaps_ctd.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(camId),
        std::forward_as_tuple(
            std::piecewise_construct,
            std::forward_as_tuple(img_dmp, minDownscale, maxDownscale, allocSuccess, queue),
            std::forward_as_tuple(1)));

    if (not allocSuccess)
        currentDeviceCache.mipmaps_ctd.erase(camId); // Don't keep corrupted image around

    ALICEVISION_LOG_TRACE(deviceName << ": Built mipmap image in device memory (id: " << camId << ", view id: " << viewId << ").");
}

const DeviceMipmapImage& DeviceCache::requestMipmapImage(int camId, const mvsUtils::MultiViewParams& mp, const sycl::queue& queue)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache(queue);

    // get view id for logs
    const IndexT viewId = mp.getViewId(camId);

    ALICEVISION_LOG_TRACE("Request mipmap image on device cache (id: " << camId << ", view id: " << viewId << ").");

    // check if the mipmap image is in the cache
    auto mipmap_ctd = currentDeviceCache.mipmaps_ctd.find(camId);
    if (mipmap_ctd == currentDeviceCache.mipmaps_ctd.end())
        ALICEVISION_THROW_ERROR("Request mipmap image on device cache: Not found (id: " << camId << ", view id: " << viewId << ").")

    // return the cached device mipmap image
    return mipmap_ctd->second.first;
}

void DeviceCache::freeMipmapImage(int camId, sycl::queue& queue)
{
    // get current device cache
    SingleDeviceCache& currentDeviceCache = getCurrentDeviceCache(queue);

    // check if the mipmap image is in the cache
    auto mipmap_ctd = currentDeviceCache.mipmaps_ctd.find(camId);
    if (mipmap_ctd == currentDeviceCache.mipmaps_ctd.end())
    {
        ALICEVISION_LOG_INFO("WARNING: trying to remove nonexistent image from device cache. This shouldn't ever happen");
        return;
    }

    unsigned int userCount = --mipmap_ctd->second.second; // Decrement user counter
    if (userCount == 0) currentDeviceCache.mipmaps_ctd.erase(mipmap_ctd);
}


}  // namespace depthMap
}  // namespace aliceVision
