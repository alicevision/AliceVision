// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "caching.hpp"

#include <aliceVision/system/Logger.hpp>


namespace aliceVision {
namespace image {

ImageCache::ImageCache(int capacity_MB, int maxSize_MB, const ImageReadOptions& options) : 
    _memUsage(capacity_MB, maxSize_MB), 
    _options(options)
{
}

ImageCache::~ImageCache()
{
}

std::shared_ptr<Image<RGBAfColor>> ImageCache::get(const std::string& filename, int halfSampleLevel)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    CacheKey reqKey(filename, halfSampleLevel);

    // try finding the requested image in the cached images
    {
        auto it = std::find(_keys.begin(), _keys.end(), reqKey);
        if (it != _keys.end())
        {
            // image becomes LRU
            _keys.erase(it);
            _keys.push_back(reqKey);

            return _imagePtrs[reqKey];
        }
    }

    // load image from disk and apply downscale
    auto img = std::make_shared<Image<RGBAfColor>>();
    readImage(filename, *img, _options);
    int downscale = 1 << halfSampleLevel;
    downscaleImageInplace(*img, downscale);

    // try adding image to cache if it fits in capacity
    if (img->MemorySize() + _memUsage.contentSize <= _memUsage.capacity) 
    {
        add(reqKey, img);
        return _imagePtrs[reqKey];
    }

    // image is too large to fit in capacity
    // retrieve missing space
    int missingSpace = img->MemorySize() + _memUsage.contentSize - _memUsage.capacity;

    // try removing an unused image that is sufficiently large
    {
        auto it = _keys.begin();
        while (it != _keys.end())
        {
            const CacheKey& key = *it;
            if (_imagePtrs[key].use_count() == 1 && _imagePtrs[key]->MemorySize() >= missingSpace)
            {
                // remove unused image from cache
                _memUsage.nbImages--;
                _memUsage.contentSize -= _imagePtrs[key]->MemorySize();
                _imagePtrs.erase(key);
                _keys.erase(it);

                add(reqKey, img);
                return _imagePtrs[reqKey];
            }
            ++it;
        }
    }

    // try adding image to cache it it fits in maxSize
    if (img->MemorySize() + _memUsage.contentSize <= _memUsage.maxSize) 
    {
        add(reqKey, img);
        return _imagePtrs[reqKey];
    }

    return nullptr;
}

void ImageCache::add(const CacheKey& key, std::shared_ptr<Image<RGBAfColor>> img)
{
    // store (key, img) entry
    _imagePtrs[key] = img;

    // image becomes LRU
    _keys.push_back(key);

    // update memory usage
    _memUsage.nbImages++;
    _memUsage.contentSize += img->MemorySize();
}

std::string ImageCache::toString() const
{
    const std::lock_guard<std::mutex> lock(_mutex);

    std::string description = "Image cache content (LRU to MRU): ";

    for (const CacheKey& key : _keys)
    {
        std::string keyDesc = key.filename + 
                              ", halfSampleLevel: " + std::to_string(key.halfSampleLevel) + 
                              ", usages: " + std::to_string(_imagePtrs.at(key).use_count()) + 
                              ", size: " + std::to_string(_imagePtrs.at(key)->MemorySize());
        description += "\n * " + keyDesc;
    }

    std::string memUsageDesc = "\nMemory usage: "
                               "\n * capacity: " + std::to_string(_memUsage.capacity) + 
                               "\n * max size: " + std::to_string(_memUsage.maxSize) + 
                               "\n * nb images: " + std::to_string(_memUsage.nbImages) + 
                               "\n * content size: " + std::to_string(_memUsage.contentSize);
    
    description += memUsageDesc;
 
    return description;
}

}
}
