// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "caching.hpp"

#include <aliceVision/system/Logger.hpp>


namespace aliceVision {
namespace image {

CacheValue::CacheValue()
{
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<unsigned char>> img)
{
    CacheValue value;
    value.imgUChar = img;
    return value;
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<float>> img)
{
    CacheValue value;
    value.imgFloat = img;
    return value;
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<RGBColor>> img)
{
    CacheValue value;
    value.imgRGB = img;
    return value;
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<RGBfColor>> img)
{
    CacheValue value;
    value.imgRGBf = img;
    return value;
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<RGBAColor>> img)
{
    CacheValue value;
    value.imgRGBA = img;
    return value;
}

CacheValue CacheValue::wrap(std::shared_ptr<Image<RGBAfColor>> img)
{
    CacheValue value;
    value.imgRGBAf = img;
    return value;
}

int CacheValue::useCount() const
{
    if (imgUChar)
    {
        return imgUChar.use_count();
    }
    if (imgFloat)
    {
        return imgFloat.use_count();
    }
    if (imgRGB)
    {
        return imgRGB.use_count();
    }
    if (imgRGBf)
    {
        return imgRGBf.use_count();
    }
    if (imgRGBA)
    {
        return imgRGBA.use_count();
    }
    if (imgRGBAf)
    {
        return imgRGBAf.use_count();
    }
    return 0;
}

int CacheValue::memorySize() const
{
    if (imgUChar)
    {
        return imgUChar->MemorySize();
    }
    if (imgFloat)
    {
        return imgFloat->MemorySize();
    }
    if (imgRGB)
    {
        return imgRGB->MemorySize();
    }
    if (imgRGBf)
    {
        return imgRGBf->MemorySize();
    }
    if (imgRGBA)
    {
        return imgRGBA->MemorySize();
    }
    if (imgRGBAf)
    {
        return imgRGBAf->MemorySize();
    }
    return 0;
}

ImageCache::ImageCache(int capacity_MB, int maxSize_MB, const ImageReadOptions& options) : 
    _memUsage(capacity_MB, maxSize_MB), 
    _options(options)
{
}

ImageCache::~ImageCache()
{
}

std::string ImageCache::toString() const
{
    const std::lock_guard<std::mutex> lock(_mutex);

    std::string description = "Image cache content (LRU to MRU): ";

    for (const CacheKey& key : _keys)
    {
        std::string keyDesc = key.filename + 
                              ", nbChannels: " + std::to_string(key.nbChannels) + 
                              ", typeDesc: " + std::to_string(key.typeDesc) + 
                              ", halfSampleLevel: " + std::to_string(key.halfSampleLevel) + 
                              ", usages: " + std::to_string(_imagePtrs.at(key).useCount()) + 
                              ", size: " + std::to_string(_imagePtrs.at(key).memorySize());
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
