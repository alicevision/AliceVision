// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageCache.hpp"

#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace image {

std::ostream& operator<<(std::ostream& os, ELoadStatus status)
{
    switch (status)
    {
        case ELoadStatus::NONE:
            os << "NONE";
            break;
        case ELoadStatus::NOT_LOADED:
            os << "NOT_LOADED";
            break;
        case ELoadStatus::LOADING:
            os << "LOADING";
            break;
        case ELoadStatus::LOADED:
            os << "LOADED";
            break;
        case ELoadStatus::ERROR:
            os << "ERROR";
            break;
        default:
            os << "UNKNOWN";
            break;
    }
    return os;
}

int CacheKey::baseTypeSize() const
{
    return oiio::TypeDesc(typeDesc).basesize();
}

int CacheKey::pixelTypeSize() const
{
    return baseTypeSize() * nbChannels;
}

int CacheKey::imageMemorySize(const int width, const int height) const
{
    return width * height * pixelTypeSize();
}

CacheValue::CacheValue(std::shared_ptr<Image<unsigned char>> img)
: imgUChar(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
}

CacheValue::CacheValue(std::shared_ptr<Image<float>> img)
: imgFloat(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
}

CacheValue::CacheValue(std::shared_ptr<Image<RGBColor>> img)
: imgRGB(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
}

CacheValue::CacheValue(std::shared_ptr<Image<RGBfColor>> img)
: imgRGBf(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
}

CacheValue::CacheValue(std::shared_ptr<Image<RGBAColor>> img)
: imgRGBA(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
}

CacheValue::CacheValue(std::shared_ptr<Image<RGBAfColor>> img)
: imgRGBAf(img)
, _memorySize(img->memorySize())
, _loadStatus(ELoadStatus::LOADED)
{
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

unsigned long long int CacheValue::effectiveMemorySize() const
{
    if (imgUChar)
    {
        return imgUChar->memorySize();
    }
    if (imgFloat)
    {
        return imgFloat->memorySize();
    }
    if (imgRGB)
    {
        return imgRGB->memorySize();
    }
    if (imgRGBf)
    {
        return imgRGBf->memorySize();
    }
    if (imgRGBA)
    {
        return imgRGBA->memorySize();
    }
    if (imgRGBAf)
    {
        return imgRGBAf->memorySize();
    }
    return 0;
}

ImageCache::ImageCache(float capacity_MiB, float maxSize_MiB, const ImageReadOptions& options)
  : _info(capacity_MiB, maxSize_MiB),
    _options(options)
{}

ImageCache::~ImageCache() {}

std::string ImageCache::toString() const
{
    std::scoped_lock<std::mutex> lock(_mutex);
    return _toString();
}

std::string ImageCache::_toString() const
{
    std::string description = "Image cache content (LRU to MRU): ";

    for (const CacheKey& key : _keys)
    {
        std::string keyDesc = key.filename + ", nbChannels: " + std::to_string(key.nbChannels) + ", typeDesc: " + std::to_string(key.typeDesc) +
                              ", downscaleLevel: " + std::to_string(key.downscaleLevel) +
                              ", usages: " + std::to_string(_imagePtrs.at(key).useCount()) +
                              ", size: " + std::to_string(_imagePtrs.at(key).memorySize());
        description += "\n * " + keyDesc;
    }

    std::string memUsageDesc = "\nMemory usage: "
                               "\n * capacity: " +
                               std::to_string(_info.capacity) + "\n * max size: " + std::to_string(_info.maxSize) +
                               "\n * nb images: " + std::to_string(_info.nbImages) + "\n * content size: " + std::to_string(_info.contentSize);
    description += memUsageDesc;

    std::string statsDesc = "\nUsage statistics: "
                            "\n * nb load from disk: " +
                            std::to_string(_info.nbLoadFromDisk) + "\n * nb load from cache: " + std::to_string(_info.nbLoadFromCache) +
                            "\n * nb remove unused: " + std::to_string(_info.nbRemoveUnused);
    description += statsDesc;

    return description;
}

CacheValue ImageCache::get(const CacheKey& key)
{
    std::scoped_lock<std::mutex> lock(_mutex);
    CacheValue* v = _getPtr(key);
    if(v == nullptr)
    {
        return CacheValue();
    }
    return *v;
}

CacheValue* ImageCache::_getPtr(const CacheKey& key)
{
    if(_keys.empty())
        return nullptr;
    // find the requested image in the cached images
    auto it = std::find(_keys.begin(), _keys.end(), key);
    if (it != _keys.end())
    {
        // image becomes MRU
        _keys.erase(it);
        _keys.push_back(key);

        _info.nbLoadFromCache++;

        ALICEVISION_LOG_TRACE("[image] ImageCache: get " << key.filename << " with downscale level " << key.downscaleLevel << " from thread "
                                                        << std::this_thread::get_id());

        return &_imagePtrs.at(key);
    }
    return nullptr;
}

const CacheValue ImageCache::getReadOnly(const CacheKey& key) const
{
    std::scoped_lock<std::mutex> lock(_mutex);
    const CacheValue* v = _getPtrReadOnly(key);
    if(v == nullptr)
    {
        return CacheValue();
    }
    return *v;
}

const CacheValue* ImageCache::_getPtrReadOnly(const CacheKey& key) const
{
    if(_keys.empty())
        return nullptr;
    // find the requested image in the cached images
    auto it = std::find(_keys.begin(), _keys.end(), key);
    if (it != _keys.end())
    {
        return &_imagePtrs.at(key);
    }
    return nullptr;
}

CacheValue ImageCache::getAtMaxDownscale(const CacheKey& key)
{
    std::scoped_lock<std::mutex> lock(_mutex);
    
    for(CacheKey localKey = key; localKey.downscaleLevel > 0; --localKey.downscaleLevel)
    {
        if(CacheValue* c = _getPtr(localKey))
        {
            return *c;
        }
    }
    return CacheValue();
}

CacheValue ImageCache::getOrCreate(const CacheKey& key, int width, int height, bool lazyCleaning)
{
    std::scoped_lock<std::mutex> lock(_mutex);
    return _getOrCreate(key, width, height, lazyCleaning);
}

CacheValue ImageCache::_getOrCreate(const CacheKey& key, int width, int height, bool lazyCleaning)
{
    CacheValue* ptr = _getPtr(key);
    if(ptr != nullptr)
    {
        return *ptr;
    }

    // buffer size in bytes
    const unsigned long long int memSize = key.imageMemorySize((width / key.downscaleLevel), (height / key.downscaleLevel));

    // add image to cache if it fits in capacity
    if (memSize + _info.contentSize <= _info.capacity)
    {
        _createBuffer(key, memSize);

        ALICEVISION_LOG_TRACE("[image] ImageCache: " << _toString());
        return _imagePtrs.at(key);
    }

    // retrieve missing capacity
    long long int missingCapacity = memSize + _info.contentSize - _info.capacity;

    // find unused image with size bigger than missing capacity
    // remove it and add image to cache
    if (lazyCleaning)
    {
        auto it = _keys.begin();
        while (it != _keys.end())
        {
            const CacheKey& key = *it;
            const CacheValue& value = _imagePtrs.at(key);
            if (value.useCount() == 1 && value.memorySize() >= missingCapacity)
            {
                _info.nbImages--;
                _info.contentSize -= value.memorySize();
                _imagePtrs.erase(key);
                _keys.erase(it);

                _info.nbRemoveUnused++;

                _createBuffer(key, memSize);

                ALICEVISION_LOG_TRACE("[image] ImageCache: " << _toString());
                return _imagePtrs.at(key);
            }
            ++it;
        }
    }

    // remove as few unused images as possible
    while (missingCapacity > 0)
    {
        auto it = std::find_if(_keys.begin(), _keys.end(), [this](const CacheKey& k) { return _imagePtrs.at(k).useCount() == 1; });

        if (it != _keys.end())
        {
            const CacheKey& key = *it;
            const CacheValue& value = _imagePtrs.at(key);

            _info.nbImages--;
            _info.contentSize -= value.memorySize();
            _imagePtrs.erase(key);
            _keys.erase(it);

            _info.nbRemoveUnused++;

            missingCapacity = memSize + _info.contentSize - _info.capacity;
        }
        else
        {
            break;
        }
    }

    // add image to cache if it fits in maxSize
    if (memSize + _info.contentSize <= _info.maxSize)
    {
        _createBuffer(key, memSize);

        ALICEVISION_LOG_TRACE("[image] ImageCache: " << _toString());
        return _imagePtrs.at(key);
    }

    return CacheValue(); // not enough space
}

bool ImageCache::updateImage(const CacheKey& key, CacheValue newValue)
{
    std::scoped_lock<std::mutex> lock(_mutex);
    CacheValue* v = _getPtr(key);
    if(v == nullptr)
    {
        return false;
    }

    // Set the new image buffer
    *v = newValue;

    if(newValue.status() == ELoadStatus::LOADED)
    {
        _info.nbLoadFromDisk++;
    }
    else if(newValue.status() == ELoadStatus::ERROR)
    {
        _info.contentSize -= newValue.memorySize();
    }
    else if(newValue.status() == ELoadStatus::LOADING)
    {
        ALICEVISION_THROW_ERROR("ImageCache: updateImage with status LOADING: " << key.filename);
    }
    else if(newValue.status() == ELoadStatus::NOT_LOADED)
    {
        ALICEVISION_THROW_ERROR("ImageCache: updateImage with status NOT_LOADED: " << key.filename);
    }
    else if(newValue.status() == ELoadStatus::NONE)
    {
        ALICEVISION_THROW_ERROR("ImageCache: updateImage with status NONE: " << key.filename);
    }
    else
    {
        ALICEVISION_THROW_ERROR("ImageCache: updateImage with unknown status: " << newValue.status());
    }

    return true;
}

bool ImageCache::contains(const CacheKey& key) const
{
    std::scoped_lock<std::mutex> lock(_mutex);
    const CacheValue* v = _getPtrReadOnly(key);
    return v != nullptr;
}

bool ImageCache::containsValidImage(const CacheKey& key) const
{
    std::scoped_lock<std::mutex> lock(_mutex);
    return _containsValidImage(key);
}
bool ImageCache::_containsValidImage(const CacheKey& key) const
{
    const CacheValue* v = _getPtrReadOnly(key);
    if(v == nullptr)
    {
        return false;
    }
    return v->useCount() > 0;
}


void ImageCache::_createBuffer(const CacheKey& key, unsigned long long int memorySize)
{
    // add to cache as MRU
    _imagePtrs.insert({key, CacheValue(memorySize)});
    _keys.push_back(key);

    // update memory usage
    _info.nbImages++;
    _info.contentSize += memorySize;
}

template<typename TPix>
CacheValue imageProviderSync(ImageCache& imageCache, const std::string& filename, int downscaleLevel)
{
    using namespace aliceVision;
    using namespace aliceVision::image;

    CacheKey key(CacheKeyInit<TPix>(filename, downscaleLevel));
    CacheValue value = imageCache.get(key);
    if((value.status() == ELoadStatus::LOADED) ||
       (value.status() == ELoadStatus::ERROR))
    {
        return value;
    }

    // Retrieve metadata from disk
    int width, height;
    auto metadata = readImageMetadata(filename, width, height);

    value = imageCache.getOrCreate(key, width, height);
    if(value.status() == ELoadStatus::NONE)
    {
        ALICEVISION_LOG_WARNING("Cannot load image: " << filename << " image cache is full.");
        return value;
    }
    else if((value.status() == ELoadStatus::LOADED) ||
            (value.status() == ELoadStatus::ERROR))
    {
        return value;
    }

    std::shared_ptr<Image<TPix>> img = std::make_shared<Image<TPix>>(width, height);

    // load image from disk
    const ImageReadOptions options = imageCache.readOptionsCopy();

    readImage(filename, *img, options);

    // apply downscale
    if (downscaleLevel > 1)
    {
        imageAlgo::resizeImage(downscaleLevel, *img);
    }

    CacheValue newValue(img);
    const bool valid = imageCache.updateImage(key, newValue);
    if(!valid)
    {
        ALICEVISION_LOG_WARNING("Cannot load image: " << filename << ". Error while updating image in cache.");
    }
    return newValue;
}

template
CacheValue imageProviderSync<unsigned char>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);
template
CacheValue imageProviderSync<float>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);
template
CacheValue imageProviderSync<RGBColor>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);
template
CacheValue imageProviderSync<RGBfColor>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);
template
CacheValue imageProviderSync<RGBAColor>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);
template
CacheValue imageProviderSync<RGBAfColor>(ImageCache& imageCache, const std::string& filename, int downscaleLevel);


}  // namespace image
}  // namespace aliceVision
