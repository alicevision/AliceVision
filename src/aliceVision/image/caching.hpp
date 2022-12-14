// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Image.hpp"
#include "pixelTypes.hpp"
#include "io.hpp"
#include "imageAlgo.hpp"

#include <aliceVision/system/Logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

#include <memory>
#include <unordered_map>
#include <functional>
#include <list>
#include <mutex>
#include <thread>


namespace aliceVision {
namespace image {

/**
 * @brief A struct used to identify a cached image using its file description, color type info and half-sampling level.
 */
struct CacheKey 
{
    std::string filename;
    int nbChannels;
    oiio::TypeDesc::BASETYPE typeDesc;
    int halfSampleLevel;
    std::time_t lastWriteTime;

    CacheKey(const std::string& path, int nchannels, oiio::TypeDesc::BASETYPE baseType, int level, std::time_t time) : 
        filename(path), 
        nbChannels(nchannels), 
        typeDesc(baseType),  
        halfSampleLevel(level), 
        lastWriteTime(time)
    {
    }

    bool operator==(const CacheKey& other) const
    {
        return (filename == other.filename &&
                nbChannels == other.nbChannels &&
                typeDesc == other.typeDesc &&
                halfSampleLevel == other.halfSampleLevel &&
                lastWriteTime == other.lastWriteTime);
    }
};

struct CacheKeyHasher
{
    std::size_t operator()(const CacheKey& key) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, key.filename);
        boost::hash_combine(seed, key.nbChannels);
        boost::hash_combine(seed, key.typeDesc);
        boost::hash_combine(seed, key.halfSampleLevel);
        boost::hash_combine(seed, key.lastWriteTime);
        return seed;
    }
};


/**
 * @brief A struct to store information about the cache current state and usage.
 */
struct CacheInfo
{
    /// memory usage limits
    const int capacity;
    const int maxSize;

    /// current state of the cache
    int nbImages = 0;
    int contentSize = 0;

    /// usage statistics
    int nbLoadFromDisk = 0;
    int nbLoadFromCache = 0;
    int nbLoadFromHigherScale = 0;
    int nbRemoveUnused = 0;

    CacheInfo(float capacity_MB, float maxSize_MB) : 
        capacity(capacity_MB * 1000000), 
        maxSize(maxSize_MB * 1000000)
    {
    }
};


/**
 * @brief A class to support shared pointers for all types of images.
 */
class CacheValue
{
public:
    /**
     * @brief Factory method to create a CacheValue instance that wraps a shared inter to an image
     * @param[in] img shared pointer to an image 
     * @result CacheValue instance wrapping the shared pointer
     */
    static CacheValue wrap(std::shared_ptr<Image<unsigned char>> img);
    static CacheValue wrap(std::shared_ptr<Image<float>> img);
    static CacheValue wrap(std::shared_ptr<Image<RGBColor>> img);
    static CacheValue wrap(std::shared_ptr<Image<RGBfColor>> img);
    static CacheValue wrap(std::shared_ptr<Image<RGBAColor>> img);
    static CacheValue wrap(std::shared_ptr<Image<RGBAfColor>> img);

private:
    /// constructor is private to only allow creating instances using the static methods above
    /// thus ensuring that only one of the shared pointers is non-null
    CacheValue();

public:
    /**
     * @brief Template method to get a shared pointer to the image with pixel type given as template argument.
     * @note At most one of the generated methods will provide a non-null pointer.
     * @return shared pointer to an image with the pixel type given as template argument
     */
    template<typename TPix>
    std::shared_ptr<Image<TPix>> get();

    /**
     * @brief Count the number of usages of the wrapped shared pointer.
     * @return the use_count of the wrapped shared pointer if there is one, otherwise 0
     */
    int useCount() const;

    /**
     * @brief Retrieve the memory size (in bytes) of the wrapped image.
     * @return the memory size of the wrapped image if there is one, otherwise 0
     */
    int memorySize() const;

private:
    std::shared_ptr<Image<unsigned char>> imgUChar = nullptr;
    std::shared_ptr<Image<float>> imgFloat = nullptr;
    std::shared_ptr<Image<RGBColor>> imgRGB = nullptr;
    std::shared_ptr<Image<RGBfColor>> imgRGBf = nullptr;
    std::shared_ptr<Image<RGBAColor>> imgRGBA = nullptr;
    std::shared_ptr<Image<RGBAfColor>> imgRGBAf = nullptr;
    
};

template<>
inline std::shared_ptr<Image<unsigned char>> CacheValue::get<unsigned char>() { return imgUChar; }

template<>
inline std::shared_ptr<Image<float>> CacheValue::get<float>() { return imgFloat; }

template<>
inline std::shared_ptr<Image<RGBColor>> CacheValue::get<RGBColor>() { return imgRGB; }

template<>
inline std::shared_ptr<Image<RGBfColor>> CacheValue::get<RGBfColor>() { return imgRGBf; }

template<>
inline std::shared_ptr<Image<RGBAColor>> CacheValue::get<RGBAColor>() { return imgRGBA; }

template<>
inline std::shared_ptr<Image<RGBAfColor>> CacheValue::get<RGBAfColor>() { return imgRGBAf; }


/**
 * @brief A class for retrieving images from disk (optionnaly downscaled) that implements a caching mechanism.
 * 
 * When creating an instance of this class, one must provide two memory size limits: 
 * - a capacity: the amount of space dedicated to storing images for later use
 * - a maximal size: the amount of space that cannot be exceeded.
 * 
 * When one attempts to retrieve an image through this cache, the following policy applies: 
 * 1. if the image is already in the cache, return it
 * 2. if the image fits in the capacity, load it, store it in the cache and return it
 * 3. find the Leat-Recently-Used image in the cache that is not used externally and which size is bigger than the missing capacity, 
 * if such an image exists remove it and apply step 2
 * 4. remove all images that are not used externally from Least- to Most-Recently-Used until the capacity is big enough to fit the image
 * or until there is nothing to remove
 * 5. if the image fits in the maximal size, load it, store it and return it
 * 6. the image is too big for the cache, throw an error.
 * 
 * In the process described above, we also take advantage of the cache content when loading an image: 
 * if the same image with a lower half-sampling level (i.e. higher resolution) exists in the cache, 
 * we take the high-resolution version of the image from the cache and create and new downscaled version of it
 * instead of loading the image from disk.
 */
class ImageCache 
{
public:
    /**
     * @brief Create a new image cache by defining memory usage limits and image reading options.
     * @param[in] capacity_MB the cache capacity (in MB)
     * @param[in] maxSize_MB the cache maximal size (in MB)
     * @param[in] options the reading options that will be used when loading images through this cache
     */
    ImageCache(float capacity_MB, float maxSize_MB, const ImageReadOptions& options);

    /**
     * @brief Destroy the cache and the unused images it contains.
     */
    ~ImageCache();

    /// make image cache class non-copyable
    ImageCache(const ImageCache&) = delete;
    ImageCache& operator=(const ImageCache&) = delete;

    /**
     * @brief Retrieve a cached image at a given half-sampling level.
     * @note This method is thread-safe.
     * @param[in] filename the image's filename on disk
     * @param[in] halfSampleLevel the half-sampling level
     * @return a shared pointer to the cached image
     * @throws std::runtime_error if the image does not fit in the maximal size of the cache
     */
    template<typename TPix>
    std::shared_ptr<Image<TPix>> get(const std::string& filename, int halfSampleLevel);

    /**
     * @return information on the current cache state and usage
     */
    inline const CacheInfo& info() const
    {
        return _info;
    }

    /**
     * @return the image reading options of the cache
     */
    inline const ImageReadOptions& readOptions() const
    {
        return _options;
    }

    /**
     * @brief Provide a description of the current internal state of the cache (useful for logging).
     * @return a string describing the cache content
     */
    std::string toString() const;

private:
    /**
     * @brief Load a new image corresponding to the given key and add it as a new entry in the cache.
     * @param[in] key the key used to identify the entry in the cache
     */
    template<typename TPix>
    void load(const CacheKey& key);

    CacheInfo _info;
    ImageReadOptions _options;
    std::unordered_map<CacheKey, CacheValue, CacheKeyHasher> _imagePtrs;
    /// ordered from LRU (Least Recently Used) to MRU (Most Recently Used)
    std::list<CacheKey> _keys;
    mutable std::mutex _mutex;

};


// Since some methods in the ImageCache class are templated
// their definition must be given in this header file

template<typename TPix>
std::shared_ptr<Image<TPix>> ImageCache::get(const std::string& filename, int halfSampleLevel)
{
    const std::lock_guard<std::mutex> lock(_mutex);

    ALICEVISION_LOG_DEBUG("[image::ImageCache] reading " << filename 
                         << " with half-sampling level " << halfSampleLevel
                         << " from thread " << std::this_thread::get_id());

    using TInfo = ColorTypeInfo<TPix>;

    auto lastWriteTime = boost::filesystem::last_write_time(filename);
    CacheKey keyReq(filename, TInfo::size, TInfo::typeDesc, halfSampleLevel, lastWriteTime);

    // find the requested image in the cached images
    {
        auto it = std::find(_keys.begin(), _keys.end(), keyReq);
        if (it != _keys.end())
        {
            // image becomes LRU
            _keys.erase(it);
            _keys.push_back(keyReq);

            _info.nbLoadFromCache++;

            ALICEVISION_LOG_DEBUG("[image::ImageCache] " << toString());
            return _imagePtrs.at(keyReq).get<TPix>();
        }
    }

    // retrieve image size
    int width, height;
    readImageSize(filename, width, height);
    int downscale = 1 << halfSampleLevel;
    int memSize = (width / downscale) * (height / downscale) * sizeof(TPix);
 
    // add image to cache if it fits in capacity
    if (memSize + _info.contentSize <= _info.capacity) 
    {
        load<TPix>(keyReq);

        ALICEVISION_LOG_DEBUG("[image::ImageCache] " << toString());
        return _imagePtrs.at(keyReq).get<TPix>();
    }

    // retrieve missing capacity
    int missingCapacity = memSize + _info.contentSize - _info.capacity;

    // find unused image with size bigger than missing capacity
    // remove it and add image to cache
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

                load<TPix>(keyReq);

                ALICEVISION_LOG_DEBUG("[image::ImageCache] " << toString());
                return _imagePtrs.at(keyReq).get<TPix>();
            }
            ++it;
        }
    }

    // remove as few unused images as possible
    while (missingCapacity > 0)
    {
        auto it = std::find_if(_keys.begin(), _keys.end(), [this](const CacheKey& k){
            return _imagePtrs.at(k).useCount() == 1;
        });
        
        if (it != _keys.end())
        {
            const CacheKey& key = *it;
            const CacheValue& value = _imagePtrs.at(key);

            _info.nbImages--;
            _info.contentSize -= value.memorySize();
            _imagePtrs.erase(key);
            _keys.erase(it);

            _info.nbRemoveUnused++;
        }
        else 
        {
            break;
        }
    }

    // add image to cache if it fits in maxSize
    if (memSize + _info.contentSize <= _info.maxSize) 
    {
        load<TPix>(keyReq);

        ALICEVISION_LOG_DEBUG("[image::ImageCache] " << toString());
        return _imagePtrs.at(keyReq).get<TPix>();
    }

    ALICEVISION_THROW_ERROR("[image::ImageCache] Failed to load image \n" << toString());

    return nullptr;
}

template<typename TPix>
void ImageCache::load(const CacheKey& key)
{
    auto img = std::make_shared<Image<TPix>>();

    // find the same image with a higher scale
    auto it = std::find_if(_keys.begin(), _keys.end(), [&key](const CacheKey& k){
        return k.filename == key.filename &&
               k.nbChannels == key.nbChannels &&
               k.typeDesc == key.typeDesc &&
               k.halfSampleLevel < key.halfSampleLevel &&
               k.lastWriteTime == key.lastWriteTime;
    });

    if (it != _keys.end())
    {
        // retrieve high-scale image from cache
        const CacheKey& keyHighScale = *it;
        CacheValue& valueHighScale = _imagePtrs.at(keyHighScale);

        // apply downscale
        int downscale = 1 << (key.halfSampleLevel - keyHighScale.halfSampleLevel);
        imageAlgo::resizeImage(downscale, *(valueHighScale.get<TPix>()), *img);

        _info.nbLoadFromHigherScale++;
    }
    else 
    {
        // load image from disk
        readImage(key.filename, *img, _options);

        // apply downscale
        int downscale = 1 << key.halfSampleLevel;
        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, *img, *img);
        }

        _info.nbLoadFromDisk++;
    }

    // create wrapper around shared pointer
    CacheValue value = CacheValue::wrap(img);

    // add to cache as MRU
    _imagePtrs.insert({key, value});
    _keys.push_back(key);

    // update memory usage
    _info.nbImages++;
    _info.contentSize += value.memorySize();
}

}
}
