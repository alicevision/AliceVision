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
#include <aliceVision/utils/filesIO.hpp>

#include <boost/functional/hash.hpp>

#include <filesystem>
#include <memory>
#include <unordered_map>
#include <functional>
#include <list>
#include <mutex>
#include <thread>
#include <algorithm>

namespace aliceVision {
namespace image {

enum class ELoadStatus
{
    NONE = 0,
    NOT_LOADED = 1,
    LOADING = 2,
    LOADED = 3,
    ERROR = 4
};

std::ostream& operator<<(std::ostream& os, ELoadStatus status);

template<typename TPix>
struct CacheKeyInit
{
    using TInfo = ColorTypeInfo<TPix>;
    CacheKeyInit(const std::string& filename, int downscaleLevel = 1)
    : filename(filename), downscaleLevel(downscaleLevel)
    {
        if (downscaleLevel < 1)
        {
            ALICEVISION_THROW_ERROR("[image] ImageCache CacheKeyInit: cannot load image with downscale level < 1, "
                                    << "request was made with downscale level " << downscaleLevel);
        }
    }
    std::string filename;
    int downscaleLevel;
};

/**
 * @brief A struct used to identify a cached image using its file description, color type info and downscale level.
 */
struct CacheKey
{
    std::string filename;
    int nbChannels;
    oiio::TypeDesc::BASETYPE typeDesc;
    int downscaleLevel;

    CacheKey(const CacheKey& other)
    {
        filename = other.filename;
        nbChannels = other.nbChannels;
        typeDesc = other.typeDesc;
        downscaleLevel = other.downscaleLevel;
    }
    CacheKey(const std::string& path, int nchannels, oiio::TypeDesc::BASETYPE baseType, int level, std::time_t time)
      : filename(path),
        nbChannels(nchannels),
        typeDesc(baseType),
        downscaleLevel(level)
    {}
    template<typename TPix>
    CacheKey(const CacheKeyInit<TPix>& init)
      : filename(init.filename),
        nbChannels(ColorTypeInfo<TPix>::size),
        typeDesc(ColorTypeInfo<TPix>::typeDesc),
        downscaleLevel(init.downscaleLevel)
    {}

    bool operator==(const CacheKey& other) const
    {
        return (filename == other.filename &&
            nbChannels == other.nbChannels &&
            typeDesc == other.typeDesc &&
            downscaleLevel == other.downscaleLevel);
    }

    std::ostream& operator<<(std::ostream& os) const
    {
        os << "CacheKey: " << filename << " " << nbChannels << " " << typeDesc << " " << downscaleLevel;
        return os;
    }

    int baseTypeSize() const;
    int pixelTypeSize() const;
    int imageMemorySize(const int width, const int height) const;
};

struct CacheKeyHasher
{
    std::size_t operator()(const CacheKey& key) const noexcept
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, key.filename);
        boost::hash_combine(seed, key.nbChannels);
        boost::hash_combine(seed, key.typeDesc);
        boost::hash_combine(seed, key.downscaleLevel);
        return seed;
    }
};

/**
 * @brief A struct to store information about the cache current state and usage.
 */
struct CacheInfo
{
    /// memory usage limits
    const std::atomic<unsigned long long int> capacity;
    const std::atomic<unsigned long long int> maxSize;

    /// current state of the cache
    std::atomic<int> nbImages = 0;
    std::atomic<unsigned long long int> contentSize = 0;

    /// usage statistics
    std::atomic<int> nbLoadFromDisk = 0;
    std::atomic<int> nbLoadFromCache = 0;
    std::atomic<int> nbRemoveUnused = 0;

    CacheInfo(float capacity_MiB, float maxSize_MiB)
      : capacity(capacity_MiB * 1024 * 1024),
        maxSize(maxSize_MiB * 1024 * 1024)
    {
        // Check that max size is higher than capacity
        if (maxSize < capacity)
        {
            ALICEVISION_THROW_ERROR("[image] ImageCache: maximum size must be higher than capacity");
        }
    }
};

/**
 * @brief A class to support shared pointers for all types of images.
 */
class CacheValue
{
  public:
    CacheValue() = default;
    CacheValue(const CacheValue&) = default;

    CacheValue& operator=(const CacheValue&) = default;

    CacheValue(unsigned long long int memorySize)
    : _memorySize(memorySize)
    , _loadStatus(ELoadStatus::NOT_LOADED)
    {
    }

    CacheValue(std::shared_ptr<Image<unsigned char>> img);
    CacheValue(std::shared_ptr<Image<float>> img);
    CacheValue(std::shared_ptr<Image<RGBColor>> img);
    CacheValue(std::shared_ptr<Image<RGBfColor>> img);
    CacheValue(std::shared_ptr<Image<RGBAColor>> img);
    CacheValue(std::shared_ptr<Image<RGBAfColor>> img);

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

    void setStatus(ELoadStatus status) { _loadStatus = status; }
    ELoadStatus status() const { return _loadStatus; }

    /**
     * @brief Retrieve the memory size (in bytes) reserved for the wrapped image.
     */
    unsigned long long int memorySize() const { return _memorySize; }
    void setMemorySize(unsigned long long int v) { _memorySize = v; }

    /**
     * @brief Retrieve the memory size (in bytes) currently allocated for the wrapped image.
     */
    unsigned long long int effectiveMemorySize() const;

    /**
     * @brief Retrieve the memory size of the base type of the image.
    */
    // int baseTypeSize() const;

    // void set(const CacheKey& key)
    // {
    //     filename = key.filename;
    //     nbChannels = key.nbChannels;
    //     typeDesc = key.typeDesc;
    //     downscaleLevel = key.downscaleLevel;
    // }

    // std::string filename;
    // oiio::TypeDesc::BASETYPE typeDesc;
    // int downscaleLevel = 0;
    // std::time_t lastWriteTime = 0;
    // int nbChannels = 0;
    // int width = 0;
    // int height = 0;

  private:
    std::shared_ptr<Image<unsigned char>> imgUChar;
    std::shared_ptr<Image<float>> imgFloat;
    std::shared_ptr<Image<RGBColor>> imgRGB;
    std::shared_ptr<Image<RGBfColor>> imgRGBf;
    std::shared_ptr<Image<RGBAColor>> imgRGBA;
    std::shared_ptr<Image<RGBAfColor>> imgRGBAf;
    unsigned long long int _memorySize = 0;
    ELoadStatus _loadStatus = ELoadStatus::NONE;
};

template<>
inline std::shared_ptr<Image<unsigned char>> CacheValue::get<unsigned char>()
{
    return imgUChar;
}

template<>
inline std::shared_ptr<Image<float>> CacheValue::get<float>()
{
    return imgFloat;
}

template<>
inline std::shared_ptr<Image<RGBColor>> CacheValue::get<RGBColor>()
{
    return imgRGB;
}

template<>
inline std::shared_ptr<Image<RGBfColor>> CacheValue::get<RGBfColor>()
{
    return imgRGBf;
}

template<>
inline std::shared_ptr<Image<RGBAColor>> CacheValue::get<RGBAColor>()
{
    return imgRGBA;
}

template<>
inline std::shared_ptr<Image<RGBAfColor>> CacheValue::get<RGBAfColor>()
{
    return imgRGBAf;
}

/**
 * @brief A class for retrieving images from disk (optionally downscaled) that implements a caching mechanism.
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
 */
class ImageCache
{
  public:
    /**
     * @brief Create a new image cache by defining memory usage limits and image reading options.
     * @param[in] capacity_MiB the cache capacity (in MiB)
     * @param[in] maxSize_MiB the cache maximal size (in MiB)
     * @param[in] options the reading options that will be used when loading images through this cache
     */
    ImageCache(float capacity_MiB, float maxSize_MiB, const ImageReadOptions& options);

    /**
     * @brief Destroy the cache and the unused images it contains.
     */
    ~ImageCache();

    /// make image cache class non-copyable
    ImageCache(const ImageCache&) = delete;
    ImageCache& operator=(const ImageCache&) = delete;

    CacheValue get(const CacheKey& keyReq);
    const CacheValue getReadOnly(const CacheKey& keyReq) const;
    CacheValue getAtMaxDownscale(const CacheKey& key);
    CacheValue getOrCreate(const CacheKey& key, int width, int height, bool lazyCleaning = true);
    bool updateImage(const CacheKey& key, CacheValue newValue);
    bool contains(const CacheKey& key) const;
    bool containsValidImage(const CacheKey& key) const;

    /**
     * @return information on the current cache state and usage
     */
    inline const CacheInfo& info() const { return _info; }

    /**
     * @return the image reading options of the cache
     */
    inline const ImageReadOptions& readOptions() const { return _options; }
    inline ImageReadOptions readOptionsCopy() const
    {
        std::scoped_lock<std::mutex> lock(_mutexReadOptions);
        return _options;
    }

    /**
     * @brief Provide a description of the current internal state of the cache (useful for logging).
     * @return a string describing the cache content
     */
    std::string toString() const;

private:
    CacheValue* _getPtr(const CacheKey& keyReq);
    const CacheValue* _getPtrReadOnly(const CacheKey& keyReq) const;
    CacheValue _getOrCreate(const CacheKey& key, int width, int height, bool lazyCleaning = true);
    bool _containsValidImage(const CacheKey& key) const;
    void _createBuffer(const CacheKey& key, unsigned long long int memorySize);
    std::string _toString() const;

  private:
    CacheInfo _info;
    ImageReadOptions _options;
    std::unordered_map<CacheKey, CacheValue, CacheKeyHasher> _imagePtrs;
    /// ordered from LRU (Least Recently Used) to MRU (Most Recently Used)
    std::list<CacheKey> _keys;

    mutable std::mutex _mutex;
    mutable std::mutex _mutexReadOptions;
};

template<typename TPix>
CacheValue imageProviderSync(ImageCache& imageCache, const std::string& filename, int downscaleLevel = 1);

}  // namespace image
}  // namespace aliceVision
