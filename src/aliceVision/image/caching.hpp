// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Image.hpp"
#include "pixelTypes.hpp"
#include "io.hpp"
#include "resampling.hpp"

#include <memory>
#include <unordered_map>
#include <functional>
#include <list>


namespace aliceVision {
namespace image {

/**
 * @brief A struct used to identify a cached image using its filename and half-sampling level.
 */
struct CacheKey 
{
    std::string filename;
    int halfSampleLevel;

    CacheKey(const std::string& path = "",  int level = 0) : 
        filename(path),  
        halfSampleLevel(level)
    {
    }

    bool operator==(const CacheKey& other) const
    {
        return (filename == other.filename &&
                halfSampleLevel == other.halfSampleLevel);
    }
};

struct CacheKeyHasher
{
    std::size_t operator()(const CacheKey& elt) const noexcept
    {
        std::size_t h1 = std::hash<std::string>{}(elt.filename);
        std::size_t h2 = std::hash<int>{}(elt.halfSampleLevel);
        return h1 ^ (h2 << 1);
    }
};

/**
 * @brief A struct to store information about the cache current memory usage.
 */
struct CacheMemoryUsage
{
    int capacity;
    int maxSize;
    int nbImages;
    int contentSize;

    CacheMemoryUsage(int capacity_MB, int maxSize_MB) : 
        capacity(capacity_MB * 1000000), 
        maxSize(maxSize_MB * 1000000), 
        nbImages(0), 
        contentSize(0)
    {
    }
};

/**
 * @brief A class to retrieve images that handles loading from disk, downscaling and caching.
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
    ImageCache(int capacity_MB, int maxSize_MB, const ImageReadOptions& options);

    /**
     * @brief Destroy the cache and the images it contains.
     */
    ~ImageCache();

    // make image cache class non-copyable
    ImageCache(const ImageCache&) = delete;
    ImageCache& operator=(const ImageCache&) = delete;

    /**
     * @brief Retrieve a cached image at a given half-sampling level.
     * @param[in] filename the image's filename on disk
     * @param[in] halfSampleLevel the half-sampling level
     * @return a shared pointer to the cached image
     */
    std::shared_ptr<Image<RGBAfColor>> get(const std::string& filename, int halfSampleLevel);

    /**
     * @return the current memory usage of the cache
     */
    inline const CacheMemoryUsage& memoryUsage() const
    {
        return _memUsage;
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
     * @brief Add an entry (key, image) to the cache and mark it as most-recently-used (MRU).
     * @param[in] key the key used to identify the image in the cache
     * @param[in] img a shared pointer to the image
     */
    void add(const CacheKey& key, std::shared_ptr<Image<RGBAfColor>> img);

    CacheMemoryUsage _memUsage;
    ImageReadOptions _options;
    std::unordered_map<CacheKey, std::shared_ptr<Image<RGBAfColor>>, CacheKeyHasher> _imagePtrs;
    std::list<CacheKey> _keys; // ordered from LRU to MRU

};

}
}
