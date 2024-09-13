// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/ImageCache.hpp>

#include <filesystem>
#include <string>
#include <omp.h>

#define BOOST_TEST_MODULE imageCaching

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::image;

namespace fs = std::filesystem;

BOOST_AUTO_TEST_CASE(create_cache)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().capacity, 256 * 1024 * 1024);
    BOOST_CHECK_EQUAL(cache.info().maxSize, 1024 * 1024 * 1024);
    BOOST_CHECK_EQUAL(cache.readOptions().workingColorSpace, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 0);
    BOOST_CHECK_EQUAL(cache.info().contentSize, 0);
}

BOOST_AUTO_TEST_CASE(get_non_loaded_image)
{
    ImageCache cache(0, 0, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto value = cache.get(CacheKeyInit<RGBAfColor>(filename));
    BOOST_CHECK_EQUAL(value.status(), ELoadStatus::NONE);
}

// BOOST_AUTO_TEST_CASE(load_without_space_in_cache)
// {
//     ImageCache cache(0, 0, EImageColorSpace::LINEAR);
//     const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
//     auto img1 = imageProviderSync<RGBAfColor>(cache, filename);
//     BOOST_CHECK_EQUAL(img1.status(), ELoadStatus::NONE);
// }

BOOST_AUTO_TEST_CASE(init_image_twice)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto img1 = cache.getOrCreate(CacheKeyInit<RGBAfColor>(filename), 512, 512);
    auto img2 = cache.getOrCreate(CacheKeyInit<RGBAfColor>(filename), 512, 512);
    // BOOST_CHECK_EQUAL(img1, img2);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 1);
    BOOST_CHECK_EQUAL(cache.info().contentSize, img1.memorySize());
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 0);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromCache, 1);
}

BOOST_AUTO_TEST_CASE(load_image_twice)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto img1 = imageProviderSync<RGBAfColor>(cache, filename);
    auto img2 = imageProviderSync<RGBAfColor>(cache, filename);
    // BOOST_CHECK_EQUAL(img1, img2);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 1);
    BOOST_CHECK_EQUAL(cache.info().contentSize, img1.memorySize());
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 1);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromCache, 2);
}


BOOST_AUTO_TEST_CASE(init_all_pixel_types)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto imgUChar = cache.get(CacheKeyInit<unsigned char>(filename));
    auto imgFloat = cache.get(CacheKeyInit<float>(filename));
    auto imgRGB = cache.get(CacheKeyInit<RGBColor>(filename));
    auto imgRGBf = cache.get(CacheKeyInit<RGBfColor>(filename));
    auto imgRGBA = cache.get(CacheKeyInit<RGBAColor>(filename));
    auto imgRGBAf = cache.get(CacheKeyInit<RGBAfColor>(filename));
    BOOST_CHECK_EQUAL(cache.info().nbImages, 0);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 0);
}

BOOST_AUTO_TEST_CASE(load_all_pixel_types)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto imgUChar = imageProviderSync<unsigned char>(cache, filename);
    auto imgFloat = imageProviderSync<float>(cache, filename);
    auto imgRGB = imageProviderSync<RGBColor>(cache, filename);
    auto imgRGBf = imageProviderSync<RGBfColor>(cache, filename);
    auto imgRGBA = imageProviderSync<RGBAColor>(cache, filename);
    auto imgRGBAf = imageProviderSync<RGBAfColor>(cache, filename);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 6);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 6);
}

BOOST_AUTO_TEST_CASE(load_all_pixel_types_multithreads)
{
    ALICEVISION_LOG_INFO("load_all_pixel_types_multithreads");
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string folder = std::string(THIS_SOURCE_DIR) + "/image_test";

    std::vector<std::string> files;
    // Collect all file paths
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (fs::is_regular_file(entry.status())) {
            files.push_back(entry.path().string());
        }
    }
    ALICEVISION_LOG_INFO("Nb images: " << files.size());

    #pragma omp parallel for
    for (size_t i = 0; i < files.size(); ++i)
    {
        const std::string filepath = files[i];
        ALICEVISION_LOG_INFO("Loading image " << filepath);

        auto imgUChar = imageProviderSync<unsigned char>(cache, filepath);
        auto imgFloat = imageProviderSync<float>(cache, filepath);
        auto imgRGB = imageProviderSync<RGBColor>(cache, filepath);
        auto imgRGBf = imageProviderSync<RGBfColor>(cache, filepath);
        auto imgRGBA = imageProviderSync<RGBAColor>(cache, filepath);
        auto imgRGBAf = imageProviderSync<RGBAfColor>(cache, filepath);
    }
    BOOST_CHECK_EQUAL(cache.info().nbImages, 8 * 6);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 8 * 6);
}

