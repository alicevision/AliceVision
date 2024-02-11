// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/image/all.hpp"

#define BOOST_TEST_MODULE imageCaching

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::image;

BOOST_AUTO_TEST_CASE(create_cache)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().capacity, 256 * 1024 * 1024);
    BOOST_CHECK_EQUAL(cache.info().maxSize, 1024 * 1024 * 1024);
    BOOST_CHECK_EQUAL(cache.readOptions().workingColorSpace, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 0);
    BOOST_CHECK_EQUAL(cache.info().contentSize, 0);
}

BOOST_AUTO_TEST_CASE(load_without_space)
{
    ImageCache cache(0, 0, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    BOOST_CHECK_THROW(auto img = cache.get<RGBAfColor>(filename), std::exception);
}

BOOST_AUTO_TEST_CASE(load_image_twice)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto img1 = cache.get<RGBAfColor>(filename);
    auto img2 = cache.get<RGBAfColor>(filename);
    BOOST_CHECK_EQUAL(img1, img2);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 1);
    BOOST_CHECK_EQUAL(cache.info().contentSize, img1->memorySize());
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 1);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromCache, 1);
}

BOOST_AUTO_TEST_CASE(load_all_pixel_types)
{
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto imgUChar = cache.get<unsigned char>(filename);
    auto imgFloat = cache.get<float>(filename);
    auto imgRGB = cache.get<image::RGBColor>(filename);
    auto imgRGBf = cache.get<image::RGBfColor>(filename);
    auto imgRGBA = cache.get<image::RGBAColor>(filename);
    auto imgRGBAf = cache.get<image::RGBAfColor>(filename);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 6);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 6);
}
