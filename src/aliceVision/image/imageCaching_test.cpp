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

BOOST_AUTO_TEST_CASE(create_cache) {
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().capacity, 256000000);
    BOOST_CHECK_EQUAL(cache.info().maxSize, 1024000000);
    BOOST_CHECK_EQUAL(cache.readOptions().workingColorSpace, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 0);
    BOOST_CHECK_EQUAL(cache.info().contentSize, 0);
}

BOOST_AUTO_TEST_CASE(load_without_space) {
    ImageCache cache(0, 0, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    BOOST_CHECK_THROW(auto img = cache.get<RGBAfColor>(filename, 0), std::exception);
}

BOOST_AUTO_TEST_CASE(load_image_twice) {
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto img1 = cache.get<RGBAfColor>(filename, 0);
    auto img2 = cache.get<RGBAfColor>(filename, 0);
    BOOST_CHECK_EQUAL(img1, img2);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 1);
    BOOST_CHECK_EQUAL(cache.info().contentSize, img1->MemorySize());
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 1);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromCache, 1);
}

BOOST_AUTO_TEST_CASE(reload_lower_scale) {
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto img1 = cache.get<RGBAfColor>(filename, 0);
    auto img2 = cache.get<RGBAfColor>(filename, 1);
    BOOST_CHECK_EQUAL(img1->Width() / 2, img2->Width());
    BOOST_CHECK_EQUAL(img1->Height() / 2, img2->Height());
    BOOST_CHECK_EQUAL(cache.info().nbImages, 2);
    BOOST_CHECK_EQUAL(cache.info().contentSize, img1->MemorySize() + img2->MemorySize());
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 1);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromHigherScale, 1);
}

BOOST_AUTO_TEST_CASE(load_all_pixel_types) {
    ImageCache cache(256, 1024, EImageColorSpace::LINEAR);
    const std::string filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
    auto imgUChar = cache.get<unsigned char>(filename, 0);
    auto imgFloat = cache.get<float>(filename, 0);
    auto imgRGB = cache.get<image::RGBColor>(filename, 0);
    auto imgRGBf = cache.get<image::RGBfColor>(filename, 0);
    auto imgRGBA = cache.get<image::RGBAColor>(filename, 0);
    auto imgRGBAf = cache.get<image::RGBAfColor>(filename, 0);
    BOOST_CHECK_EQUAL(cache.info().nbImages, 6);
    BOOST_CHECK_EQUAL(cache.info().nbLoadFromDisk, 6);
}
