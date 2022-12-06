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
    BOOST_CHECK_EQUAL(cache.memoryUsage().capacity, 256000000);
    BOOST_CHECK_EQUAL(cache.memoryUsage().maxSize, 1024000000);
    BOOST_CHECK_EQUAL(cache.readOptions().workingColorSpace, EImageColorSpace::LINEAR);
    BOOST_CHECK_EQUAL(cache.memoryUsage().nbImages, 0);
    BOOST_CHECK_EQUAL(cache.memoryUsage().contentSize, 0);
}
