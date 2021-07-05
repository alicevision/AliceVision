// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <aliceVision/system/Logger.hpp>
#include "aliceVision/numeric/numeric.hpp"
#include <aliceVision/numeric/gps.hpp>

#define BOOST_TEST_MODULE numeric

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace std;


BOOST_AUTO_TEST_CASE ( conversionTest )
{
    const Vec3 gps{43.597824, 1.4548992, 150};
    const Vec3 expected{4625021.304, 117467.403, 4375942.605};
    const auto res = WGS84ToCartesian(gps);

    EXPECT_MATRIX_CLOSE_FRACTION(res, expected, 1e-5);
}

BOOST_AUTO_TEST_CASE ( parseGPSTest )
{
    const std::string gps{"1, 27, 19.0008"};
    const double expected{1.45528};
    const auto res = parseGPSFromString(gps, "E");

    BOOST_CHECK_CLOSE_FRACTION(res, expected, 1e-5);

    BOOST_CHECK_THROW(parseGPSFromString(gps, "G"), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE ( parseAltTest )
{
    const std::string gps{"10785.65"};
    const double expected{10785.65};
    const auto res = parseAltitudeFromString(gps, "0");
    BOOST_CHECK_CLOSE_FRACTION(res, expected, 1e-5);
    BOOST_CHECK(parseAltitudeFromString(gps, "1") < 0);

    BOOST_CHECK_THROW(parseAltitudeFromString(gps, "N"), std::invalid_argument);
}