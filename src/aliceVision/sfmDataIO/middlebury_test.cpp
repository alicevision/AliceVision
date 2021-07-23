// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE middlebury

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmDataIO/middlebury.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(middlebury_parse)
{
    const std::string line{"temple0299.png "
                           "1520.400000 0.000000 302.320000 "
                           "0.000000 1525.900000 246.870000 "
                           "0.000000 0.000000 1.000000 "
                           "0.10793659924390045000 -0.99055951509200579000 0.08450761861721302300 "
                           "-0.32994878599451571000 0.04449292035991074500 0.94294972223262863000 "
                           "-0.93780781035583538000 -0.12966197244580752000 -0.32203149494584499000 "
                           "0.06997376920479636600 -0.01263011688236693700 0.56781167778967301000"};
    const std::string imageNameGT{"temple0299.png"};
    const auto matKGT = (Mat3() << 1520.400000, 0.000000, 302.320000,
                         0.000000, 1525.900000, 246.870000,
                         0.000000, 0.000000, 1.000000).finished();
    const auto rotationGT= (Mat3() << 0.10793659924390045000, -0.99055951509200579000, 0.08450761861721302300,
                        -0.32994878599451571000, 0.04449292035991074500, 0.94294972223262863000,
                        -0.93780781035583538000, -0.12966197244580752000, -0.32203149494584499000).finished();;
    const auto translationGT= (Vec3() << 0.06997376920479636600, -0.01263011688236693700, 0.56781167778967301000).finished();;

    std::string imageName;
    Mat3 matK;
    Mat3 rotation;
    Vec3 translation;
    sfmDataIO::parseMiddleburyCamera(line, imageName, matK, rotation, translation);

    const auto threshold{1e-4};
    BOOST_CHECK(imageName == imageNameGT);
    EXPECT_MATRIX_CLOSE_FRACTION(matK, matKGT, threshold);
    EXPECT_MATRIX_CLOSE_FRACTION(rotation, rotationGT, threshold);
    EXPECT_MATRIX_CLOSE_FRACTION(translation, translationGT, threshold);


}