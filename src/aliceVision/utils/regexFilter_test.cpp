// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/utils/regexFilter.hpp>

#include <string>
#include <vector>

#define BOOST_TEST_MODULE utilsRegexFilter

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(utils_regexMatching)
{
    const std::vector<std::string> exemplePaths
    {
        "C:/Users/img_0001.jpg", "C:/Users/img_0002.jpg", "C:/Users/img_0003.jpg", "C:/Users/img_0004.jpg",
        "C:/Users/img_0005.jpg", "C:/Users/img_0006.jpg", "C:/Users/img_0007.jpg", "C:/Users/img_0008.jpg",
        "C:/Users/img_0009.jpg", "C:/Users/img_0010.jpg", "C:/Users/img_0011.jpg", "C:/Users/img_0012.jpg",

        "C:/Users/000001.png",   "C:/Users/000002.png",   "C:/Users/000003.png",   "C:/Users/000004.png",

        "C:/Users/test00.exr",   "C:/Users/test01.exr",   "C:/Users/test02.exr",   "C:/Users/test03.exr"
    };

    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/*####.jpg");
        BOOST_CHECK_EQUAL(test.size(), 12);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/*000#.jpg");
        BOOST_CHECK_EQUAL(test.size(), 9);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/*000#.jpg");
        BOOST_CHECK_EQUAL(test.size(), 9);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        const std::vector<std::string> testDesired{"C:/Users/test00.exr", "C:/Users/test01.exr", "C:/Users/test02.exr", "C:/Users/test03.exr"};
        utils::filterStrings(test, "C:/Users/*#?.exr");
        BOOST_CHECK_EQUAL(test, testDesired);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/*_####.???");
        BOOST_CHECK_EQUAL(test.size(), 12);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/#####.???");
        BOOST_CHECK_EQUAL(test.size(), 0);
    }
    {
        std::vector<std::string> test;
        test.assign(exemplePaths.begin(), exemplePaths.end());
        utils::filterStrings(test, "C:/Users/@.???");
        BOOST_CHECK_EQUAL(test.size(), 4);
    }
}