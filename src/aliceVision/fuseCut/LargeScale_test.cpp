// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/fuseCut/LargeScale.hpp>

#include <string>

#define BOOST_TEST_MODULE fuseCut

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::fuseCut;

BOOST_AUTO_TEST_CASE(fuseCut_largeScale_io)
{
    sfmData::SfMData sfmData;
    mvsUtils::MultiViewParams mp{sfmData};
    LargeScale ls{&mp, "test_tmp"};
    ls.space = {{
      {1.1, 1.2, 1.3},
      {2.1, 2.2, 2.3},
      {3.1, 3.2, 3.3},
      {4.1, 4.2, 4.3},
      {5.1, 5.2, 5.3},
      {6.1, 6.2, 6.3},
      {7.1, 7.2, 7.3},
      {8.1, 8.2, 8.3},
    }};
    ls.dimensions = {1, 2, 3};
    ls.maxOcTreeDim = 5;
    ls.spaceFileName = "test_tmp/ls_space.txt";
    ls.saveSpaceToFile();

    LargeScale newLs{&mp, "test_tmp"};
    newLs.spaceFileName = "test_tmp/ls_space.txt";
    newLs.loadSpaceFromFile();

    BOOST_CHECK_EQUAL(ls.space, newLs.space);
    BOOST_CHECK_EQUAL(ls.dimensions, newLs.dimensions);
    BOOST_CHECK_EQUAL(ls.maxOcTreeDim, newLs.maxOcTreeDim);
}
