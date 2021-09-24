// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>

#include <boost/filesystem.hpp>

#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmDataIO;

namespace fs = boost::filesystem;

BOOST_AUTO_TEST_CASE(Compatibility_abc_1_2_0)
{
    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    fs::path pathSource(__FILE__);
    fs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.0.abc";

    //TODO when we will have files to compare
    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

BOOST_AUTO_TEST_CASE(Compatibility_json_1_2_0)
{
    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    fs::path pathSource(__FILE__);
    fs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.0.json";

    //TODO when we will have files to compare
    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad);
}