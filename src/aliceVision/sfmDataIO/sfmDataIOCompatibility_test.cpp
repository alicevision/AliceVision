// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>
#include <aliceVision/vfs/filesystem.hpp>

#include <boost/filesystem.hpp>
#include <boost/preprocessor/stringize.hpp>

#include <sstream>

#define BOOST_TEST_MODULE sfmDataIO

#include <boost/test/unit_test.hpp>

using namespace aliceVision;
using namespace aliceVision::sfmDataIO;

BOOST_AUTO_TEST_CASE(Compatibility_abc_1_2_0)
{
    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.0.abc";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

BOOST_AUTO_TEST_CASE(Compatibility_json_1_2_0)
{
    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.0.json";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad);
}

BOOST_AUTO_TEST_CASE(Compatibility_abc_1_2_1) {

    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.1.abc";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

BOOST_AUTO_TEST_CASE(Compatibility_json_1_2_1) {

    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.1.json";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

BOOST_AUTO_TEST_CASE(Compatibility_abc_1_2_2) {

    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.2.abc";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

BOOST_AUTO_TEST_CASE(Compatibility_json_1_2_2) {

    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    vfs::path toLoad = pathSource.parent_path() / "compatibilityData" / "scene_v1.2.2.json";

    sfmData::SfMData sfmDataLoad;
    BOOST_CHECK(sfmDataIO::Load(sfmDataLoad, toLoad.string(), ESfMData::ALL));

    BOOST_CHECK(sfmData == sfmDataLoad); 
}

/*
BOOST_AUTO_TEST_CASE(Compatibility_generate_files_current_version)
{
    sfmData::SfMData sfmData;
    generateSampleScene(sfmData);

    vfs::path pathSource(__FILE__);
    {
        vfs::path outputPath =
            pathSource.parent_path() / "compatibilityData" /
            "scene_v" BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_MAJOR) "." BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_MINOR) "." BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_REVISION) ".json";
        BOOST_CHECK(sfmDataIO::Save(sfmData, outputPath.string(), ESfMData::ALL));
    }
    {
        vfs::path outputPath =
            pathSource.parent_path() / "compatibilityData" /
            "scene_v" BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_MAJOR) "." BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_MINOR) "." BOOST_PP_STRINGIZE(ALICEVISION_SFMDATAIO_VERSION_REVISION) ".abc";
        BOOST_CHECK(sfmDataIO::Save(sfmData, outputPath.string(), ESfMData::ALL));
    }
}
*/
