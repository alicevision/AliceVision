// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename1, sfmDataFilename2;
    std::string outSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("firstinput,i1", po::value<std::string>(&sfmDataFilename1)->required(),
         "First SfMData file to merge.")
        ("secondinput,i2", po::value<std::string>(&sfmDataFilename2)->required(),
         "Second SfMData file to merge.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmMerge");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData1;
    if (!sfmDataIO::load(sfmData1, sfmDataFilename1, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename1 << "' cannot be read");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData2;
    if (!sfmDataIO::load(sfmData2, sfmDataFilename2, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename2 << "' cannot be read");
        return EXIT_FAILURE;
    }

    {
        auto& views1 = sfmData1.getViews();
        auto& views2 = sfmData2.getViews();
        const size_t totalSize = views1.size() + views2.size();

        views1.insert(views2.begin(), views2.end());
        if (views1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common view ID between both SfMData");
            return EXIT_FAILURE;
        }
    }

    {
        auto& intrinsics1 = sfmData1.getIntrinsics();
        auto& intrinsics2 = sfmData2.getIntrinsics();
        const size_t totalSize = intrinsics1.size() + intrinsics2.size();

        intrinsics1.insert(intrinsics2.begin(), intrinsics2.end());
        if (intrinsics1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common intrinsics ID between both SfMData");
            return EXIT_FAILURE;
        }
    }

    {
        auto& rigs1 = sfmData1.getRigs();
        auto& rigs2 = sfmData2.getRigs();
        const size_t totalSize = rigs1.size() + rigs2.size();

        rigs1.insert(rigs2.begin(), rigs2.end());
        if (rigs1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common rigs ID between both SfMData");
            return EXIT_FAILURE;
        }
    }

    {
        auto& landmarks1 = sfmData1.getLandmarks();
        auto& landmarks2 = sfmData2.getLandmarks();
        const size_t totalSize = landmarks1.size() + landmarks2.size();

        landmarks1.insert(landmarks2.begin(), landmarks2.end());
        if (landmarks1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common rigs landmarks between both SfMData");
            return EXIT_FAILURE;
        }
    }

    sfmData1.addFeaturesFolders(sfmData2.getRelativeFeaturesFolders());
    sfmData1.addMatchesFolders(sfmData2.getRelativeMatchesFolders());

    if (!sfmDataIO::save(sfmData1, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
