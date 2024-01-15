// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outRSfMDataFilename;
    std::string outNRSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file to align.")
        ("reconstructedOutput", po::value<std::string>(&outRSfMDataFilename)->required(),
         "Output SfMData scene.")
        ("notReconstructedOutput", po::value<std::string>(&outNRSfMDataFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmSplitReconstructed");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Create reconstructed only sfmData
    {
        sfmData::SfMData outReconstructed = sfmData;
        auto& views = outReconstructed.getViews();

        auto it = views.begin();
        while (it != views.end())
        {
            if (sfmData.isPoseAndIntrinsicDefined(it->first))
            {
                it++;
                continue;
            }

            it = views.erase(it);
        }

        // Export the SfMData scene in the expected format
        if (!sfmDataIO::save(outReconstructed, outRSfMDataFilename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outRSfMDataFilename << "'");
            return EXIT_FAILURE;
        }
    }

    // Create non reconstructed only sfmData
    {
        sfmData::SfMData outNonReconstructed = sfmData;
        outNonReconstructed.getConstraints2D().clear();
        outNonReconstructed.getRotationPriors().clear();
        outNonReconstructed.getLandmarks().clear();
        auto& views = outNonReconstructed.getViews();

        auto it = views.begin();
        while (it != views.end())
        {
            if (!sfmData.isPoseAndIntrinsicDefined(it->first))
            {
                it++;
                continue;
            }

            it = views.erase(it);
        }

        // Export the SfMData scene in the expected format
        if (!sfmDataIO::save(outNonReconstructed, outNRSfMDataFilename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outNRSfMDataFilename << "'");
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
