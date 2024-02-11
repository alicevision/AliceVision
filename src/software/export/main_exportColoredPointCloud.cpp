// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/colorize.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

// Convert from a SfMData format to another
int aliceVision_main(int argc, char** argv)
{
    // command-line parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outputSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputSfMDataFilename)->required(),
         "Output point cloud with visibilities as SfMData file.");
    // clang-format on

    CmdLine cmdline("AliceVision exportColoredPointCloud");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (outputSfMDataFilename.empty())
    {
        ALICEVISION_LOG_ERROR("No output filename specified.");
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // compute the scene structure color
    sfmData::colorizeTracks(sfmData);

    // export the SfMData scene in the expected format
    ALICEVISION_LOG_INFO("Saving output result to " << outputSfMDataFilename << "...");
    if (!sfmDataIO::save(sfmData, outputSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" + sfmDataFilename + "' cannot be save.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
