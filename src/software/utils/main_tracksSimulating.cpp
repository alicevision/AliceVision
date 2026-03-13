// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/tracksUtils.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>

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
    std::string tracksFilename;
    double sigmaNoise = 0.0;
    double outlierRatio = 0.0;
    double outlierEpipolarRatio = 0.2;
    bool randomNoiseVariancePerView = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&tracksFilename)->required(),
         "Path to the tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("sigmaNoise", po::value<double>(&sigmaNoise)->default_value(sigmaNoise))
        ("outlierRatio", po::value<double>(&outlierRatio)->default_value(outlierRatio))
        ("outlierEpipolarRatio", po::value<double>(&outlierEpipolarRatio)->default_value(outlierEpipolarRatio))
        ("randomNoiseVariancePerView", po::value<bool>(&randomNoiseVariancePerView)->default_value(randomNoiseVariancePerView), "Use different noise variance per view.");

    // clang-format on

    CmdLine cmdline("AliceVision tracksSimulating");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    track::TracksMap mapTracks;
    track::simulateTracks(sfmData, sigmaNoise, outlierRatio, outlierEpipolarRatio, randomNoiseVariancePerView, mapTracks);

     // write the json file
    ALICEVISION_LOG_INFO("Export to file");
    boost::json::value jv = boost::json::value_from(mapTracks);
    std::ofstream of(tracksFilename);
    of << boost::json::serialize(jv);
    of.close();

    return EXIT_SUCCESS;
}
