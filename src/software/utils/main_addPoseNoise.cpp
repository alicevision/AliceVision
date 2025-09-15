// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/poseNoise.hpp>

#include <boost/program_options.hpp>

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
    std::string sfmDataOutputFilename;
    std::string outputSfMViewsAndPoses;
    float positionNoise = 0.;
    float rotationNoise = 0.;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("positionNoise", po::value<float>(&positionNoise)->default_value(positionNoise), "Noise level to add to view positions.")
        ("rotationNoise", po::value<float>(&rotationNoise)->default_value(rotationNoise), "Noise level to add to view orientations. [0..1]")
        ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses), "Path to the output SfMData file (with only views and poses).");
    // clang-format on

    CmdLine cmdline("This program adds noise to view positions and view orientations and saves it to a given file path.\n"
                    "AliceVision addPoseNoise");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (!sfm::addPoseNoise(sfmData, positionNoise, rotationNoise))
    {
        ALICEVISION_LOG_INFO("Error during sfmData processing");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Export SfM: " << sfmDataOutputFilename);
    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmDataOutputFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    if (!outputSfMViewsAndPoses.empty())
    {
        sfmDataIO::save(sfmData, outputSfMViewsAndPoses,
                        sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
