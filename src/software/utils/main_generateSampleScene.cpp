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
#include <aliceVision/sfmDataIO/sceneSample.hpp>

#include <filesystem>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <random>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace fs = std::filesystem;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmOutputDataFilepath;  // output file path for SfM Data
    sfmDataIO::ESceneType sceneSample = sfmDataIO::ESceneType::SCENE_SPHERE;  // scene type to generate ('cube' or 'sphere')
    int randomSeed = 1;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("output,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "Output sfm file to generate.")
        ("scene,s", po::value<sfmDataIO::ESceneType>(&sceneSample)->default_value(sceneSample),
         "Scene sample to generate. ['cube', 'sphere']");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");
    // clang-format on

    CmdLine cmdline("This program is used to generate a sample scene and save it to a given file path.\n"
                    "AliceVision generateSampleScene");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    // Eigen's Random() used by the scene generation relies on std::rand
    std::srand(randomSeed == -1 ? std::random_device()() : randomSeed);
    sfmDataIO::generateSampleScene(sfmData, sceneSample);

    ALICEVISION_LOG_INFO("Export SfM: " << sfmOutputDataFilepath);
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be write.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
