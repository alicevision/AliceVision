// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/colmap.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <boost/program_options.hpp>

#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmDataFilename;
    std::string outDirectory;
    bool copyImages{false};

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outDirectory)->required(),
         "The base path where the folder structure will be created with the relevant cameras.txt, images.txt "
         "and points3D.txt files.")
        ("copyImages", po::value<bool>(&copyImages)->default_value(copyImages),
         "Copy original images to Colmap folder. This is required if your images are not all in the same "
         "folder.");
    // clang-format on

    CmdLine cmdline("Export an AV SfMData to a Colmap scene, creating the folder structure and "
                    "the scene files that can be used for running a MVS step.\n"
                    "AliceVision exportColmap");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (!fs::exists(outDirectory))
    {
        fs::create_directory(outDirectory);
    }

    // Read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("Unable to read the sfmdata file " << sfmDataFilename);
        return EXIT_FAILURE;
    }

    sfmDataIO::convertToColmapScene(sfmData, outDirectory, copyImages);

    return EXIT_SUCCESS;
}