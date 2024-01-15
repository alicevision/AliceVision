// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/middlebury.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <filesystem>
#include <iostream>
#include <vector>
#include <ostream>
#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

/*
 * This program generate an SfMData from the configuration files of the Middlebury dataset
 */
int aliceVision_main(int argc, char** argv)
{
    // the text file containing the cameras
    std::string middleburyFile;
    // the sfm data file to generate
    std::string sfmDataFilename;
    // whether to use shared intrinsics among all views
    bool uniqueIntrinsics{false};
    // whether to import or not the poses
    bool importPoses{true};
    // whether to lock or not the intrinsics
    bool lockIntrinsics{true};
    // whether to lock or not the poses
    bool lockPoses{true};

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&middleburyFile)->required(),
         "The text file containing the cameras (e.g. temple_par.txt).")
        ("output,o", po::value<std::string>(&sfmDataFilename)->required(),
         "Output SfMData filename.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("uniqueIntrinsics", po::bool_switch(&uniqueIntrinsics),
          "Consider all the camera having the same intrinsics (the first camera instrinsics will be used for all the others).")
        ("importPoses", po::value<bool>(&importPoses)->default_value(importPoses),
         "Import the poses, disable this if you want, e.g. test the sfm part and assess the camera pose estimation.")
        ("lockIntrinsics", po::value<bool>(&lockIntrinsics)->default_value(lockIntrinsics),
         "Set the intrinsics to locked, so they will not be refined in the SfM step.")
        ("lockPoses", po::value<bool>(&lockPoses)->default_value(lockPoses),
         "Set the poses to locked, so they will not be refined in the SfM step.");
    // clang-format on

    CmdLine cmdline(
      "This program generates an SfMData from the configuration files of the Middlebury dataset: https://vision.middlebury.edu/mview/data\n"
      "AliceVision importMiddlebury");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // check input file exist
    if (!exists(fs::path(middleburyFile)))
    {
        ALICEVISION_LOG_ERROR("File " << middleburyFile << " does not exist");
        return EXIT_FAILURE;
    }

    // get the base path
    const auto basePath = fs::path(middleburyFile).parent_path().string();

    // parse file
    const auto sfmData = sfmDataIO::middleburySceneToSfmData(middleburyFile, basePath, uniqueIntrinsics, importPoses, lockIntrinsics, lockPoses);

    if (!sfmDataIO::save(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("Unable to save " << sfmDataFilename);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
