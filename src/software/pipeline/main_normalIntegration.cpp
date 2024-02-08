// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Photometric Stereo
#include <aliceVision/photometricStereo/photometricDataIO.hpp>
#include <aliceVision/photometricStereo/normalIntegration.hpp>

// Command line parameters
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{
    namespace po = boost::program_options;

    system::Timer timer;

    bool isPerspective(true);
    std::string outputFolder;
    std::string pathToK;
    std::string inputPath;
    std::string sfmDataFile;

    // Image downscale factor during process
    int downscale = 1;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPath,i", po::value<std::string>(&inputPath)->required(),
         "Path to the input: a folder containing the normal maps and the masks.")
        ("outputPath,o", po::value<std::string>(&outputFolder)->required(),
         "Path to the output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("sfmDataFile,s", po::value<std::string>(&sfmDataFile)->default_value(""),
         "Path to the input SfMData file.")
        ("downscale,d", po::value<int>(&downscale)->default_value(downscale),
         "Downscale factor for faster results.");
    // clang-format on

    CmdLine cmdline("AliceVision normalIntegration");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (sfmDataFile.compare("") == 0)
    {
        photometricStereo::normalIntegration(inputPath, isPerspective, downscale, outputFolder);
    }
    else
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, sfmDataFile, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("The input file '" + sfmDataFile + "' cannot be read.");
            return EXIT_FAILURE;
        }
        photometricStereo::normalIntegration(sfmData, inputPath, isPerspective, downscale, outputFolder);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
};
