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

// Lighting calibration
#include <aliceVision/lightingEstimation/lightingCalibration.hpp>
#include <aliceVision/lightingEstimation/ellipseGeometry.hpp>

// Command line parameters
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <fstream>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

// Namespaces
namespace po = boost::program_options;
namespace fs = std::filesystem;

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    std::string inputPath;
    std::string inputDetection;
    std::string ouputJSON;
    std::string method;
    bool saveAsModel;
    bool ellipticEstimation;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPath,i", po::value<std::string>(&inputPath)->required(),
         "Path to the SfMData input.")
        ("inputDetection, j", po::value<std::string>(&inputDetection)->required(),
         "Path to the folder containing the JSON file that describes spheres' positions and radius")
        ("outputFile, o", po::value<std::string>(&ouputJSON)->required(),
         "Path to JSON output file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("saveAsModel, s", po::value<bool>(&saveAsModel)->default_value(false),
         "Calibration used for several datasets.")
        ("method, m", po::value<std::string>(&method)->default_value("brightestPoint"),
         "Method for light estimation.")
        ("ellipticEstimation, e", po::value<bool>(&ellipticEstimation)->default_value(false),
         "Used ellipse model for calibration spheres (more precise).");
    // clang-format on

    CmdLine cmdline("AliceVision lightingCalibration");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (fs::is_directory(inputPath))
    {
        ALICEVISION_LOG_ERROR("Directory input: WIP");
        ALICEVISION_THROW(std::invalid_argument, "Input directories are not yet supported");
    }
    else if (fs::path(inputDetection).extension() != ".json")
    {
        ALICEVISION_LOG_ERROR("The input detection file must be a JSON file.");
        ALICEVISION_THROW(std::invalid_argument, "JSON needed for sphere positions and radius.");
    }
    else
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, inputPath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read.");
            return EXIT_FAILURE;
        }

        lightingEstimation::lightCalibration(sfmData, inputDetection, ouputJSON, method, saveAsModel, ellipticEstimation);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
