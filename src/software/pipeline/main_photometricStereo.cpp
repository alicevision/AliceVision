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
#include <aliceVision/photometricStereo/photometricStereo.hpp>

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

#include <filesystem>
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

namespace po = boost::program_options;
namespace fs = std::filesystem;

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    std::string inputPath;
    std::string maskPath;
    std::string outputPath;
    std::string pathToLightData;

    // PhotometricStereo parameters
    photometricStereo::PhotometricSteroParameters PSParameters;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPath,i", po::value<std::string>(&inputPath)->required(),
         "Path to the input: could be an SfMData file or a folder with images.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputPath,o", po::value<std::string>(&outputPath)->default_value(""),
         "Output path.")
        ("maskPath,m", po::value<std::string>(&maskPath)->default_value(""),
         "Path to mask folder/file.")
        ("pathToJSONLightFile,l", po::value<std::string>(&pathToLightData)->default_value("defaultJSON.txt"),
         "Path to light file (JSON). If empty, .txt files are expected in the image folder.")
        ("SHOrder,s", po::value<size_t>(&PSParameters.SHOrder)->default_value(0),
         "Spherical harmonics order, 0 = directional, 1 = directional + ambiant, 2 = second order SH.")
        ("removeAmbiant,a", po::value<bool>(&PSParameters.removeAmbiant)->default_value(false),
         "True if the ambiant light is to be removed on PS images, false otherwise.")
        ("isRobust,r", po::value<bool>(&PSParameters.isRobust)->default_value(false),
         "True to use the robust algorithm, false otherwise.")
        ("downscale, d", po::value<int>(&PSParameters.downscale)->default_value(1),
         "Downscale factor for faster results.");
    // clang-format on

    CmdLine cmdline("AliceVision photometricStereo");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // If the path to light data is empty, set it to inputPath :
    if (pathToLightData.compare("") && fs::is_directory(inputPath))
    {
        ALICEVISION_LOG_WARNING("Warning: path to light data has been set to inputpath folder");
        pathToLightData = inputPath;
    }

    image::Image<image::RGBfColor> normalsIm;
    image::Image<image::RGBfColor> albedoIm;

    if (fs::is_directory(inputPath))
    {
        photometricStereo::photometricStereo(inputPath, pathToLightData, outputPath, PSParameters, normalsIm, albedoIm);
    }
    else
    {
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, inputPath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read.");
            return EXIT_FAILURE;
        }
        photometricStereo::photometricStereo(sfmData, pathToLightData, maskPath, outputPath, PSParameters, normalsIm, albedoIm);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
