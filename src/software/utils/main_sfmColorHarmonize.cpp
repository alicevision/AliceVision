// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2013 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <software/utils/sfmColorHarmonize/colorHarmonizeEngineGlobal.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>
#include <filesystem>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outputFolder;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    EHistogramSelectionMethod selectionMethod;
    int imgRef;

    // user optional parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Output path.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken()->required(),
         "Path to folder(s) in which computed matches are stored.")
        ("referenceImage", po::value<int>(&imgRef)->required(),
         "Reference image ID.")
        ("selectionMethod", po::value<EHistogramSelectionMethod>(&selectionMethod)->required(),
         EHistogramSelectionMethod_description().c_str());

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str());
    // clang-format on

    CmdLine cmdline("AliceVision sfmColorHarmonize");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (sfmDataFilename.empty())
    {
        ALICEVISION_LOG_ERROR("It is an invalid file input");
        return EXIT_FAILURE;
    }

    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    if (!fs::exists(outputFolder))
        fs::create_directory(outputFolder);

    // harmonization process

    aliceVision::system::Timer timer;

    ColorHarmonizationEngineGlobal colorHarmonizeEngine(
      sfmDataFilename, featuresFolders, matchesFolders, outputFolder, describerTypes, selectionMethod, imgRef);

    if (colorHarmonizeEngine.Process())
    {
        ALICEVISION_LOG_INFO("Color harmonization took: " << timer.elapsed() << " s");
        return EXIT_SUCCESS;
    }
    else
    {
        ALICEVISION_LOG_ERROR("Something goes wrong during the color harmonization process.");
    }

    return EXIT_FAILURE;
}
