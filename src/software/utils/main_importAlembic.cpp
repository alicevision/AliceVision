// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfmDataIO/ExternalAlembicImporter.hpp>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string abcFilename;
    std::string sfmDataOutputFilename;
    double frameRate = 24.0;
    int imageWidth = 640;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&abcFilename)->required(),
         "The external Alembic file to import.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData file populated with the camera poses from the external Alembic file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("imageWidth", po::value<int>(&imageWidth)->default_value(imageWidth),
         "Alembic does not export the camera resolutions. Setup the image width for all images, the height will depend on the sensor size ratio.")
        ("framerate", po::value<double>(&frameRate)->default_value(frameRate),
         "Alembic frame rate to compute frame id from time.");
    // clang-format on

    CmdLine cmdline("AliceVision Alembic importer");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());


    std::unique_ptr<sfmDataIO::ExternalAlembicImporter> importer;

    try
    {
        importer = std::make_unique<sfmDataIO::ExternalAlembicImporter>(abcFilename, frameRate, imageWidth);
    }
    catch(...)
    {
        ALICEVISION_LOG_ERROR("Error during importer");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if (!importer->populateSfM(sfmData))
    {
        ALICEVISION_LOG_ERROR("Failed to import Alembic file.");
        return EXIT_FAILURE;
    }

    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmDataOutputFilename << "' cannot be write.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
