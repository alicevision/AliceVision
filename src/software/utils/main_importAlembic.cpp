// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
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
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string abcFilename;
    std::string sfmDataOutputFilename;
    std::string imagesDir;
    std::string extension;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&abcFilename)->required(), "Input Alembic file.")
        ("imagesDir", po::value<std::string>(&imagesDir)->required(), "directory with images")
        ("extension", po::value<std::string>(&extension)->required(), "images extension")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.");
    // clang-format on

    CmdLine cmdline("AliceVision Alembic importer");

    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    if (!std::filesystem::exists(imagesDir))
    {
        ALICEVISION_LOG_ERROR("Images directory does not exist.");
        return EXIT_FAILURE;
    }

    if (!std::filesystem::is_directory(imagesDir))
    {
        ALICEVISION_LOG_ERROR("Images directory value is not a directory.");
        return EXIT_FAILURE;
    }

    std::vector<std::string> files;
    for (const auto & item: std::filesystem::directory_iterator(imagesDir))
    {
        const std::string itemExtension = item.path().extension().string();
        if (boost::algorithm::to_lower_copy(itemExtension) == extension)
        {
            files.push_back(std::filesystem::canonical(item.path()).string());
        }
    }
    std::sort(files.begin(), files.end());
    

    std::unique_ptr<sfmDataIO::ExternalAlembicImporter> importer;

    try
    {
        importer = std::make_unique<sfmDataIO::ExternalAlembicImporter>(abcFilename);
    }
    catch(...)
    {
        ALICEVISION_LOG_ERROR("Error during importer");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    importer->populateSfM(sfmData, files);

    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmDataOutputFilename << "' cannot be write.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
