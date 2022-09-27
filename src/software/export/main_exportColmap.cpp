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

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int aliceVision_main(int argc, char* argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outDirectory;
    bool copyImages{false};

    po::options_description allParams("Export an AV sfmdata to a Colmap scene, creating the folder structure and "
                                      "the scene files that can be used for running a MVS step");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("output,o", po::value<std::string>(&outDirectory)->required(),
        "The base path where the folder structure will be created with the relevant cameras.txt, images.txt "
             "and points3D.txt files.")
        ("copyImages", po::value<bool>(&copyImages)->default_value(copyImages),
             "Copy original images to colmap folder. This is required if your images are not all in the same "
             "folder.");

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal,  error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    if(!fs::exists(outDirectory))
    {
        fs::create_directory(outDirectory);
    }

    // Read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("Unable to read the sfmdata file " << sfmDataFilename);
        return EXIT_FAILURE;
    }

    sfmDataIO::convertToColmapScene(sfmData, outDirectory, copyImages);

    return EXIT_SUCCESS;
}