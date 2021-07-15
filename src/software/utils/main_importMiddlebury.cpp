// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/middlebury.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

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
namespace bfs = boost::filesystem;


/*
 * This program generate an SfMData from the configuration files of the Middlebury dataset
 */
int aliceVision_main(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
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

    po::options_description allParams("This program generate an SfMData from the configuration files of the Middlebury dataset: https://vision.middlebury.edu/mview/data");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()("input,i", po::value<std::string>(&middleburyFile)->required(), "The text file containing the cameras (e.g. temple_par.txt).")
        ("output,o", po::value<std::string>(&sfmDataFilename)->required(), "Output sfmdata filename");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("uniqueIntrinsics", po::bool_switch(&uniqueIntrinsics),
         "Consider all the camera having the same intrinsics (the first camera instrinsics will be used for all the others")
        ("importPoses", po::value<bool>(&importPoses)->default_value(importPoses),
         "Import the poses, disable this if you want, e.g. test the sfm part and assess the camera pose estimation.")
        ("lockIntrinsics", po::value<bool>(&lockIntrinsics)->default_value(lockIntrinsics),
         "Set the intrinsics to locked, so they will not be refined in the sfm step.")
        ("lockPoses", po::value<bool>(&lockPoses)->default_value(lockPoses),
         "Set the poses to locked, so they will not be refined in the sfm step")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check input file exist
    if(!exists(bfs::path(middleburyFile)))
    {
        ALICEVISION_LOG_ERROR("File " << middleburyFile << " does not exist");
        return EXIT_FAILURE;
    }

    // get the base path
    const auto basePath = bfs::path(middleburyFile).parent_path().string();

    // parse file
    const auto sfmData =
        sfmDataIO::middleburySceneToSfmData(middleburyFile, basePath, uniqueIntrinsics, importPoses, lockIntrinsics, lockPoses);

    if(!sfmDataIO::Save(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("Unable to save " << sfmDataFilename);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
