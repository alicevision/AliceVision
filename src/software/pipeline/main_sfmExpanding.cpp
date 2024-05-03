// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/hardwareContext.hpp>

#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/sfm/pipeline/expanding/SfmBundle.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionHistory.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionChunk.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionPolicyLegacy.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionIteration.hpp>
#include <aliceVision/sfm/pipeline/expanding/ExpansionProcess.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;


    int randomSeed = std::mt19937::default_seed;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.")
    ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.");
   
    CmdLine cmdline("AliceVision SfM Expanding");

    cmdline.add(requiredParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    
    // load input SfMData scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (sfmData.getValidViews().size() < 2)
    {
        ALICEVISION_LOG_INFO("Expansion requires that some views are already defined.");
        return EXIT_SUCCESS;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    sfm::SfmBundle::uptr sfmBundle = std::make_unique<sfm::SfmBundle>();
    sfm::ExpansionHistory::sptr expansionHistory = std::make_shared<sfm::ExpansionHistory>();

    sfm::ExpansionChunk::uptr expansionChunk = std::make_unique<sfm::ExpansionChunk>();
    expansionChunk->setBundleHandler(sfmBundle);
    expansionChunk->setExpansionHistoryHandler(expansionHistory);

    sfm::ExpansionPolicy::uptr expansionPolicy = std::make_unique<sfm::ExpansionPolicyLegacy>();
    sfm::ExpansionIteration::uptr expansionIteration = std::make_unique<sfm::ExpansionIteration>();
    expansionIteration->setExpansionHistoryHandler(expansionHistory);
    expansionIteration->setExpansionPolicyHandler(expansionPolicy);
    expansionIteration->setExpansionChunkHandler(expansionChunk);

    sfm::ExpansionProcess::uptr expansionProcess = std::make_unique<sfm::ExpansionProcess>();
    expansionProcess->setExpansionHistoryHandler(expansionHistory);
    expansionProcess->setExpansionIterationHandler(expansionIteration);

    if (!expansionProcess->process(sfmData, tracksHandler))
    {
        ALICEVISION_LOG_INFO("Error processing sfmData");
        return EXIT_FAILURE;
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    

    return EXIT_SUCCESS;
}