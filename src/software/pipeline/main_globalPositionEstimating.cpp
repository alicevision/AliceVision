// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>

#include <aliceVision/sfm/pipeline/positioning/GlobalPositioning.hpp>

#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>
#include <random>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // Command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;

    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(),
         "SfMData output file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(),
         "Tracks file.");

    po::options_description optionalParams("Optional parameters");
    // clang-format on

    CmdLine cmdline("AliceVision Global Rotation Estimating");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    
    // Load input SfMData scene
    ALICEVISION_LOG_INFO("Load sfmData");
    sfmData::SfMData sfmData;
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    
    // Create landmarks from tracks
    ALICEVISION_LOG_INFO("Creating landmarks");
    const track::TracksPerView & tracksPerView = tracksHandler.getTracksPerView();
    const track::TracksMap & tracks = tracksHandler.getAllTracks();
    sfmData::Landmarks & landmarks = sfmData.getLandmarks();

    for (const auto & [idView, sview] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(idView))
        {
            continue;
        }

        sfmData::CameraPose & pose = sfmData.getPoses().at(sview->getPoseId());        
        pose.setRotationOnly(false);

        const track::TrackIdSet & setTracks = tracksPerView.at(idView);
        for (const auto & idTrack : setTracks)
        {
            const track::Track & track = tracks.at(idTrack);
            const track::TrackItem & trackItem = track.featPerView.at(idView);
            
            sfmData::Landmark & landmark = landmarks[idTrack];
            landmark.descType = track.descType;
            
            sfmData::Observation & obs = landmark.getObservations()[idView];
            obs.setCoordinates(trackItem.coords);
            obs.setScale(trackItem.scale);
            obs.setFeatureId(trackItem.featureId);
        }
    }    


    ALICEVISION_LOG_INFO("Processing positioning");
    std::mt19937 gen(randomSeed);
    sfm::GlobalPositioning positionning;
    if (!positionning.process(sfmData, gen))
    {
        ALICEVISION_LOG_ERROR("Positionning failed");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Saving sfmData to " << sfmDataOutputFilename);
    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    return EXIT_SUCCESS;
}